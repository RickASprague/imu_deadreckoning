#!/usr/bin/env julia
#
# world.jl — 3D WORLD-frame viewer for the phone's integrated pose.
#
# Companion to body.jl. Where body.jl fixes the phone at the origin and moves
# the IMU arrows in its frame, this one is the opposite: a fixed world/home
# (the table the phone started flat on, screen-up, at t=0) and we watch the
# *pose* ΔT ∈ SE(3) carry the body frame away from home as it's dead-reckoned.
#
# It listens for `vslam.msgs.PoseBatch` — the integrated ΔT the phone now
# broadcasts (PosePublisher.java) on a SEPARATE port from the IMU stream:
#     body.jl   ← ImuBatch  on 5555
#     world.jl  ← PoseBatch on 5556   (this file)
#
# What you see:
#   • home triad + ground grid at z = 0 — the fixed world frame (dim).
#   • the live body triad — bright x(red)/y(green)/z(blue), at ΔT's (R, p).
#     At t = 0, ΔT = I, so it sits exactly on the home triad. As it drifts it
#     lifts/rotates off home — that separation IS the dead-reckoning drift.
#   • a breadcrumb triad dropped every CRUMB_PERIOD seconds, plus a trail line
#     of the position path, so the drift-over-time is visible at a glance.
#
# Run (screen on + phone streaming):
#   julia +1.11 --project=. world.jl [port]        # default 5556

using GLMakie
using Sockets
using Printf

# Generated protobuf bindings: vslam.msgs.{Mat4f, PoseSample, PoseBatch}
include(joinpath(@__DIR__, "proto_gen", "vslam", "vslam.jl"))
using .vslam.msgs: PoseBatch
import ProtoBuf as PB

const CRUMB_PERIOD = 1.0            # seconds between breadcrumb triads
const AXIS_LEN     = 0.30f0         # body/world triad arrow length (m-ish units)
const CRUMB_LEN    = 0.15f0         # breadcrumb triad arrow length
const GRID_HALF    = 1.5f0          # ground grid spans ±this (metres)
const GRID_STEP    = 0.25f0

decode_batch(bytes) = PB.decode(PB.ProtoDecoder(IOBuffer(bytes)), PoseBatch)

# ---- ΔT = [R p; 0 1] from a row-major Mat4f (16 floats, m[4r + c]). ----------
# Returns (p, cx, cy, cz): body origin in world, and the three body-axis
# directions in world (columns of R), i.e. where x/y/z point after the pose.
function unpack_pose(M::AbstractVector{Float32})
    p  = Point3f(M[4],  M[8],  M[12])          # translation (col 4)
    cx = Vec3f(M[1],  M[5],  M[9])             # R[:,1]  (body x in world)
    cy = Vec3f(M[2],  M[6],  M[10])            # R[:,2]  (body y in world)
    cz = Vec3f(M[3],  M[7],  M[11])            # R[:,3]  (body z in world)
    return p, cx, cy, cz
end

# ---- shared state: written by the UDP task, read by the render loop. ---------
# @async keeps everything on thread 1, so plain Observables/Refs are race-free.
const HOME_O = Point3f(0, 0, 0)

# live body triad (origin + per-axis directions, all live Observables)
const live_o  = Observable([HOME_O])
const live_dx = Observable([Vec3f(AXIS_LEN, 0, 0)])
const live_dy = Observable([Vec3f(0, AXIS_LEN, 0)])
const live_dz = Observable([Vec3f(0, 0, AXIS_LEN)])

# breadcrumb triads — grown one entry per CRUMB_PERIOD. Seed with the home crumb
# so the arrays are never empty (Makie dislikes empty arrows) and home is marked.
const crumb_o  = Observable([HOME_O])
const crumb_dx = Observable([Vec3f(CRUMB_LEN, 0, 0)])
const crumb_dy = Observable([Vec3f(0, CRUMB_LEN, 0)])
const crumb_dz = Observable([Vec3f(0, 0, CRUMB_LEN)])

const trail = Observable([HOME_O])            # position path polyline

const npackets  = Ref(0)
const nposes    = Ref(0)
const first_ns  = Ref(0)                       # first pose t_mono_ns (elapsed base)
const last_ns   = Ref(0)                       # latest pose t_mono_ns
const crumb_ns  = Ref(0)                       # t_mono_ns of last breadcrumb
const arrivals  = Float64[]                    # wall-clock arrival times (Hz est.)
const last_seq  = Ref{Int64}(-1)               # last batch seq; detect session restart

# Reordering tolerance: only a big backward jump (or seq 0) counts as a new
# session, so mild UDP reordering within a session doesn't spuriously reset.
const REORDER_MARGIN = 5

# Wipe the accumulated view (trail, breadcrumbs, live triad, clocks) back to the
# home/table state — called when a new streaming session starts (seq resets).
function reset_state!()
    live_o[]  = [HOME_O]
    live_dx[] = [Vec3f(AXIS_LEN, 0, 0)]
    live_dy[] = [Vec3f(0, AXIS_LEN, 0)]
    live_dz[] = [Vec3f(0, 0, AXIS_LEN)]
    crumb_o[]  = [HOME_O]
    crumb_dx[] = [Vec3f(CRUMB_LEN, 0, 0)]
    crumb_dy[] = [Vec3f(0, CRUMB_LEN, 0)]
    crumb_dz[] = [Vec3f(0, 0, CRUMB_LEN)]
    trail[]    = [HOME_O]
    first_ns[] = 0
    last_ns[]  = 0
    crumb_ns[] = 0
    nposes[]   = 0
    empty!(arrivals)
end

# Decoded batches wait here between the receive task and the render loop.
const posechan = Channel{PoseBatch}(1024)

# Receive task: keep it MINIMAL — recv + decode + enqueue only. It touches no
# Makie state, so it can never be starved by (or block) rendering. Coupling the
# socket read to the scene mutation on one cooperative task was the receive-stall
# bug: once rendering got busy the read task stopped draining the socket.
function udp_loop(port::Int)
    sock = UDPSocket()
    bind(sock, ip"0.0.0.0", port; reuseaddr=true)  # 0.0.0.0 to catch subnet broadcast
    @info "world listening for PoseBatch UDP broadcast" port
    while true
        _, bytes = recvfrom(sock)
        batch = try
            decode_batch(bytes)
        catch
            continue                               # ignore malformed datagrams
        end
        put!(posechan, batch)                      # hand off; do no scene work here
    end
end

# Apply one decoded batch to the scene. Runs on the RENDER task (drained from
# posechan each frame), so all Observable mutation stays on one task.
function process_batch!(batch::PoseBatch)
    npackets[] += 1
    # Session restart: the pose stream resets its batch counter to 0 (ΔT back at
    # identity). Reset the view on seq 0, or on a big backward jump in case the
    # seq-0 datagram was dropped (mild UDP reordering is tolerated).
    sq = batch.seq
    if sq == 0 || sq + REORDER_MARGIN < last_seq[]
        reset_state!()
    end
    last_seq[] = sq
    isempty(batch.samples) && return
    tnow = time()
    for s in batch.samples
        pm = s.pose
        pm === nothing && continue
        length(pm.m) < 16 && continue
        nposes[] += 1
        push!(arrivals, tnow)

        p, cx, cy, cz = unpack_pose(pm.m)

        # live triad follows the newest pose
        live_o[]  = [p]
        live_dx[] = [cx * AXIS_LEN]
        live_dy[] = [cy * AXIS_LEN]
        live_dz[] = [cz * AXIS_LEN]

        # trail: append position (grows a polyline)
        push!(trail[], p); notify(trail)

        # clock bookkeeping
        t = s.t_mono_ns
        first_ns[] == 0 && (first_ns[] = t; crumb_ns[] = t)
        last_ns[] = t

        # breadcrumb every CRUMB_PERIOD of the pose (monotonic) clock
        if (t - crumb_ns[]) * 1e-9 >= CRUMB_PERIOD
            crumb_ns[] = t
            push!(crumb_o[],  p);            notify(crumb_o)
            push!(crumb_dx[], cx * CRUMB_LEN); notify(crumb_dx)
            push!(crumb_dy[], cy * CRUMB_LEN); notify(crumb_dy)
            push!(crumb_dz[], cz * CRUMB_LEN); notify(crumb_dz)
        end
    end
end

# Ground grid on z = 0 as line segments: the table the phone started flat on.
function ground_segments()
    segs = Point3f[]
    g = -GRID_HALF
    while g <= GRID_HALF + 1f-3
        push!(segs, Point3f(g, -GRID_HALF, 0), Point3f(g, GRID_HALF, 0))  # ∥ y
        push!(segs, Point3f(-GRID_HALF, g, 0), Point3f(GRID_HALF, g, 0))  # ∥ x
        g += GRID_STEP
    end
    return segs
end

function build_scene()
    fig = Figure(size = (960, 820), backgroundcolor = :black)
    ls  = LScene(fig[1, 1], show_axis = false)
    scn = ls.scene

    ar  = (; shaftradius = 0.012, tipradius = 0.032, tiplength = 0.11)  # triad arrows
    arc = (; shaftradius = 0.006, tipradius = 0.018, tiplength = 0.06)  # breadcrumbs

    # ground grid (the home/table plane, z = 0)
    linesegments!(scn, ground_segments(); color = (:gray40, 0.5), linewidth = 1)

    # fixed WORLD/home triad at origin — dim, so the bright live triad reads
    # against it. At t = 0 the live triad sits right on top of this.
    arrows3d!(scn, [HOME_O], [Vec3f(AXIS_LEN, 0, 0)]; color = (:red,   0.35), ar...)
    arrows3d!(scn, [HOME_O], [Vec3f(0, AXIS_LEN, 0)]; color = (:green, 0.35), ar...)
    arrows3d!(scn, [HOME_O], [Vec3f(0, 0, AXIS_LEN)]; color = (:blue,  0.35), ar...)
    text!(scn, Point3f(0, 0, AXIS_LEN + 0.08); text = "home", color = :gray70, fontsize = 16)

    # breadcrumb triads (dropped every CRUMB_PERIOD) — small, semi-transparent
    arrows3d!(scn, crumb_o, crumb_dx; color = (:red,   0.6), arc...)
    arrows3d!(scn, crumb_o, crumb_dy; color = (:green, 0.6), arc...)
    arrows3d!(scn, crumb_o, crumb_dz; color = (:blue,  0.6), arc...)

    # position trail
    lines!(scn, trail; color = :orange, linewidth = 2)

    # live body triad — bright, full length, follows the pose
    arrows3d!(scn, live_o, live_dx; color = :red,   ar...)
    arrows3d!(scn, live_o, live_dy; color = :green, ar...)
    arrows3d!(scn, live_o, live_dz; color = :blue,  ar...)

    # legend + hints (top-left, relative space)
    text!(fig.scene, Point2f(0.015, 0.985); text = "bright triad: live pose",
          space = :relative, align = (:left, :top), color = :white, fontsize = 16)
    text!(fig.scene, Point2f(0.015, 0.952); text = "small triads: breadcrumbs (1 s)",
          space = :relative, align = (:left, :top), color = :gray80, fontsize = 15)
    text!(fig.scene, Point2f(0.015, 0.919); text = "orange: position trail",
          space = :relative, align = (:left, :top), color = :orange, fontsize = 15)
    text!(fig.scene, Point2f(0.015, 0.886); text = "drag: rotate   scroll: zoom   right-drag: pan",
          space = :relative, align = (:left, :top), color = :gray60, fontsize = 12)

    hud = Observable("")
    text!(fig.scene, Point2f(0.985, 0.985); text = hud, space = :relative,
          align = (:right, :top), color = :white, fontsize = 18)

    return fig, ls, hud
end

# Look at the z-up world from an isometric-ish angle so lift/rotation is legible.
function look_iso!(ls)
    update_cam!(ls.scene, Vec3f(2.4, -2.4, 1.7), Vec3f(0, 0, 0.15), Vec3f(0, 0, 1))
end

function (@main)(args)
    port = length(args) >= 1 ? parse(Int, args[1]) : 5556
    GLMakie.activate!(title = "Phone World Frame")
    fig, ls, hud = build_scene()
    screen = display(fig)
    look_iso!(ls)

    # display() can briefly report the window as not-open; wait for it to open so
    # the render loop below doesn't return immediately and exit the process.
    for _ in 1:250
        isopen(screen) && break
        sleep(0.02)
    end

    # Receive on a SEPARATE OS thread. On one thread the receive task competes
    # with GLMakie's render loop in the single cooperative scheduler and gets
    # starved — packets never arrive even though the frame loop runs fine.
    # Needs `julia -t 2+`; Threads.@spawn falls back to @async on one thread.
    if Threads.nthreads() < 2
        @warn "started with 1 thread — receive can starve under GLMakie. Launch: julia -t 3 --project=. world.jl"
    end
    Threads.@spawn udp_loop(port)

    # Frame-time instrumentation: dt of each loop iteration, and the worst seen.
    # If the loop is real-time these stay ~16 ms; a stall shows up as a Δt spike
    # to hundreds of ms / seconds; a hang freezes the HUD (infinite frame).
    tframe = time(); frame_dt = 0.0; frame_max = 0.0
    while isopen(screen)
        # measure how long the PREVIOUS iteration took (top-to-top wall time)
        tnow = time()
        frame_dt  = tnow - tframe
        tframe    = tnow
        frame_max = max(frame_max, frame_dt)

        # Drain everything the receive task has decoded since the last frame and
        # apply it here (on this task) — decoupled from the socket read.
        while isready(posechan)
            process_batch!(take!(posechan))
        end
        cutoff = tnow - 1.0
        while !isempty(arrivals) && arrivals[1] < cutoff
            popfirst!(arrivals)
        end
        elapsed = first_ns[] == 0 ? 0.0 : (last_ns[] - first_ns[]) * 1e-9
        ncrumb  = length(crumb_o[]) - 1        # minus the seeded home crumb
        hud[] = @sprintf("elapsed:  %6.1f s\ncrumbs:   %d\npkt #:    %d\nposes:    %d\nrate:     %4.0f Hz\nframe:    %6.1f ms\nframemax: %6.1f ms",
                         elapsed, ncrumb, npackets[], nposes[], length(arrivals),
                         frame_dt * 1e3, frame_max * 1e3)
        sleep(1 / 60)
    end
    return nothing            # main's return value is the exit code; nothing → 0
end
