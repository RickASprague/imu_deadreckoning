#!/usr/bin/env julia
#
# body.jl — 3D body-frame viewer for the phone's IMU stream.
#
# Companion to world.jl: this one is the BODY frame — the phone is fixed at the
# origin and the gyro/accel arrows move in its frame. world.jl is the WORLD
# frame — it listens to the integrated pose and watches the body move (with
# breadcrumbs) against a fixed table/home.
#
# Draws the phone as a point at the origin with its body triad:
#     x → right   (red)
#     y → up      (green)
#     z → out of the screen, toward you (blue)
# We look straight at the phone "face" (camera on +z, up = +y). The LScene
# already carries an interactive cam3d camera, so mouse-drag rotation works out
# of the box — that's the "later, rotate with the mouse" step, free.
#
# The magenta arrow is the body angular rate ω^b_ib (the gyro reading), drawn
# NOT normalized: its length is |ω| in rad/s, in the same units as the unit
# body axes. Still phone ⇒ stub; a flick ⇒ a long arrow along the rotation axis.
#
# HUD (top-right) shows only metadata — display FPS, datagram count (pkt #),
# total samples (|msg|), and the live sample rate in Hz. No raw IMU numbers.
#
# Run (screen on + phone streaming):
#   julia +1.11 --project=. body.jl [port]         # default 5555

using GLMakie
using Sockets
using Printf

# Generated protobuf bindings: vslam.msgs.{Vec3f, Vec3d, ImuSample, ImuBatch}
include(joinpath(@__DIR__, "proto_gen", "vslam", "vslam.jl"))
using .vslam.msgs: ImuBatch
import ProtoBuf as PB

const PORT = length(ARGS) >= 1 ? parse(Int, ARGS[1]) : 5555

decode_batch(bytes) = PB.decode(PB.ProtoDecoder(IOBuffer(bytes)), ImuBatch)

# Accelerometer specific force is ~g when still, so drawn at true length its
# arrow would be ~9.8× the unit axes and shoot off-screen. Scale so 1 g ≈ 1 unit.
const ACCEL_SCALE = 1.0f0 / 9.81f0

# ---- shared state: written by the UDP task, read by the render loop. --------
# @async keeps everything on thread 1, so plain Refs are safe (no data race).
const gyro_obs  = Observable([Vec3f(0, 0, 0)])  # ω^b_ib, body frame [rad/s]
const accel_obs = Observable([Vec3f(0, 0, 0)])  # a^b, body frame, scaled by 1/g
const npackets  = Ref(0)                         # datagrams received
const nsamples  = Ref(0)                         # ImuSamples received
const arrivals  = Float64[]                      # wall-clock arrival times (Hz est.)

function udp_loop(port::Int)
    sock = UDPSocket()
    bind(sock, ip"0.0.0.0", port; reuseaddr=true)   # 0.0.0.0 to catch subnet broadcast
    @info "viewer3d listening for UDP broadcast" port
    while true
        _, bytes = recvfrom(sock)
        batch = try
            decode_batch(bytes)
        catch
            continue                                # ignore any malformed datagram
        end
        npackets[] += 1
        isempty(batch.samples) && continue
        nsamples[] += length(batch.samples)
        tnow = time()
        for _ in batch.samples
            push!(arrivals, tnow)
        end
        s = batch.samples[end]                      # latest sample in the batch
        g = s.gyro
        g === nothing || (gyro_obs[]  = [Vec3f(g.x, g.y, g.z)])
        a = s.accel
        a === nothing || (accel_obs[] = [Vec3f(a.x, a.y, a.z) * ACCEL_SCALE])
    end
end

# Build the scene; returns (figure, hud_observable).
function build_scene()
    fig = Figure(size = (960, 760), backgroundcolor = :black)
    ls  = LScene(fig[1, 1], show_axis = false)
    scn = ls.scene
    O   = Point3f(0, 0, 0)

    # slim arrow proportions (Makie 0.24 arrows3d recipe)
    ar = (; shaftradius = 0.012, tipradius = 0.032, tiplength = 0.11)

    # phone body — a thin slab at the origin (x wide, y tall, z thin), with a
    # slightly proud lighter "screen" on the +z face you're looking at.
    body   = Rect3f(Vec3f(-0.25, -0.5, -0.025), Vec3f(0.5, 1.0, 0.05))
    screen = Rect3f(Vec3f(-0.21, -0.44, 0.025), Vec3f(0.42, 0.88, 0.004))
    mesh!(scn, body;   color = (:gray20, 0.75), transparency = true)
    mesh!(scn, screen; color = (:steelblue, 0.55), transparency = true)

    # body triad, unit length
    arrows3d!(scn, [O], [Vec3f(1, 0, 0)]; color = :red,   ar...)
    arrows3d!(scn, [O], [Vec3f(0, 1, 0)]; color = :green, ar...)
    arrows3d!(scn, [O], [Vec3f(0, 0, 1)]; color = :blue,  ar...)
    text!(scn, Point3f(1.12, 0, 0); text = "x", color = :red,   fontsize = 24)
    text!(scn, Point3f(0, 1.12, 0); text = "y", color = :green, fontsize = 24)
    text!(scn, Point3f(0, 0, 1.12); text = "z", color = :blue,  fontsize = 24)

    # ω^b_ib — length is the magnitude, not normalized (Observable updates live)
    arrows3d!(scn, [O], gyro_obs;  color = :magenta, ar...)
    # a^b — accelerometer specific force, scaled by 1/g (see ACCEL_SCALE)
    arrows3d!(scn, [O], accel_obs; color = :yellow,  ar...)

    # Legend, top-left (relative 0..1 space on the root scene).
    text!(fig.scene, Point2f(0.015, 0.985); text = "magenta: gyro",
          space = :relative, align = (:left, :top), color = :magenta, fontsize = 17)
    text!(fig.scene, Point2f(0.015, 0.950); text = "yellow: accelerometer",
          space = :relative, align = (:left, :top), color = :yellow, fontsize = 17)
    text!(fig.scene, Point2f(0.015, 0.915); text = "drag: rotate   scroll: zoom   right-drag: pan",
          space = :relative, align = (:left, :top), color = :gray70, fontsize = 13)

    # HUD, anchored top-right in relative (0..1) space on the root scene
    hud = Observable("")
    text!(fig.scene, Point2f(0.985, 0.985); text = hud, space = :relative,
          align = (:right, :top), color = :white, fontsize = 18)

    return fig, ls, hud
end

# Point the camera straight at the face: eye on +z, up = +y ⇒ x right, y up,
# z toward the viewer. Call after display() so LScene doesn't reset it.
function look_at_face!(ls)
    update_cam!(ls.scene, Vec3f(0, 0, 4), Vec3f(0, 0, 0), Vec3f(0, 1, 0))
end

function main()
    GLMakie.activate!(title = "Phone Body Frame")
    fig, ls, hud = build_scene()
    screen = display(fig)          # GLMakie Screen — its isopen() is the reliable
    look_at_face!(ls)             # liveness signal (scene.window_open is unset for LScene)

    # display() can briefly report the window as not-open; wait for it to open so
    # the render loop below doesn't return immediately and exit the process.
    for _ in 1:250
        isopen(screen) && break
        sleep(0.02)
    end

    @async udp_loop(PORT)

    # Render/HUD loop: measure display FPS, prune the 1 s Hz window, update HUD.
    tprev = time(); frames = 0; fps = 0.0
    while isopen(screen)
        frames += 1
        tnow = time()
        if tnow - tprev >= 0.5
            fps = frames / (tnow - tprev); frames = 0; tprev = tnow
        end
        cutoff = tnow - 1.0
        while !isempty(arrivals) && arrivals[1] < cutoff
            popfirst!(arrivals)
        end
        rate = length(arrivals)               # samples in the last second ≈ Hz
        hud[] = @sprintf("FPS:   %5.1f\npkt #: %d\n|msg|: %d\nRate:  %4.0f Hz",
                         fps, npackets[], nsamples[], rate)
        sleep(1 / 60)                          # display refresh cap (≠ data rate)
    end
end

# Only launch when run as a script (lets the file be `include`d for testing).
if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
