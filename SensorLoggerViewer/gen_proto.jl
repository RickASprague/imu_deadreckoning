#!/usr/bin/env julia
#
# gen_proto.jl — regenerate the Julia protobuf bindings under ./proto_gen from
# the .proto files in ../msgs. Run this after editing any .proto.
#
#   julia +1.11 --project=. gen_proto.jl
#
# proto_gen/ is generated output (gitignored) — this script is the source of
# truth for how it's produced. Add any new .proto to PROTOS below.

using ProtoBuf

const PROTO_ROOT = normpath(joinpath(@__DIR__, "..", "msgs"))
const OUT_DIR    = joinpath(@__DIR__, "proto_gen")

# Paths relative to PROTO_ROOT.
const PROTOS = [
    "linalg/linalg.proto",         # Vec3f / Vec3d / Mat4f (imported by imu, pose)
    "imu/imu.proto",               # ImuSample / ImuBatch
    "pose/pose.proto",             # PoseSample / PoseBatch
]

isdir(OUT_DIR) || mkpath(OUT_DIR)
protojl(PROTOS, PROTO_ROOT, OUT_DIR)
println("regenerated ", OUT_DIR)
