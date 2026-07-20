#!/usr/bin/env bash
#
# run-world.sh — launch the world-frame pose viewer with the threads it needs.
# world.jl runs the UDP receive on a separate OS thread (Threads.@spawn) so
# GLMakie's render loop can't starve it — that REQUIRES julia -t >= 2, else the
# spawn falls back to a cooperative task and no packets are received.
#
#   scripts/run-world.sh [port]        # default 5556
#
set -euo pipefail
DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../SensorLoggerViewer" && pwd)"
cd "$DIR"
exec julia +1.11 -t 3 --project=. world.jl "$@"
