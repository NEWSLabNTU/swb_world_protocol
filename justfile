# swb_world_protocol: the world protocol, its Rust and Python server shells, the world client
# and the conformance check. No ROS needed to build or test it.

default:
    @just --list

# Build the Rust workspace (swb_core, swb_world_check)
build:
    cargo build --profile dev-release

# Rust and Python tests
test:
    cargo nextest run --no-tests pass
    python3 -m unittest discover -s swb_world_server/test

# Format check and clippy
check:
    cargo fmt --all -- --check
    cargo clippy --all-targets -- -D warnings

# Regenerate the Python shell's descriptor set after editing proto/swb_world.proto
proto-desc:
    swb_world_server/scripts/gen_world_proto.sh
