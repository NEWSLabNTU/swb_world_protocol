# swb_world_protocol

The world protocol of [scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge):
how the scenario bridge drives a simulator's **world server**, whatever the simulator.

| Path | What |
|---|---|
| `proto/swb_world.proto` | The protocol: Hello, LoadWorld, Spawn, Step, the owner lease, capabilities |
| `proto/*.proto` (the rest) | SSv2's `simulation_interface` message types, which the shared world code is typed in; byte-identical to SSv2's, checked by scenario-world-bridge |
| `swb_core/` | Rust: the world client, the world server shell (`WorldServer`, loop, lease), the `SimBackend` seam and the world policy every Rust world server shares (entities, NPC tracking, gap trace, contacts, weather, maps) |
| `swb_world_server/` | Python: the world server shell (`World` base class, lease, capabilities); `isaac_world` is built on it |
| `swb_world_check/` | `swb_world_check hello|conformance|rtt <endpoint>`: probe or conformance-check any world server |

A new simulator becomes a backend by implementing a world server on either shell and passing
`swb_world_check conformance`.

```bash
just build   # cargo, no ROS
just test    # Rust and Python
just check   # fmt + clippy
```

The CARLA and Isaac Sim world servers are
[swb_carla_world](https://github.com/NEWSLabNTU/swb_carla_world) and
[swb_isaac_world](https://github.com/NEWSLabNTU/swb_isaac_world).
