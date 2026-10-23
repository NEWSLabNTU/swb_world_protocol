# Writing a world server

A simulator becomes a backend of the scenario bridge by serving the
[world protocol](world-protocol.md). Two shells do the protocol's bookkeeping so that a
server only implements its world:

| shell | language | used by |
|---|---|---|
| `swb_world_server` | Python 3.9 or later, `pyzmq`, `protobuf` (3.12 to 7.x) | the Isaac Sim world server ([swb_isaac_world](https://github.com/NEWSLabNTU/swb_isaac_world)) |
| `swb_core::world_server` | Rust | the CARLA world server ([swb_carla_world](https://github.com/NEWSLabNTU/swb_carla_world)) |

A server is done when `swb_world_check conformance` passes against it ("The conformance
check", below).

## What the shells do for you

Both shells:

- check `Hello.protocol_version` and answer `HelloReply` from the world's name, version,
  capabilities, `max_physics_dt` and readiness;
- run the owner lease (one bridge at a time, 15 s, the provably-gone rule) and refuse every
  other request from a non-owner or before a `Hello`;
- remove every entity when a new owner takes the server over (`reset`) and when the owner
  releases it;
- assign handles (nonzero, unique for the process's lifetime) and keep the entity table:
  a second entity of one name, a `Despawn` of an unknown handle and an `Apply` naming an
  unknown handle are refused before the world sees them;
- refuse what the world did not declare: `SetSignals` without `SIGNALS`, a `TRACKED` spawn
  without `TRACKED_MOTION` or `TRACKS_TARGETS`, an `AUTOPILOT` spawn or `SetGoal` without
  `AUTOPILOT`, an `EXTERNAL` non-ego without `AGENT_VEHICLES`, the weather without
  `WEATHER`, and so on;
- refuse a `Step` that carries `poses`, and a `LoadWorld` while the world is not ready;
- turn a world's exception (Python) or `Refusal` (Rust) into a refusal whose text is the
  error, and keep serving.

The Python shell also refuses a `Spawn` whose box dimensions are not all positive and a
`Step` with `substeps < 1` or `dt <= 0`, keeps the clock and the episode, and holds each
`Apply`'s drives for the next `Step`. In the Rust shell the world does these itself.

## The Python shell

`swb_world_server` is a plain Python package, not a ROS package: put the directory
`swb_world_server/` of this repository on `sys.path` (or `PYTHONPATH`). It reads the protocol
from `swb_world_server/swb_world.desc`, a descriptor set of `proto/swb_world.proto`, rather
than generated `_pb2` code, so it works with any protobuf runtime. After editing the
`.proto`, regenerate it with `just proto-desc` (needs `protoc`); the shell's tests fail while
it is stale.

### `World`

Subclass `swb_world_server.world.World`, set its class attributes and implement the core
methods. Poses are `((x, y, z), (qw, qx, qy, qz))` at the SSv2 entity origin, world frame;
twists `((vx, vy, vz), (wx, wy, wz))`, world frame. A method raises to refuse; the message
is the error the bridge passes on to SSv2.

| member | required | |
|---|---|---|
| `simulator_name` | yes | the name in `HelloReply` and the map directory's `<name>/world.yaml` |
| `capabilities` | | short names (`"SIGNALS"`, `"TRACKED_MOTION"`, ...); default none |
| `max_physics_dt` | | the longest physics step, s; 0 (default): SSv2's step whole |
| `simulator_version` | | property, for the log |
| `ready()` | | `(ready, why_not)`; default `(True, "")` |
| `load(world, ground_paths, signals, request=None)` | yes | replace the environment with `world` (empty: the default) and remove every entity. `signals`: dicts with `way_id`, `regulatory_element_ids`, `opendrive_id`, `position` (a tuple or None); empty unless `SIGNALS`. `request`: the `LoadWorld` message |
| `spawn(handle, name, kind, subtype, center, dims, pose, model="", motion="KINEMATIC")` | yes | one body. `kind`: `"EGO"`, `"VEHICLE"`, `"PEDESTRIAN"`, `"MISC_OBJECT"`; `center` and `dims` the declared box; `motion`: `"KINEMATIC"`, `"TRACKED"`, `"EXTERNAL"`. Return None, or a dict for `SpawnReply.body` (below) |
| `despawn(handle)` | yes | remove one body |
| `step(drives, substeps, dt, render, sim_time)` | yes | apply `{handle: Drive}`, run `substeps` physics steps of `dt`, return `(states, contacts)` (below). `sim_time`: elapsed seconds at the step's start |
| `set_signals(colours)` | `SIGNALS` | `{opendrive_id: "OFF" / "RED" / "AMBER" / "GREEN"}`, from the next step on |
| `set_road_condition(friction_scale, wetness, properties)` | `ROAD_CONDITION` | raise `ValueError` listing every problem, and change nothing, for what cannot be applied |
| `set_environment(weather, time_of_day, sim_time=0.0)` | `ENVIRONMENT` | the parts given (dicts; see the docstring); raise `ValueError` as above |
| `set_weather(preset, parameters)` | `WEATHER` | a preset by your simulator's name for it (non-empty: it wins), or `{name: value}` (those not named keep their values); keep it and apply it again after every `load`; return the weather now as `get_weather` does; raise `ValueError`, naming the valid names, for an unknown one |
| `get_weather()` | `WEATHER` | `(preset, {name: value})`: the preset the weather matches (`""` if none) and every parameter, by your names |
| `reset(handles)` | | a new owner or a release: every entity goes, the world stays. Default: `despawn` each |
| `close()` | | the server is stopping |

`Drive` (a named tuple, `swb_world_server.world.Drive`): `pose`, `twist`, `teleport`,
`kinematic`. A `KINEMATIC` body goes to `pose`. A `TRACKED` body takes `twist`, after being
put at `pose` when `teleport`; with `kinematic` it is kinematic from this step on and goes to
`pose`. Only entities whose `EntityPose` set a pose, a twist or `kinematic` are in `drives`:
an `EXTERNAL` body never is.

`step` returns:

- `states`: `[(handle, pose, twist)]` for **every** entity, `twist` None where the simulator
  has none. The pose is at the body's origin (the entity origin unless `spawn` returned an
  `origin_offset`).
- `contacts`: `[(handle, other, other_path, impulse)]`, `other` 0 for the environment.
  Empty unless `CONTACTS`.

The dict `spawn` may return (each key optional):

| key | `Body` field | default |
|---|---|---|
| `model` | `model` | `"<simulator_name>:box"` |
| `box_location` | `box_location` | the declared centre |
| `box_extent` | `box_extent` (half extents) | half the declared dimensions |
| `origin_offset` | `origin_offset` | `(0, 0, 0)` |
| `wheels` | `wheels`: `(front_x, rear_x, track, radius)`, entity frame | unset |
| `spec_hash` | `spec_hash` | empty |

A vehicle server that builds the ego from a sim_vehicle_spec spec should return `wheels`
and `spec_hash`, so the bridge's spawn and start checks can see the car (see
[world-protocol.md](world-protocol.md), "Spawn").

### What the shell hands the world

The shapes the Python shell passes, for what the table above leaves to the docstrings:

- **`load(world, ground_paths, signals, request)`**: `world` is `LoadWorld.world`, which the
  bridge fills from `<map dir>/<simulator_name>/world.yaml` (`world:`, a path relative to the
  map directory or absolute, or a name your server knows; `ground_paths:`, prims bodies rest
  on; `map_origin:`, where your world's origin sits in the map frame). No such file, or an
  empty `world:`: `world` is `""`, your default world. `signals` comes from the Lanelet2
  map's traffic lights, only under `SIGNALS`. `request` is the whole `LoadWorld` (its
  `lanelet2_map_path`, `map_dir`, `map_origin`, `step_time`, ...). The conformance check
  sends `lanelet2_map_path`/`map_dir` from `--map` and no `world`, so its signal check sends
  an empty `SetSignals`.
- **`spawn(...)`** receives the name, kind, subtype, the declared box, the pose, the model
  and the motion; the rest of the `Spawn` request (`role_name`, axles, performance,
  `asset_key`) does not reach a Python world. A server that needs them needs the Rust
  shell, or an extension of this one.
- **`step(drives, ...)`**: `drives` is `{handle: Drive}` for the handles an `Apply` named
  since the last step; a handle not named is not in it (keep its last motion yourself if you
  want one). `Drive` is a named tuple: `pose` (`((x, y, z), (qw, qx, qy, qz))`) or `None`,
  `twist` (`((vx, vy, vz), (wx, wy, wz))`, world frame; `wz` the yaw rate) or `None`,
  `teleport` and `kinematic`, plain bools.
- **`set_road_condition(friction_scale, wetness, properties)`**: `friction_scale` a float
  (1: as the map is); `wetness` one of OpenSCENARIO's levels as the scenario spells them (`"dry"`,
  `"moist"`, `"wetWithPuddles"`, `"lowFlooded"`, `"highFlooded"`; `"dry"` when not given); `properties` a dict
  of str to str, the scenario's `swb.` properties, e.g.
  `{"swb.zone.<zone>.class": "ice"}` for a zone your world defines (refuse a zone it does
  not have, by name, with `ValueError`).
- **`set_environment(weather, time_of_day, sim_time)`**: each argument holds only the parts
  given (a part not given is absent, not `None`), each replacing that part:
  `weather` keys `cloud_cover_oktas` (0 clear to 8 overcast, 9 sky hidden), `sun`
  (`(azimuth, elevation, illuminance)`: rad clockwise from north, rad above the horizon, lx
  with 0 for not given), `fog_visual_range` (m), `precipitation` (`(kind, mm/h)` with kind
  `"dry"`, `"rain"` or `"snow"`), `wind` (`(direction, speed)`: rad counterclockwise from
  +x, the way it blows to; m/s). `time_of_day` keys `hours` (0-24, local), `day_of_year`
  (1-366), `animation` (bool), `date_time` (the scenario's string).
- **`set_weather(preset, parameters)`**: the last request decides: a preset replaces the
  whole weather; parameters change only those named, on top of what is there. "Apply it
  again after every `load`" means the weather as it stands after the last request. An empty
  request (no preset, no parameters) never reaches the world: the shell refuses it with
  `ok: false`; a `ValueError` becomes `WeatherReply.success: false` with its message.

A `World` subclass need not call `World.__init__` (it has none worth calling).

### The clock and the episode

The Python shell keeps `sim_time` (elapsed seconds, starting at 0, adding `substeps * dt`
each step and never reset) and replies `sim_time_ns = sim_ns(sim_time)`, which is
`nanos(sim_time) + BASE_NS`, and `sim_time = sim_time_ns / 1e9`, as the Rust shell's servers
do. Its `episode_id` starts at 1 with the first `LoadWorld` and
goes up by one on each load of a different `world` string; the same world loaded again is
the same episode. A world does nothing for either.

### What the Python shell does not carry

`AUTOPILOT`, `TRACKS_TARGETS` and `READS_BACK` have no `World` method: the shell refuses
`SetGoal` outright, does not pass `EntityPose.ssv2_twist` to the world, and answers every
`Apply` with an empty `ApplyReply` (no read-back, no tracking samples). A Python world must
not declare these three; one that does fails the conformance check (`TRACKS_TARGETS`) or
misleads the bridge. They need the Rust shell, or an extension of this one. (`WEATHER` is
carried: `set_weather` and `get_weather` above; the shell refuses an empty request and turns
a `ValueError` into a `WeatherReply` with `success: false`.) `AGENT_VEHICLES` passes through `spawn`
(`motion="EXTERNAL"` with a kind other than `"EGO"`).

### The entry point

The shell has no `main`: a server writes its own, in three calls from
`swb_world_server.server` and `swb_world_server.lifetime`:

- `lifetime.die_with_parent()`: SIGTERM when the parent dies (a supervisor killed with
  SIGKILL); exits 1 if the parent is already gone. Linux only, a no-op elsewhere. So the
  parent must outlive the server: started as `python3 server.py &` from a script that then
  exits, the server is stopped at once. Run it under a supervisor (play_launch, systemd), in
  the foreground, or as `setsid sh -c 'python3 server.py' &`, which keeps a `sh` as its
  parent.
- `bind(port, host="127.0.0.1")`: the REP socket, bound at once, before the simulator's
  long start. A held port logs the error, waits 30 s (so a respawning supervisor does not
  loop) and exits 1.
- `run(world, socket)`: serve until `Shutdown`, SIGINT or SIGTERM; then close the socket and
  call `world.close()`.

A complete server, core only, that passes the conformance check: it poses kinematic bodies
exactly and leaves everything else where it is.

```python
#!/usr/bin/env python3
import logging
import sys

sys.path.insert(0, "/path/to/swb_world_protocol/swb_world_server")

from swb_world_server import lifetime
from swb_world_server.server import bind, run
from swb_world_server.world import World


class FakeWorld(World):
    simulator_name = "fake"
    capabilities = ()          # core only
    max_physics_dt = 0.0       # takes SSv2's step whole

    def __init__(self):
        self.poses = {}        # handle -> pose

    def load(self, world, ground_paths, signals, request=None):
        self.poses.clear()

    def spawn(self, handle, name, kind, subtype, center, dims, pose, model="",
              motion="KINEMATIC"):
        self.poses[handle] = pose
        return None            # the declared box at the entity origin

    def despawn(self, handle):
        del self.poses[handle]

    def step(self, drives, substeps, dt, render, sim_time):
        for handle, drive in drives.items():
            if drive.pose is not None:
                self.poses[handle] = drive.pose
        return [(h, p, None) for h, p in self.poses.items()], []


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    lifetime.die_with_parent()
    run(FakeWorld(), bind(5599))
```

```bash
python3 fake_world.py &
swb_world_check conformance tcp://127.0.0.1:5599     # 19 passed, 0 failed
```

The shell's own tests (`swb_world_server/test/test_server.py`) drive a similar fake world
through `WorldServer.handle(bytes) -> bytes`, with no socket: the way to unit-test a world.

## The Rust shell

`swb_core::world_server` (crate `swb_core`, a path dependency):

- implement the trait `World`: `simulator_name`, `simulator_version`, `capabilities`,
  `load_world(&LoadWorld) -> LoadWorldReply`, `spawn(handle, &Spawn) -> Body`,
  `despawn`, `apply(&Apply) -> ApplyReply`, `step(&Step) -> StepReply`; and, with their
  capabilities, `set_signals`, `set_goal`, `set_weather`, `get_weather`,
  `set_road_condition`, `set_environment`. Optional: `max_physics_dt`, `ready`,
  `set_step_time`, `reset`, `release` (default: as `reset`), `idle` (called after a while
  without a world request; `Some(reason)` ends the server as `End::Lost`), `shutdown`.
  Every method returns `WorldResult<T>`; `Refusal::new(text)` refuses, `Refusal::fatal(text)`
  refuses and ends the server after the reply (`Reply.fatal`).
- `Shell::new(world)`, `bind(&zmq::Context, endpoint)`, then
  `serve(&socket, &mut shell, &stop_flag, idle_every)`, which returns why it ended
  (`End::Stopped`, `End::Fatal(reason)`, `End::Lost(reason)`). `die_with_parent()` as in
  Python.

The Rust world keeps its own clock and episode: fill `LoadWorldReply.sim_time_ns` and
`StepReply.sim_time_ns` with `swb_core::sim_time::sim_ns(elapsed)`, and both `episode_id`s.
It also holds the `Apply`'s drives for its `step` itself.

## The conformance check

```bash
swb_world_check conformance <endpoint> [--map <lanelet2_map.osm or map dir>]
swb_world_check hello <endpoint> [--wait SECONDS] [--quiet]
swb_world_check rtt <endpoint> [--n 2000] [--gap-us 0]
```

`swb_world_check` is built by this repository (`just build`; `target/dev-release/`) and as
an ament_cargo package. The endpoint is the server's ZMQ address, such as
`tcp://127.0.0.1:5591`.

- `hello`: one probe `Hello`. Exit 0 with the server's description when it answers within
  2 s, 1 when not; `--wait` retries that long. Changes nothing, safe beside a running
  scenario.
- `rtt`: the round trip of N probe `Hello`s (p50, p95, max). Changes nothing.
- `conformance`: takes the server over as a bridge would, runs every check below, releases
  it, and exits 0 when none failed. It moves bodies in the world: run it between scenarios,
  never during one (a server that already has an owner is refused before any check).
  `--map` is the map to load (a CARLA town needs one); without it the server loads its
  default world (`LoadWorld.world` empty).

The checks, in order. Each prints `PASS`, `FAIL` or `skip`; the last line is
`<n> passed, <m> failed`.

| check | passes when |
|---|---|
| a wrong protocol version is refused | a probe `Hello` of version 5 is refused |
| a request before hello is refused | a `Step` from a client that has not said hello is refused |
| hello takes ownership | `Hello` (not a probe) succeeds, and `HelloReply.owner` equals the check's `ClientId` |
| a second bridge is refused while the owner lives | a `Hello` from another `ClientId` (host `swb-world-check-elsewhere`) is refused with an error containing `owned by` |
| a second bridge's requests are refused | a `Step` from that client is refused |
| a probe changes nothing | that client's probe `Hello` is answered with the same owner and the same `instance` |
| (the server becomes ready) | only when the server was not ready: up to 60 s of `Hello`s for `ready: true`, else this fails and the run ends |
| load a world | `LoadWorld` (step time 0.05 s, realtime factor 1, map origin 0, the `--map` paths) answers a `LoadWorldReply`, within 300 s |
| the time is past BASE_NS and the episode named | `LoadWorldReply.sim_time_ns >= BASE_NS` and `episode_id != 0` |
| spawn the ego | `Spawn` of `conformance_ego`, `EGO`, `EXTERNAL`, at (0, 0, 0.5) with the Model 3's box (centre (1.415, 0, 0.744), 4.792 x 2.163 x 1.488 m), model `vehicle.tesla.model3`, answers a nonzero handle |
| spawn a kinematic vehicle | `conformance_npc`, `VEHICLE`, `KINEMATIC`, at (20, 0, 0.5) |
| a second entity of one name is refused | `conformance_npc` again is refused |
| five frames of Apply and Step | five times: `Apply` (the NPC posed 1 m further in x each frame, the ego listed with nothing set) and `Step` (`want_states`) both answer. Each step is 0.05 s, split into `ceil(0.05 / max_physics_dt)` substeps when `max_physics_dt > 0` |
| time advances every step | each `StepReply.sim_time_ns` is greater than the one before, starting from the load's |
| a kinematic body goes where it is posed | the NPC's `x` in the step replies' `states` moved 4 m (within 0.5) from the first frame to the fifth |
| despawning an unknown handle is refused | `Despawn` of handle 999999 is refused |
| a tracked body follows at 5 m/s | with `TRACKS_TARGETS` or `TRACKED_MOTION`: a `TRACKED` vehicle at (60, 0, 0.5) is driven for 20 frames at 5 m/s (by pose and `ssv2_twist`, or by `twist`) and moves more than 2.5 m; under `TRACKS_TARGETS` the 20th `ApplyReply` has a `TrackingSample` for it. Skipped otherwise |
| signal colours are taken / signals are refused without SIGNALS | with `SIGNALS`, an empty `SetSignals` succeeds; without, it is refused |
| the weather is set and read back by parameter | with `WEATHER`: `GetWeather` succeeds with at least one parameter; the check takes the first name in sorted order, sets it alone (`use_parameters: true`) to its value + 1, reads it back within 1e-3 x max(1, abs(value)), then sets all the original parameters back. Skipped otherwise |
| road condition (three checks) | with `ROAD_CONDITION`: friction 0.7 `wetWithPuddles` is taken; a property `swb.zone.swb_check_no_such_zone.class` is refused with the zone's id in the error; friction 1 `dry` is taken. Skipped otherwise |
| a clear noon is taken | with `ENVIRONMENT`: 0 oktas, fog visual range 100 km, dry, 12:00 on day 172 is taken. Skipped otherwise |
| (autopilot) | with `AUTOPILOT`: skipped; it needs a road, and the scenario suite checks it |
| the same world again: the same episode, time going on | the same `LoadWorld` again answers the first load's `episode_id` and a `sim_time_ns` no less than the last step's |
| release | `Release` succeeds |
| no owner after release | a probe `Hello` answers no owner |

A core-only server runs 19 checks. Not checked here: `AGENT_VEHICLES`, `READS_BACK`,
`CONTACTS`, `AUTOPILOT`, and `Body.wheels` and `spec_hash` (the bridge checks those against
the catalog at a real ego's spawn).

Request timeouts: `Hello` 2 s, `Spawn` 20 s, `Apply` 10 s, `Step` 60 s, `LoadWorld` and
`SetWeather` 300 s. A server that does not answer in time fails that check.
