# The world protocol

How the scenario bridge (scenario-world-bridge's `RemoteBackend`) drives a simulator's
**world server**. The bridge knows no simulator: everything it asks of one goes over this
protocol, and a simulator becomes a backend by serving it. The definition is
[`proto/swb_world.proto`](../proto/swb_world.proto) (package `swb_world`); this page says
what the messages mean together. How to write a server, and the conformance check that
says one is done: [world-server.md](world-server.md).

## Transport

- ZMQ, TCP. The server binds a `REP` socket; the bridge connects a `REQ` socket (relaxed and
  correlated, so after a timeout it can send again and a late reply to the old request is
  dropped). The servers of this project bind `tcp://127.0.0.1:<port>`.
- One `Request` in, one `Reply` out, strictly alternating. Nothing is pushed: the server only
  ever answers.
- Every `Request` carries one request in its `oneof` and the sender's `ClientId`
  (`client`, field 100), on `Hello` too.
- Every `Reply` has `ok`. `ok: false` is a refusal: `error` says why, in words the bridge
  passes on to SSv2. `fatal: true` means the server cannot go on (a hung simulator call, its
  simulator gone) and exits after this reply; the bridge ends as after a request that timed
  out. A refusal is not fatal unless it says so: the server keeps serving.
- An undecodable request, or one with no request set, is refused (`ok: false`), not dropped.

`PROTOCOL_VERSION` is **4**. It is defined in three places that must agree:
`swb_core::world_client::PROTOCOL_VERSION` (used by the bridge, the Rust shell and
`swb_world_check`) and `swb_world_server/protocol.py`'s `PROTOCOL_VERSION`. Bump it on any
change a peer of the other version would misread. A field added for a capability the bridge
only uses when the server declares it needs no bump.

## Frames and units

- The world frame is right-handed and Z-up, in metres and radians (ROS's). A simulator with
  another convention (CARLA: left-handed) converts inside its server.
- The bridge subtracts the map origin (`<map dir>/<simulator>/world.yaml`) before it sends
  a pose. The server therefore works in world coordinates. It sees map coordinates only when
  it reads map files itself, and `LoadWorld.map_origin` tells it where the world origin sits
  in the map frame.
- Poses (`Pose`: `position`, `orientation` as a quaternion x, y, z, w) are at the **SSv2
  entity origin**. For a vehicle that is the rear axle centre on the ground. The declared box
  sits around that origin (`Spawn.box_center` is the box centre from it).
- `Pose.rpy`, when set, is roll, pitch and yaw in radians, authoritative over the quaternion.
  A server that holds orientation as Euler angles sets it so a reader does not round-trip
  through the quaternion.
- A `Twist` is world frame at the entity origin unless the field says otherwise
  (`EntityPose.ssv2_twist` and `ReadBack.twist` are in the entity's own frame, x forward).

## Ownership: one bridge at a time

A server is driven by one client at a time, its **owner**.

- A `Hello` with `probe: true` is a health check. It is answered, and nothing changes: no
  ownership taken, no lease refreshed. `swb_world_check hello` and the bridge's idle checks
  are probes.
- A `Hello` with `probe: false`:
  - with no owner, makes the sender the owner, and every entity the server still has is
    removed (the world stays loaded);
  - from the owner, refreshes its lease;
  - from anyone else, takes the server over only when the owner is **provably gone** or has
    been **silent longer than the lease** (15 s; `LEASE` in the Rust shell, `LEASE_SECONDS`
    in the Python shell). Otherwise it is refused with `owned by <host>:<pid> ...`.
- "Provably gone": the owner's `host` is this host, and its `pid` either does not run or
  runs with a start time other than `ClientId.start_time` (`/proc/<pid>/stat` field 22,
  clock ticks since boot; 0 means unknown and is never taken as a mismatch). A client on
  another host is never provably gone; only its lease lapses.
- Every request other than `Hello` must come from the owner. With no owner it is refused
  (`<request> before hello: say hello first`); from another client, refused
  (`<request> from <client> refused: this world server is owned by <owner>`). Each request
  from the owner refreshes its lease.
- `Release`: the owner leaves. The server undoes what it did for it (entities destroyed, the
  simulator's settings restored) and has no owner after. The world stays loaded.
- `HelloReply.owner` is the owner after this `Hello` (unset: none). A client knows it owns
  the server when that equals its own `ClientId`, field for field.

## Capabilities

The **core** is what every server does: `Hello`, `LoadWorld`, `Spawn` of `KINEMATIC` bodies
and of the ego as `EXTERNAL`, `Apply`, `Step`, `Despawn`, `SetStepTime`, `Release`,
`Shutdown`. Everything else is declared in `HelloReply.capabilities`. The bridge never sends a
request or a drive a server did not declare. It refuses SSv2's request instead, naming the
missing capability. A server refuses a request for a capability it did not declare (both
shells do this for you).

| capability | the server does | requests and fields |
|---|---|---|
| `SIGNALS` | holds traffic lamps at the colours it is told | `LoadWorld.signals`, `SetSignals` |
| `AUTOPILOT` | its own traffic drives a vehicle toward a goal | `Motion.AUTOPILOT`, `SetGoal` |
| `AGENT_VEHICLES` | spawns `EXTERNAL` vehicles other than the ego, driven by another client through the simulator | `Spawn` with `motion: EXTERNAL`, `kind` not `EGO` |
| `TRACKED_MOTION` | moves `TRACKED` bodies by the velocity the bridge computes | `EntityPose.twist`, `teleport`, `kinematic` |
| `TRACKS_TARGETS` | tracks `TRACKED` bodies itself from SSv2's pose and twist, and reports each one's tracking | `EntityPose.pose`, `ssv2_twist`; `ApplyReply.tracking` |
| `READS_BACK` | reads back what it moves (the ego, `EXTERNAL`, `AUTOPILOT` bodies) as SSv2 wants them | `ApplyReply.read_back` |
| `WEATHER` | a weather by preset or by named parameters | `SetWeather`, `GetWeather` |
| `CONTACTS` | reports contacts of reporting bodies | `StepReply.contacts`, `ApplyReply.contacts` |
| `ROAD_CONDITION` | OpenSCENARIO's road condition, world-wide and per zone | `SetRoadCondition` |
| `ENVIRONMENT` | OpenSCENARIO's weather and time of day | `SetEnvironment` |

A `Spawn` with `motion: TRACKED` needs `TRACKED_MOTION` or `TRACKS_TARGETS`; `AUTOPILOT`
needs `AUTOPILOT`; `EXTERNAL` for a kind other than `EGO` needs `AGENT_VEHICLES`.

## The requests

### Hello -> HelloReply

`Hello.protocol_version` must equal the server's, or the request is refused with both
versions in the message. The reply:

| field | |
|---|---|
| `protocol_version` | the server's (the client refuses a reply in another version) |
| `simulator_version` | for the log |
| `simulator_name` | `carla`, `isaac`, ...: the map directory's `<name>/world.yaml` is this server's |
| `capabilities` | as above |
| `owner` | the owner after this `Hello`, unset when none |
| `instance` | unique per server process. A bridge that sees it change knows the server restarted and every entity it had is gone |
| `max_physics_dt` | the longest physics step the server takes, s. The bridge splits each SSv2 step into `n = ceil(step / max_physics_dt)` substeps of `step / n`. 0: the server takes SSv2's step whole |
| `ready`, `not_ready_reason` | whether the server can load a world now (CARLA: connected to its server) |

### LoadWorld -> LoadWorldReply

SSv2's `Initialize`. It replaces the world: every entity is gone after it. A server that is
not `ready` refuses it.

| field | |
|---|---|
| `world` | the world as the server's `world.yaml` names it (a CARLA town, a USD stage). Empty: the server's default for the map |
| `ground_paths` | the environment a body rests on; contacts with it are not reported |
| `signals` | the map's traffic signals (only to a `SIGNALS` server); every lamp green until `SetSignals` |
| `map_dir`, `lanelet2_map_path`, `map_origin` | SSv2's map, for a server that reads map files |
| `step_time`, `realtime_factor` | SSv2's `Initialize` |

The reply carries the time the first step starts from (`sim_time_ns`) and the episode the
load began (`episode_id`); see "Time and episodes".

### Spawn -> SpawnReply

One entity: a body at `pose` with a box collider, moving as `motion` says.

| field | |
|---|---|
| `name` | unique among the server's entities: a second of one name is refused |
| `kind` | `EGO`, `VEHICLE`, `PEDESTRIAN`, `MISC_OBJECT` |
| `subtype` | SSv2's (`car`, `truck`, ...), for the look; may be empty |
| `box_center`, `box_dimensions`, `box_declared` | the declared box (centre from the entity origin; length, width, height). The Python shell refuses dimensions that are not all positive |
| `pose` | world frame, entity origin |
| `model`, `asset_key` | the look: SSv2's asset key after and before the bridge's asset map. A CARLA blueprint id (`vehicle.tesla.model3`) is what both servers of this project know. Empty or unknown: the server's default for the kind |
| `motion` | `KINEMATIC` (posed every step, pushed by nothing), `TRACKED`, `EXTERNAL` (the simulator's physics moves it and something else drives it: read back, never posed), `AUTOPILOT` |
| `role_name` | the name the simulator knows the vehicle by, for whoever attaches to it (the ego's role name) |
| `axles`, `performance` | as SSv2 declared them |

`SpawnReply.handle` is nonzero and unique for the server's lifetime (both shells assign it).
`SpawnReply.body` is the body the server chose:

| field | |
|---|---|
| `sim_id` | the simulator's id for it (CARLA's actor id); 0 when there is none |
| `model` | what the body is: a blueprint id, `isaac:box`, ... |
| `box_location`, `box_extent`, `has_box` | the model's own box: centre and **half** extents in the body's frame |
| `origin_offset`, `origin_yaw` | from the entity origin to the body's origin (entity frame), and the body's turn from the entity's heading |
| `declared_center`, `declared_dimensions` | the declared box as the server took it |
| `wheels` | a vehicle's wheels as the server knows them, entity frame: `front_x`, `rear_x` (axle x from the entity origin; 0 when the rear axle is the origin, as SSv2's is), `track`, `radius`. Unset when unknown |
| `spec_hash` | the vehicle spec (sim_vehicle_spec's `spec_hash`) the server built the body from: the rig or the physics it applied. Empty when none |

What the bridge does with the last two, for the ego (roadmap 051):

- **The spawn check**: it compares the body with what SSv2 declared (the vehicle catalog's
  car) and refuses the ego when a field is more than 2 cm off, naming each field with both
  values. The box is compared when the server reported one, the wheels when both sides know
  them, and the box's y by size only.
- **The start check**: it compares `spec_hash` with the vehicle agent's and with the
  installed catalog entry's (`Property name="swb.spec_hash"`) and refuses the ego when two
  differ, naming who loaded which. A part that reports none is left out.

### Despawn

Removes one entity. An unknown handle is refused.

### Apply -> ApplyReply

SSv2's `UpdateEntityStatus`: every entity's drive for the coming step, as `EntityPose`s.
Handles the server does not know are refused (the whole request). The drive of each motion:

- `KINEMATIC`: goes to `pose`.
- `TRACKED` under `TRACKED_MOTION`: takes `twist` before the substeps, its pose ignored
  unless `teleport` (then first put at `pose`); with `kinematic` it becomes kinematic from
  this step on (the bridge's fallback for a blocked body) and goes to `pose`.
- `TRACKED` under `TRACKS_TARGETS`: given SSv2's `pose` and `ssv2_twist`; the server
  decides how to follow.
- `EXTERNAL` and `AUTOPILOT`: listed with nothing set, so the server knows SSv2 still has them
  this frame.

The reply: `read_back` (`READS_BACK`: pose, twist, acceleration and jerk of each body the
server moves), `tracking` (`TRACKS_TARGETS`: one `TrackingSample` per tracked body),
`failure` (what failed, for SSv2's reply; empty when nothing did) and `contacts` learned
since the last reply (a simulator that reports them asynchronously, belonging to the last
step). The drives take effect at the next `Step`.

### Step -> StepReply

SSv2's `UpdateFrame`: advance the world by `substeps` physics steps of `dt` seconds. Both
shells refuse a `Step` that carries `poses` (drives come by `Apply` since protocol 4); the
Python shell also refuses `substeps < 1` or `dt <= 0`. `render`: whether to render this step. `want_states`: the reply
must carry every entity's state; a server that has them anyway may always send them.

| reply field | |
|---|---|
| `sim_time_ns` | simulation time after the step, ns, `nanos(elapsed) + BASE_NS`: required (a server that leaves it 0 fails the conformance check) |
| `sim_time` | the same time in seconds, `sim_time_ns / 1e9` (`BASE_NS` included), for the log and the bridge's frame interval; use `sim_time_ns` for time |
| `frame` | the server's frame counter after the step |
| `episode_id` | the episode the step is in (`LoadWorldReply`'s); 0 from a server that does not say |
| `states` | every entity's `EntityState`: pose after the step, world frame, at the **body's** origin (`Body.origin_offset` from the entity origin); `twist`, the body's velocity, where the server has it |
| `contacts` | `CONTACTS`: a reporting body (the ego, a tracked body) touching another at any substep, once per pair and reporting side. `other` is the other entity's handle, 0 for the environment or a body the bridge did not spawn; `impulse` the total normal impulse over the step, N s |
| `tick_seconds` | the time the server spent stepping the simulator, s, for the bridge's frame budget |

### SetStepTime

SSv2's `UpdateStepTime`. Core: a server may ignore it (the Python shell does), since every
`Step` names its `dt`.

### SetSignals (`SIGNALS`)

Colours (`OFF`, `RED`, `AMBER`, `GREEN`) by OpenDRIVE signal id, from the next step on.
Lamps not named keep theirs.

### SetGoal (`AUTOPILOT`)

SSv2's `UpdateEntityGoal` for an `AUTOPILOT` body: waypoints (the last is the goal),
`clear`, and an optional target speed.

### SetWeather, GetWeather -> WeatherReply (`WEATHER`)

A generic weather: the names belong to the server. The CARLA world server takes CARLA's
presets and the fields of CARLA's `WeatherParameters`; another server defines its own.

- `Weather`: `preset` (a name), or `parameters` (name to value) with `use_parameters: true`.
- `SetWeather` with a preset applies it. With parameters it changes **only the parameters
  named**; the others keep their current values.
- `GetWeather` returns the weather in effect: `WeatherReply.success: true`, and in
  `weather` every parameter the server has, by its own name (this is how a client learns
  the names), and the preset it is, or empty when the parameters match none.
- `WeatherReply.success: false` with a `message` is a weather the server could not apply;
  `ok: false` is a request it refused.
- A world load resets the weather to the new world's own. The CARLA world server holds the
  weather last asked for and applies it again after every load.

The bridge exposes this as ROS services (`/scenario_world_bridge/set_weather`,
`get_weather`) and passes requests through.

### SetRoadCondition (`ROAD_CONDITION`)

SSv2's `UpdateEnvironment` road condition: the whole road state from now on, replacing the
last one. `friction_scale_factor` scales every surface's friction; `wetness` is one of
OpenSCENARIO's levels (empty: dry); `properties` holds only `swb.` properties, where
`swb.zone.<id>.<field>` sets a field of a zone of the map's `surfaces.yaml`. A server refuses
a property it does not know or cannot apply, listing every problem (a zone it does not have
by its id), and then changes nothing. `LoadWorld` and `Release` go back to the default.

### SetEnvironment (`ENVIRONMENT`)

SSv2's `UpdateEnvironment` weather and time of day. Every part (`cloud_cover`, `sun`, `fog`,
`precipitation`, `wind`, `time_of_day`) is optional: a part not set is left as it was. A
server refuses, listing every problem, a part it cannot show, and then changes nothing.
`LoadWorld` and `Release` go back to the world's own sky.

### Release, Shutdown

`Release`: see "Ownership". `Shutdown`: the server stops serving after this reply.

## Time and episodes

Simulation time is one integer, the same in every process that holds the frame (roadmap
051):

```text
sim_time_ns = nanos(elapsed_seconds) + BASE_NS
BASE_NS     = 1_000_000_000_000          (1000 s)
nanos(s)    = round(s * 1e9), half away from zero
```

`elapsed_seconds` is the simulator's time into its episode. `BASE_NS` keeps ROS time far from
0, which ROS reads as "no clock yet", and from the small values for which `now() - duration`
goes negative. The rounding is Rust's `f64::round`, not Python's `round()` (half to even,
1 ns apart on about 1.4 % of CARLA frames). Implementations: `swb_core::sim_time` (`BASE_NS`,
`nanos`, `sim_ns`) and `swb_world_server.protocol` (`BASE_NS`, `nanos`, `sim_ns`).
Nothing about time is shared, stored or sent beyond the replies: any process computes it from
the frame it holds.

An **episode** is one world: from its load until the next load of another world, or the
server's death.

- `LoadWorldReply.episode_id` is nonzero, and differs from the last one whenever the world
  was loaded anew. The CARLA world server's is CARLA's world id. The Python shell counts
  loads of a different world (`LoadWorld.world` changed): the same world loaded again is the
  same episode.
- `StepReply.episode_id` repeats the episode the step is in.
- `sim_time_ns` is monotonic **within** an episode: every step's is greater than the last,
  and the same world loaded again keeps it going on. Across episodes it may start again. A
  server may keep it running across episodes (the Isaac world server and the Python shell
  do), but nothing may depend on that.
- `LoadWorldReply.sim_time_ns` is the time the first step starts from, and is at least
  `BASE_NS`.
- When the episode changes, the bridge logs it. Every vehicle stack of the old episode ends
  (its vehicle bridge exits before publishing a frame of the new one) and is restarted on the
  new one by whatever supervises it.

## SSv2's message types

`proto/` also holds SSv2's `simulation_interface` protos (`simulation_api_schema`,
`geometry_msgs`, `traffic_simulator_msgs`, `builtin_interfaces`, `autoware_control_msgs`,
`autoware_vehicle_msgs`, `rosgraph_msgs`, `std_msgs`). They are not part of the world
protocol on the wire. `swb_core`'s `SimBackend` seam and its shared world policy are typed in
them. They must be byte-identical to the SSv2 fork's (`scenario_simulator_v2`,
`simulation/simulation_interface/proto/`): a field added to one copy and not the other
decodes as silence, not as an error. scenario-world-bridge's `just test` checks this
(`scripts/test_ssv2_protos.py`), since that repository holds both.
