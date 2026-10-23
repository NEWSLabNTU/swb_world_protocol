//! Generated protobuf types from SSv2's `simulation_interface` protocol.
//!
//! All packages are siblings under this module so cross-package references (e.g.
//! `traffic_simulator_msgs` using `super::autoware_vehicle_msgs`) resolve correctly.
//!
//! `dead_code` is allowed throughout: these modules are a transcription of the wire
//! protocol, and a message type this bridge happens not to construct is a normal state, not
//! an unused-code smell. Denying it would make `just check` fail for messages SSv2 defines
//! and we simply do not send.

#[allow(clippy::all, dead_code)]
pub mod simulation_api_schema {
    include!(concat!(env!("OUT_DIR"), "/simulation_api_schema.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod geometry_msgs {
    include!(concat!(env!("OUT_DIR"), "/geometry_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod traffic_simulator_msgs {
    include!(concat!(env!("OUT_DIR"), "/traffic_simulator_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod builtin_interfaces {
    include!(concat!(env!("OUT_DIR"), "/builtin_interfaces.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod autoware_control_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_control_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod autoware_vehicle_msgs {
    include!(concat!(env!("OUT_DIR"), "/autoware_vehicle_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod rosgraph_msgs {
    include!(concat!(env!("OUT_DIR"), "/rosgraph_msgs.rs"));
}

#[allow(clippy::all, dead_code)]
pub mod std_msgs {
    include!(concat!(env!("OUT_DIR"), "/std_msgs.rs"));
}

// SSv2's protos above are the fork's files byte for byte; the scenario bridge, which holds
// the fork, checks that (scenario-world-bridge `just test`).

/// swb's world protocol with the Isaac Sim world server (`proto/swb_world.proto`, roadmap
/// 037): swb's own, not SSv2's.
#[allow(clippy::all, dead_code)]
pub mod swb_world {
    include!(concat!(env!("OUT_DIR"), "/swb_world.rs"));
}
