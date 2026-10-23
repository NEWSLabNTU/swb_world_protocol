//! Simulation time, the same integer in every process (roadmap 051, decision 5):
//!
//! ```text
//! sim_ns = nanos(elapsed_seconds) + BASE_NS
//! ```
//!
//! `elapsed_seconds` is the simulator's time into its episode, from the frame a process holds;
//! nothing is shared, stored or sent, so a restarted bridge, world server or vehicle bridge
//! needs no state to agree. `BASE_NS` keeps ROS time far from 0 (ROS's "no clock yet") and from
//! the small values for which `now() - duration` goes negative in tf2 and Autoware's monitors.
//!
//! Time is monotonic within an episode (one simulator world, from its load to the next load
//! or the server's death) and only there: an episode change ends every vehicle stack, which
//! comes back on the new one (docs/design/time-and-ticking.md, "Episodes and vehicle stacks").
//! The world protocol reports the episode beside the time (`episode_id`).
//!
//! acb_bridge computes the same with its own copy of these two (it depends on no swb
//! repository); scenario-world-bridge's `scripts/test_sim_time.py` checks they agree.

/// 1000 s, in ns: added to every simulation time.
pub const BASE_NS: i64 = 1_000_000_000_000;

/// Seconds to integer nanoseconds: `round(secs * 1e9)`, half away from zero (`f64::round`;
/// not Python's `round`, which rounds half to even and differs by 1 ns on ~1.4 % of CARLA
/// frames).
pub fn nanos(secs: f64) -> i64 {
    (secs * 1e9).round() as i64
}

/// The simulation time of a frame `elapsed` seconds into its episode, ns.
pub fn sim_ns(elapsed: f64) -> i64 {
    nanos(elapsed) + BASE_NS
}

/// Nanoseconds as `s.nnnnnnnnn` for log lines, without going through an f64.
pub fn fmt_ns(ns: i64) -> String {
    let sign = if ns < 0 { "-" } else { "" };
    let a = ns.unsigned_abs();
    format!("{sign}{}.{:09}", a / 1_000_000_000, a % 1_000_000_000)
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// `(seconds, ns)`: the rounding acb_bridge's `clock::nanos` must match, case for case.
    pub(crate) const NANOS_CASES: &[(f64, i64)] = &[
        (0.0, 0),
        (0.05000000074505806, 50_000_001),
        (5e-10, 1),
        (1.5e-9, 2),
        (2.5e-9, 3),
        (-2.5e-9, -3),
        (3.5e-9, 4),
        (0.10000000149011612, 100_000_001),
        (0.15000000223517418, 150_000_002),
        (67.0000009983778, 67_000_000_998),
        (171.60000255703926, 171_600_002_557),
        (1000.0000149011612, 1_000_000_014_901),
        (237000.05000000075, 237_000_050_000_001),
        (237171.60000255704, 237_171_600_002_557),
        (1790611998.05, 1_790_611_998_049_999_872),
    ];

    #[test]
    fn nanos_matches_the_shared_table() {
        for &(secs, ns) in NANOS_CASES {
            assert_eq!(nanos(secs), ns, "nanos({secs:?})");
        }
    }

    #[test]
    fn sim_time_is_the_frame_plus_the_base() {
        assert_eq!(sim_ns(0.0), 1_000_000_000_000);
        assert_eq!(sim_ns(0.05000000074505806), 1_000_050_000_001);
        assert_eq!(fmt_ns(sim_ns(171.60000255703926)), "1171.600002557");
        assert_eq!(fmt_ns(-1_500_000_000), "-1.500000000");
    }
}
