//! A weather as the world protocol carries it: a preset by name, or parameters by name
//! (roadmap 051). What the names mean belongs to the world server -- CARLA's world server
//! takes CARLA's presets and `WeatherParameters` fields -- so nothing here knows a
//! simulator's weather; the bridge's `/scenario_world_bridge/set_weather` and `get_weather`
//! pass a request through and show the reply.
//!
//! Weather is world state, not scenario state: SSv2 does not command it (its
//! `EnvironmentAction` is parse-only), and a world load resets it to the new world's own.
//! The world server holds the weather asked for -- the `weather` parameter, then the last
//! `set_weather` -- and applies it again after every world load.

use std::collections::BTreeMap;

/// A weather: the preset it is (or matches), and its parameters by name.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Weather {
    /// The preset's name; empty when the parameters match none.
    pub preset: String,
    pub parameters: BTreeMap<String, f64>,
}

/// What a `set_weather` request asks for.
#[derive(Debug, Clone, PartialEq)]
pub enum WeatherRequest {
    /// A preset, by the world server's name for it.
    Preset(String),
    /// Parameters by name; those not named keep the world's current values.
    Parameters(BTreeMap<String, f64>),
}

/// Decode a request from the service's fields: a non-empty `preset` wins; otherwise one value
/// per name, at least one. An empty request is refused rather than read as "every parameter
/// 0".
pub fn decode_request(
    preset: &str,
    names: &[String],
    values: &[f64],
) -> Result<WeatherRequest, String> {
    let preset = preset.trim();
    if !preset.is_empty() {
        return Ok(WeatherRequest::Preset(preset.to_string()));
    }
    if names.len() != values.len() {
        return Err(format!(
            "{} parameter names but {} values; give one value per name",
            names.len(),
            values.len()
        ));
    }
    if names.is_empty() {
        return Err(
            "give a preset, or parameters by name (names and values); the world server's \
             get_weather lists its parameters"
                .into(),
        );
    }
    let mut parameters = BTreeMap::new();
    for (n, v) in names.iter().zip(values) {
        let n = n.trim();
        if n.is_empty() {
            return Err("an empty parameter name".into());
        }
        if parameters.insert(n.to_string(), *v).is_some() {
            return Err(format!("parameter '{n}' given twice"));
        }
    }
    Ok(WeatherRequest::Parameters(parameters))
}

/// The answer to either weather service (the world protocol's `WeatherReply`).
#[derive(Debug, Clone, PartialEq)]
pub struct WeatherReply {
    pub success: bool,
    pub message: String,
    /// The weather now in effect, when known: every parameter the server has.
    pub weather: Option<Weather>,
}

impl WeatherReply {
    pub fn ok(weather: Weather, message: impl Into<String>) -> Self {
        Self {
            success: true,
            message: message.into(),
            weather: Some(weather),
        }
    }

    pub fn failed(message: impl Into<String>, weather: Option<Weather>) -> Self {
        Self {
            success: false,
            message: message.into(),
            weather,
        }
    }
}

/// The world protocol's form of a weather and of the requests and replies about it.
pub mod wire_form {
    use super::*;
    use crate::proto::swb_world as wire;

    /// A weather as the protocol carries it: its parameters, and the preset it matches.
    pub fn weather(w: &Weather) -> wire::Weather {
        wire::Weather {
            preset: w.preset.clone(),
            parameters: w.parameters.iter().map(|(k, v)| (k.clone(), *v)).collect(),
            use_parameters: true,
        }
    }

    pub fn weather_of(w: &wire::Weather) -> Weather {
        Weather {
            preset: w.preset.clone(),
            parameters: w.parameters.iter().map(|(k, v)| (k.clone(), *v)).collect(),
        }
    }

    pub fn request(r: &WeatherRequest) -> wire::Weather {
        match r {
            WeatherRequest::Preset(name) => wire::Weather {
                preset: name.clone(),
                parameters: Default::default(),
                use_parameters: false,
            },
            WeatherRequest::Parameters(p) => wire::Weather {
                preset: String::new(),
                parameters: p.iter().map(|(k, v)| (k.clone(), *v)).collect(),
                use_parameters: true,
            },
        }
    }

    /// The request a protocol `Weather` makes: its preset unless `use_parameters`.
    pub fn request_of(w: &wire::Weather) -> Result<WeatherRequest, String> {
        if w.use_parameters {
            let names: Vec<String> = w.parameters.keys().cloned().collect();
            let values: Vec<f64> = names.iter().map(|n| w.parameters[n]).collect();
            decode_request("", &names, &values)
        } else {
            decode_request(&w.preset, &[], &[])
        }
    }

    pub fn reply(r: &WeatherReply) -> wire::WeatherReply {
        wire::WeatherReply {
            success: r.success,
            message: r.message.clone(),
            weather: r.weather.as_ref().map(weather),
        }
    }

    pub fn reply_of(r: &wire::WeatherReply) -> WeatherReply {
        WeatherReply {
            success: r.success,
            message: r.message.clone(),
            weather: r.weather.as_ref().map(weather_of),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn names(n: &[&str]) -> Vec<String> {
        n.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn a_preset_wins_and_is_passed_as_given() {
        assert_eq!(
            decode_request(" clear_noon ", &names(&["cloudiness"]), &[3.0]),
            Ok(WeatherRequest::Preset("clear_noon".into()))
        );
    }

    #[test]
    fn parameters_come_by_name() {
        let r = decode_request("", &names(&["cloudiness", "wetness"]), &[30.0, 5.0]).unwrap();
        let WeatherRequest::Parameters(p) = r else {
            panic!("{r:?}")
        };
        assert_eq!(p["cloudiness"], 30.0);
        assert_eq!(p["wetness"], 5.0);
    }

    #[test]
    fn a_malformed_request_is_refused() {
        let e = decode_request("", &[], &[]).unwrap_err();
        assert!(e.contains("give a preset"), "{e}");
        let e = decode_request("", &names(&["a", "b"]), &[1.0]).unwrap_err();
        assert!(e.contains("2 parameter names but 1 values"), "{e}");
        let e = decode_request("", &names(&["a", "a"]), &[1.0, 2.0]).unwrap_err();
        assert!(e.contains("'a' given twice"), "{e}");
        let e = decode_request("", &names(&[" "]), &[1.0]).unwrap_err();
        assert!(e.contains("empty parameter name"), "{e}");
    }

    #[test]
    fn the_wire_form_round_trips() {
        let p = WeatherRequest::Preset("ClearNoon".into());
        assert_eq!(wire_form::request_of(&wire_form::request(&p)), Ok(p));
        let r = decode_request(
            "",
            &names(&["cloudiness", "sun_altitude_angle"]),
            &[1.0, 2.0],
        )
        .unwrap();
        assert_eq!(wire_form::request_of(&wire_form::request(&r)), Ok(r));
        let w = Weather {
            preset: "ClearNoon".into(),
            parameters: [("cloudiness".to_string(), 5.0)].into_iter().collect(),
        };
        let reply = WeatherReply::ok(w, "set");
        assert_eq!(wire_form::reply_of(&wire_form::reply(&reply)), reply);
    }
}
