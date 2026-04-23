use xplane_pilot::tui::parse_input_source;

#[test]
fn plain_text_defaults_to_operator() {
    assert_eq!(parse_input_source("take off"), ("operator".into(), "take off".into()));
}

#[test]
fn atc_colon_prefix() {
    assert_eq!(
        parse_input_source("atc: N1234, what runway are you on?"),
        ("atc".into(), "N1234, what runway are you on?".into())
    );
}

#[test]
fn atc_bracket_prefix() {
    assert_eq!(
        parse_input_source("[ATC] cleared for takeoff runway 16L"),
        ("atc".into(), "cleared for takeoff runway 16L".into())
    );
}

#[test]
fn operator_colon_prefix() {
    assert_eq!(parse_input_source("operator: standby"), ("operator".into(), "standby".into()));
}

#[test]
fn operator_bracket_prefix() {
    assert_eq!(parse_input_source("[operator] ready"), ("operator".into(), "ready".into()));
}

#[test]
fn case_insensitive_prefix_detection() {
    assert_eq!(parse_input_source("AtC: hello"), ("atc".into(), "hello".into()));
    assert_eq!(parse_input_source("[Operator] hi"), ("operator".into(), "hi".into()));
}

#[test]
fn whitespace_stripped_after_prefix() {
    assert_eq!(parse_input_source("atc:    spaced out"), ("atc".into(), "spaced out".into()));
}
