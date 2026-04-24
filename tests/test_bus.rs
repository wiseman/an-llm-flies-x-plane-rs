use std::thread;
use tempfile::TempDir;

use xplane_pilot::bus::{FileLog, SimBus, LOG_MAX_LINES};

#[test]
fn push_status_updates_latest_status_line() {
    let bus = SimBus::new(false);
    bus.push_status("first");
    bus.push_status("second");
    assert_eq!(bus.snapshot().0, "second");
}

#[test]
fn push_log_appends_to_log_buffer() {
    let bus = SimBus::new(false);
    bus.push_log("a");
    bus.push_log("b");
    let (_, logs, _) = bus.snapshot();
    assert_eq!(logs, vec!["a", "b"]);
}

#[test]
fn log_tail_returns_last_n_items() {
    let bus = SimBus::new(false);
    for i in 0..10 {
        bus.push_log(format!("line{i}"));
    }
    let tail = bus.log_tail(3);
    assert_eq!(tail, vec!["line7", "line8", "line9"]);
}

#[test]
fn log_buffer_has_upper_bound() {
    let bus = SimBus::new(false);
    for i in 0..1200 {
        bus.push_log(format!("line{i}"));
    }
    let (_, logs, _) = bus.snapshot();
    assert_eq!(logs.len(), LOG_MAX_LINES);
    assert_eq!(logs[0], "line200");
    assert_eq!(logs.last().unwrap(), "line1199");
}

#[test]
fn concurrent_push_log_from_many_threads() {
    let bus = SimBus::new(false);
    let n_threads = 8;
    let n_per_thread = 50;
    let mut handles = vec![];
    for tag in 0..n_threads {
        let b = bus.clone();
        handles.push(thread::spawn(move || {
            for i in 0..n_per_thread {
                b.push_log(format!("t{tag}-{i}"));
            }
        }));
    }
    for h in handles {
        h.join().unwrap();
    }
    let (_, logs, _) = bus.snapshot();
    assert_eq!(logs.len(), n_threads * n_per_thread);
}

#[test]
fn file_log_write_and_read_back() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("run.log");
    let log = FileLog::new(&path).unwrap();
    log.write("log", "bridge connected");
    log.write("radio", "[BROADCAST com1] tower, cessna 123");
    log.close();
    let content = std::fs::read_to_string(&path).unwrap();
    assert!(content.contains("[log] bridge connected"));
    assert!(content.contains("[radio] [BROADCAST com1] tower, cessna 123"));
}

#[test]
fn file_log_timestamp_iso_format() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("run.log");
    let log = FileLog::new(&path).unwrap();
    log.write("status", "hello");
    log.close();
    let content = std::fs::read_to_string(&path).unwrap();
    let re = regex_lite::Regex::new(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2} \[status\] hello").unwrap();
    assert!(re.is_match(&content), "content was {:?}", content);
}

#[test]
fn file_log_multi_line_keeps_continuation_aligned() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("run.log");
    let log = FileLog::new(&path).unwrap();
    log.write("status", "line1\nline2\nline3");
    log.close();
    let content = std::fs::read_to_string(&path).unwrap();
    let lines: Vec<&str> = content.lines().collect();
    assert_eq!(lines.len(), 3);
    for line in &lines {
        assert!(line.contains("[status]"));
    }
    assert!(lines[1].starts_with(' '));
    assert!(lines[2].starts_with(' '));
}

#[test]
fn file_log_parent_directory_created_if_missing() {
    let dir = TempDir::new().unwrap();
    let nested = dir.path().join("a/b/c/run.log");
    let log = FileLog::new(&nested).unwrap();
    log.write("log", "ok");
    log.close();
    assert!(nested.exists());
}

#[test]
fn file_log_close_is_idempotent() {
    let dir = TempDir::new().unwrap();
    let log = FileLog::new(dir.path().join("run.log")).unwrap();
    log.write("log", "one");
    log.close();
    log.close();
}

#[test]
fn file_log_write_after_close_is_noop() {
    let dir = TempDir::new().unwrap();
    let path = dir.path().join("run.log");
    let log = FileLog::new(&path).unwrap();
    log.write("log", "before close");
    log.close();
    log.write("log", "after close");
    let content = std::fs::read_to_string(&path).unwrap();
    assert!(content.contains("before close"));
    assert!(!content.contains("after close"));
}

mod regex_lite {
    // A tiny wrapper so we don't pull the full `regex` crate. Matches a single
    // static pattern used above. Keeps dev-deps lean.
    pub struct Regex {
        pattern: String,
    }
    impl Regex {
        pub fn new(pattern: &str) -> anyhow::Result<Self> {
            Ok(Self { pattern: pattern.to_string() })
        }
        pub fn is_match(&self, text: &str) -> bool {
            // We only care about: r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2} \[status\] hello"
            // Hand-written matcher for that exact shape.
            if !self.pattern.contains("\\d{4}") {
                return text.contains(&self.pattern);
            }
            // Scan for "YYYY-MM-DDTHH:MM:SS [status] hello" anywhere.
            if let Some(idx) = text.find(" [status] hello") {
                let ts = &text[..idx];
                if let Some(start) = ts.rfind(['\n', '\r']) {
                    let ts = &ts[start + 1..];
                    return looks_like_iso(ts);
                }
                return looks_like_iso(ts);
            }
            false
        }
    }
    fn looks_like_iso(s: &str) -> bool {
        if s.len() != 19 { return false; }
        let bytes = s.as_bytes();
        bytes[4] == b'-'
            && bytes[7] == b'-'
            && bytes[10] == b'T'
            && bytes[13] == b':'
            && bytes[16] == b':'
            && s[0..4].chars().all(|c| c.is_ascii_digit())
            && s[5..7].chars().all(|c| c.is_ascii_digit())
            && s[8..10].chars().all(|c| c.is_ascii_digit())
            && s[11..13].chars().all(|c| c.is_ascii_digit())
            && s[14..16].chars().all(|c| c.is_ascii_digit())
            && s[17..19].chars().all(|c| c.is_ascii_digit())
    }
}
