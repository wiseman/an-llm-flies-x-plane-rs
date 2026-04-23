//! SimBus: thread-safe output bus with status / log / radio channels.
//!
//! Used by the live control loop, the LLM worker, and the TUI to exchange
//! text updates without stepping on each other's stdout. When `echo = true`
//! every push also prints to stdout — set it to false when the TUI is
//! rendering.
//!
//! Log entries carry a `LogKind` so the TUI can style operator input vs
//! LLM output vs verbose worker chatter independently. `snapshot()` and
//! `log_tail()` still return `Vec<String>` (legacy-prefixed) so callers
//! outside the TUI keep working; the TUI uses `log_entries()` to get the
//! typed entries.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;

use chrono::Local;
use parking_lot::Mutex;

pub const LOG_MAX_LINES: usize = 1000;
pub const RADIO_MAX_LINES: usize = 200;

/// Classification of a log entry. Drives TUI styling and the compact-mode
/// filter; also maps to the legacy bracket-tag used in the on-disk `.log`
/// and in `snapshot()` / `log_tail()` return values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LogKind {
    /// Operator input submitted from the TUI (or --atc-message when routed
    /// as operator). Shown prominently.
    Operator,
    /// Final assistant text from the LLM. Shown prominently.
    Llm,
    /// /mode switch confirmations.
    Mode,
    /// Safety-monitor events (go-around, crash detection, …).
    Safety,
    /// Generic system / bridge / track / shutdown messages.
    System,
    /// Error messages. The body may already embed a subsystem tag.
    Error,
    /// LLM tool dispatch — verbose, hidden in compact mode.
    ToolCall,
    /// Per-call token usage — verbose, hidden in compact mode.
    Tokens,
    /// Heartbeat status payloads — verbose, hidden in compact mode.
    Heartbeat,
    /// Voice/PTT state transitions — verbose, hidden in compact mode.
    Voice,
}

impl LogKind {
    /// Legacy bracket-tag (with trailing space) that was textually baked
    /// into the log string before typed entries existed. Re-applied at
    /// read-time by `snapshot()`/`log_tail()`/FileLog so the on-disk `.log`
    /// and string-filter tests stay byte-compatible with the old behavior.
    pub fn legacy_prefix(self) -> Option<&'static str> {
        match self {
            LogKind::Operator => Some("[operator] "),
            LogKind::Llm => Some("[llm] "),
            LogKind::Mode => Some("[mode] "),
            LogKind::Safety => Some("[safety] "),
            LogKind::Heartbeat => Some("[heartbeat] "),
            LogKind::ToolCall | LogKind::Tokens => Some("[llm-worker] "),
            LogKind::System | LogKind::Error | LogKind::Voice => None,
        }
    }

    /// True if this kind should be hidden in the default (compact) TUI view.
    pub fn is_verbose(self) -> bool {
        matches!(
            self,
            LogKind::ToolCall | LogKind::Tokens | LogKind::Heartbeat | LogKind::Voice
        )
    }
}

#[derive(Clone, Debug)]
pub struct LogEntry {
    pub kind: LogKind,
    pub text: String,
}

impl LogEntry {
    pub fn new(kind: LogKind, text: impl Into<String>) -> Self {
        Self { kind, text: text.into() }
    }

    /// Legacy-formatted string: bracket-tag (if any) + body.
    pub fn formatted(&self) -> String {
        match self.kind.legacy_prefix() {
            Some(p) => {
                let mut s = String::with_capacity(p.len() + self.text.len());
                s.push_str(p);
                s.push_str(&self.text);
                s
            }
            None => self.text.clone(),
        }
    }
}

/// Line-buffered append-only file sink with channel tags + timestamps.
pub struct FileLog {
    path: PathBuf,
    file: Mutex<Option<File>>,
}

impl FileLog {
    pub fn new(path: impl AsRef<Path>) -> std::io::Result<Self> {
        let path = path.as_ref().to_path_buf();
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)?;
        }
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        Ok(Self {
            path,
            file: Mutex::new(Some(file)),
        })
    }

    pub fn path(&self) -> &Path {
        &self.path
    }

    pub fn write(&self, channel: &str, text: &str) {
        let timestamp = Local::now().format("%Y-%m-%dT%H:%M:%S").to_string();
        let mut guard = self.file.lock();
        let Some(file) = guard.as_mut() else { return };

        let lines: Vec<&str> = if text.is_empty() {
            vec![""]
        } else {
            text.split('\n').collect()
        };
        for (i, line) in lines.iter().enumerate() {
            let prefix = if i == 0 {
                timestamp.clone()
            } else {
                " ".repeat(timestamp.len())
            };
            let _ = writeln!(file, "{} [{}] {}", prefix, channel, line);
        }
        let _ = file.flush();
    }

    pub fn close(&self) {
        let mut guard = self.file.lock();
        *guard = None;
    }
}

struct BusInner {
    echo: bool,
    file_log: Option<Arc<FileLog>>,
    status_line: String,
    log_buffer: VecDeque<LogEntry>,
    radio_buffer: VecDeque<String>,
}

#[derive(Clone)]
pub struct SimBus {
    inner: Arc<Mutex<BusInner>>,
    /// True while the LLM worker is blocked on a `create_response` HTTP
    /// round-trip. Lock-free so the TUI draw path can poll every frame
    /// without contending with producers pushing log lines.
    llm_busy: Arc<AtomicBool>,
}

impl SimBus {
    pub fn new(echo: bool) -> Self {
        Self {
            inner: Arc::new(Mutex::new(BusInner {
                echo,
                file_log: None,
                status_line: String::new(),
                log_buffer: VecDeque::with_capacity(LOG_MAX_LINES),
                radio_buffer: VecDeque::with_capacity(RADIO_MAX_LINES),
            })),
            llm_busy: Arc::new(AtomicBool::new(false)),
        }
    }

    pub fn set_llm_busy(&self, busy: bool) {
        self.llm_busy.store(busy, Ordering::Relaxed);
    }

    pub fn is_llm_busy(&self) -> bool {
        self.llm_busy.load(Ordering::Relaxed)
    }

    pub fn with_file_log(echo: bool, file_log: Arc<FileLog>) -> Self {
        let me = Self::new(echo);
        me.inner.lock().file_log = Some(file_log);
        me
    }

    pub fn push_status(&self, text: impl Into<String>) {
        let text = text.into();
        let (echo, file_log) = {
            let mut g = self.inner.lock();
            g.status_line = text.clone();
            (g.echo, g.file_log.clone())
        };
        if let Some(f) = file_log {
            f.write("status", &text);
        }
        if echo {
            println!("{}", text);
        }
    }

    /// Shortcut: push a System-kind entry. Kept for unmigrated callers and
    /// for genuinely generic subsystem messages.
    pub fn push_log(&self, text: impl Into<String>) {
        self.push_log_entry(LogEntry::new(LogKind::System, text.into()));
    }

    /// Push a typed log entry. Body should be the bare message — the
    /// `[operator]` / `[llm]` / etc. bracket-tag is re-applied at read-time
    /// via `LogKind::legacy_prefix`.
    pub fn push_log_kind(&self, kind: LogKind, text: impl Into<String>) {
        self.push_log_entry(LogEntry::new(kind, text.into()));
    }

    pub fn push_log_entry(&self, entry: LogEntry) {
        let formatted = entry.formatted();
        let (echo, file_log) = {
            let mut g = self.inner.lock();
            if g.log_buffer.len() == LOG_MAX_LINES {
                g.log_buffer.pop_front();
            }
            g.log_buffer.push_back(entry);
            (g.echo, g.file_log.clone())
        };
        if let Some(f) = file_log {
            f.write("log", &formatted);
        }
        if echo {
            println!("{}", formatted);
        }
    }

    pub fn push_radio(&self, text: impl Into<String>) {
        let text = text.into();
        let (echo, file_log) = {
            let mut g = self.inner.lock();
            if g.radio_buffer.len() == RADIO_MAX_LINES {
                g.radio_buffer.pop_front();
            }
            g.radio_buffer.push_back(text.clone());
            (g.echo, g.file_log.clone())
        };
        if let Some(f) = file_log {
            f.write("radio", &text);
        }
        if echo {
            println!("{}", text);
        }
    }

    /// Snapshot with legacy-formatted log strings. Kept for tests and for
    /// callers that don't need the `LogKind`. The TUI uses `log_entries()`
    /// directly for styling.
    pub fn snapshot(&self) -> (String, Vec<String>, Vec<String>) {
        let g = self.inner.lock();
        (
            g.status_line.clone(),
            g.log_buffer.iter().map(LogEntry::formatted).collect(),
            g.radio_buffer.iter().cloned().collect(),
        )
    }

    /// Typed view of the log buffer for the TUI.
    pub fn log_entries(&self) -> Vec<LogEntry> {
        let g = self.inner.lock();
        g.log_buffer.iter().cloned().collect()
    }

    pub fn log_tail(&self, n: usize) -> Vec<String> {
        let g = self.inner.lock();
        let start = g.log_buffer.len().saturating_sub(n);
        g.log_buffer.iter().skip(start).map(LogEntry::formatted).collect()
    }

    pub fn radio_tail(&self, n: usize) -> Vec<String> {
        let g = self.inner.lock();
        let start = g.radio_buffer.len().saturating_sub(n);
        g.radio_buffer.iter().skip(start).cloned().collect()
    }

    pub fn close(&self) {
        let file_log = { self.inner.lock().file_log.clone() };
        if let Some(f) = file_log {
            f.close();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn push_status_updates_latest_line() {
        let bus = SimBus::new(false);
        bus.push_status("first");
        bus.push_status("second");
        let (s, _, _) = bus.snapshot();
        assert_eq!(s, "second");
    }

    #[test]
    fn push_log_appends() {
        let bus = SimBus::new(false);
        bus.push_log("a");
        bus.push_log("b");
        let (_, logs, _) = bus.snapshot();
        assert_eq!(logs, vec!["a", "b"]);
    }

    #[test]
    fn log_tail_returns_last_n() {
        let bus = SimBus::new(false);
        for i in 0..10 {
            bus.push_log(format!("line{i}"));
        }
        assert_eq!(bus.log_tail(3), vec!["line7", "line8", "line9"]);
    }

    #[test]
    fn log_buffer_is_bounded() {
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
    fn concurrent_pushes_do_not_drop_messages() {
        use std::thread;
        let bus = SimBus::new(false);
        let mut handles = vec![];
        for tag in 0..8 {
            let b = bus.clone();
            handles.push(thread::spawn(move || {
                for i in 0..50 {
                    b.push_log(format!("t{tag}-{i}"));
                }
            }));
        }
        for h in handles {
            h.join().unwrap();
        }
        let (_, logs, _) = bus.snapshot();
        assert_eq!(logs.len(), 8 * 50);
    }

    #[test]
    fn kind_legacy_prefix_matches_old_text_tags() {
        let bus = SimBus::new(false);
        bus.push_log_kind(LogKind::Operator, "takeoff");
        bus.push_log_kind(LogKind::Llm, "rolling");
        bus.push_log_kind(LogKind::Safety, "go_around triggered: …");
        bus.push_log_kind(LogKind::Heartbeat, "{\"phase\":\"cruise\"}");
        bus.push_log_kind(LogKind::ToolCall, "tool set_throttle -> ok");
        bus.push_log_kind(LogKind::Tokens, "tokens in=10 out=5");
        let (_, logs, _) = bus.snapshot();
        assert_eq!(logs[0], "[operator] takeoff");
        assert_eq!(logs[1], "[llm] rolling");
        assert_eq!(logs[2], "[safety] go_around triggered: …");
        assert_eq!(logs[3], "[heartbeat] {\"phase\":\"cruise\"}");
        assert_eq!(logs[4], "[llm-worker] tool set_throttle -> ok");
        assert_eq!(logs[5], "[llm-worker] tokens in=10 out=5");
    }

    #[test]
    fn log_entries_preserves_kind() {
        let bus = SimBus::new(false);
        bus.push_log_kind(LogKind::Operator, "a");
        bus.push_log_kind(LogKind::Llm, "b");
        bus.push_log("c");
        let entries = bus.log_entries();
        assert_eq!(entries[0].kind, LogKind::Operator);
        assert_eq!(entries[0].text, "a");
        assert_eq!(entries[1].kind, LogKind::Llm);
        assert_eq!(entries[1].text, "b");
        assert_eq!(entries[2].kind, LogKind::System);
        assert_eq!(entries[2].text, "c");
    }
}
