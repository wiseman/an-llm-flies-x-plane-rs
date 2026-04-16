//! SimBus: thread-safe output bus with status / log / radio channels.
//!
//! Mirrors sim_pilot/bus.py. Used by the live control loop, the LLM worker,
//! and the TUI to exchange text updates without stepping on each other's
//! stdout. When `echo = true` every push also prints to stdout — set it to
//! false when the TUI is rendering.

use std::collections::VecDeque;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;

use chrono::Local;
use parking_lot::Mutex;

pub const LOG_MAX_LINES: usize = 1000;
pub const RADIO_MAX_LINES: usize = 200;

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
    log_buffer: VecDeque<String>,
    radio_buffer: VecDeque<String>,
}

#[derive(Clone)]
pub struct SimBus {
    inner: Arc<Mutex<BusInner>>,
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
        }
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

    pub fn push_log(&self, text: impl Into<String>) {
        self.push_channel("log", text.into(), /* radio */ false);
    }

    pub fn push_radio(&self, text: impl Into<String>) {
        self.push_channel("radio", text.into(), /* radio */ true);
    }

    fn push_channel(&self, channel: &'static str, text: String, radio: bool) {
        let (echo, file_log) = {
            let mut g = self.inner.lock();
            let buf = if radio { &mut g.radio_buffer } else { &mut g.log_buffer };
            let cap = if radio { RADIO_MAX_LINES } else { LOG_MAX_LINES };
            if buf.len() == cap {
                buf.pop_front();
            }
            buf.push_back(text.clone());
            (g.echo, g.file_log.clone())
        };
        if let Some(f) = file_log {
            f.write(channel, &text);
        }
        if echo {
            println!("{}", text);
        }
    }

    pub fn snapshot(&self) -> (String, Vec<String>, Vec<String>) {
        let g = self.inner.lock();
        (
            g.status_line.clone(),
            g.log_buffer.iter().cloned().collect(),
            g.radio_buffer.iter().cloned().collect(),
        )
    }

    pub fn log_tail(&self, n: usize) -> Vec<String> {
        let g = self.inner.lock();
        let start = g.log_buffer.len().saturating_sub(n);
        g.log_buffer.iter().skip(start).cloned().collect()
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
}
