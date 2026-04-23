# Repository Guidelines

## Project Structure & Module Organization

`src/main.rs` defines the `sim-pilot` CLI, and `src/lib.rs` exports the library modules. Core orchestration lives in `src/core/`; low-level controllers in `src/control/`; runway, pattern, and taxi logic in `src/guidance/`; offline simulation and the X-Plane bridge in `src/sim/`; LLM/tool wiring in `src/llm/`; push-to-talk transcription in `src/transcribe/`. Integration tests live in `tests/`, with shared fixtures in `tests/common/mod.rs`. YAML defaults are under `config/`. Treat `output/` and `target/` as generated artifacts.

## Build, Test, and Development Commands

- `cargo check --lib`: fast compile check for library changes.
- `cargo run --release --`: run the default deterministic offline scenario.
- `cargo run --release -- --crosswind-kt 10 --log-csv output/flight.csv --plots-dir output/plots`: generate flight logs and plots for tuning.
- `cargo run --release -- --backend xplane`: connect to a live X-Plane 12 session with the interactive TUI; requires the web API on port `8086` and `OPENAI_API_KEY`. Pass `--headless` for a scripted run that reads startup messages from `--atc-message`.
- `cargo test`: run the full test suite.
- `cargo test --test test_scenario`: run the primary mission regression.
- `cargo fmt` and `cargo clippy --all-targets --all-features`: format and lint before opening a PR.

## Coding Style & Naming Conventions

Use standard Rust formatting with `rustfmt` defaults. Keep files and modules in `snake_case`, types and traits in `UpperCamelCase`, and constants in `SCREAMING_SNAKE_CASE`. Keep flight-law changes in `control/` or `guidance/`, and tool-surface changes in `llm/` or `core/`. If you add a Python helper with PEP 723 inline dependencies, run it with `uv run script.py`.

## Testing Guidelines

Add integration coverage in `tests/test_*.rs` and reusable helpers in `tests/common/mod.rs`. Prefer deterministic regressions over manual validation. Changes to mission flow, safety logic, taxi routing, or landing behavior should update a focused test such as `test_mode_transitions.rs`, `test_safety_monitor.rs`, `test_taxi_scenario.rs`, or `test_centerline_rollout.rs`.

## Commit & Pull Request Guidelines

Recent history uses short, imperative, sentence-case subjects. Repository policy requires a three-part commit message: a one-line summary, a short details paragraph, and `Co-authored-by: Codex <codex@openai.com>`. PRs should explain the affected subsystem, list the commands you ran, and note config changes. Include TUI screenshots for interface work and attach plot or CSV evidence when adjusting control behavior.

## Configuration & Runtime Notes

Keep secrets in a local `.env`; do not commit `OPENAI_API_KEY`. Live runs may build cached airport data under `~/.cache/sim_pilot/`; that cache is local state, not source. Use `--apt-dat-path` when testing against a nonstandard X-Plane install.
