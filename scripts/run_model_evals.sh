#!/usr/bin/env bash
# Driver for the multi-model eval matrix. Runs 3 trials per (provider,model)
# pair and dumps each summary.json into output/eval-batch-20260426/.
set -u
mkdir -p output/eval-batch-20260426

run_one() {
  local provider="$1"
  local model="$2"
  local label="$3"
  local trial="$4"
  local stem
  stem="${label}-trial${trial}"
  local outdir="output/eval-batch-20260426/${stem}"
  mkdir -p "${outdir}"
  echo "[start] ${stem}  (${provider} / ${model})"
  ./target/release/eval \
    --airport KWHP \
    --parking "FBO Parking" \
    --pilot-llm-provider "${provider}" \
    --pilot-llm-model "${model}" \
    --initial-prompt "Take off, do a lap in the pattern, land, then park." \
    --output-dir "${outdir}" \
    > "${outdir}/${stem}.stdout.log" 2>&1
  local rc=$?
  echo "[done ] ${stem}  rc=${rc}"
}

run_model_trials() {
  local provider="$1"
  local model="$2"
  local label="$3"
  for trial in 1 2 3; do
    run_one "${provider}" "${model}" "${label}" "${trial}"
  done
}

# Per provider, run trials sequentially to avoid rate-limit collisions.
# Across providers, run in parallel.
run_model_trials anthropic claude-opus-4-6                opus46  &
ANTHRO_PID=$!
run_model_trials anthropic claude-haiku-4-5               haiku45 &
ANTHRO2_PID=$!

run_model_trials openai    gpt-5.4-2026-03-05             gpt54   &
OAI_PID=$!
run_model_trials openai    gpt-5.4-mini-2026-03-17        gpt54mini &
OAI2_PID=$!

run_model_trials gemini    gemini-3.1-pro-preview         gem31pro     &
GEM_PID=$!
run_model_trials gemini    gemini-3.1-flash-lite-preview  gem31flite   &
GEM2_PID=$!

wait "${ANTHRO_PID}"  "${ANTHRO2_PID}" \
     "${OAI_PID}"     "${OAI2_PID}" \
     "${GEM_PID}"     "${GEM2_PID}"

echo "[all done]"
