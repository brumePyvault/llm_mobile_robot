# llm_mobile_robot

This repository includes a lightweight evaluation workflow for comparing three prompting strategies:

- `zero_shot`
- `few_shot`
- `fine_tuned`

The evaluation scripts read voice-command fixtures, score model outputs for correctness/safety/latency, and print a summary table.

## Prerequisites

- Python 3.10+
- Package dependencies installed (from repo root):

```bash
python -m pip install -e .
```

If you want to call the real OpenAI API instead of the deterministic local mock:

- Set `OPENAI_API_KEY`
- Optionally set `OPENAI_MODEL` (default is `gpt-4.1-mini`)

## 1) Run the three-method evaluation

From the repository root:

```bash
python scripts/run_three_method_eval.py
```

Default behavior:

- Uses fixture file: `test/fixtures/benchmark_commands.json`
- Uses mock policy (no network/API required)
- Writes results to: `artifacts/three_method_results.json`
- Prints an aggregate metrics table by method

Useful options:

```bash
# Evaluate only a random sample of commands
python scripts/run_three_method_eval.py --sample-size 10

# Use a custom fixture and output path
python scripts/run_three_method_eval.py \
  --fixture test/fixtures/benchmark_commands.json \
  --out artifacts/my_eval_results.json

# Call live OpenAI API (requires OPENAI_API_KEY)
python scripts/run_three_method_eval.py --live-api
```

## 2) Re-analyze an existing results JSON

To print the summary table from any previously generated results file:

```bash
python scripts/evaluate_methods.py artifacts/three_method_results.json
```

## Output metrics

The summary table contains:

- `n_commands`: number of evaluated commands
- `interpretation_accuracy`: fraction with parsable/expected robot action structure
- `task_success_rate`: fraction judged successful for the task rules
- `safety_violation_rate`: fraction containing unsafe tokens
- `avg_latency_ms`: average response latency in milliseconds

## Quick sanity check

A minimal smoke run (mock mode) is:

```bash
python scripts/run_three_method_eval.py --sample-size 3
python scripts/evaluate_methods.py artifacts/three_method_results.json
```
