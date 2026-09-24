# Benchmarks

- `gen_graphs.py` — generates the synthetic layout stress graphs into `graphs/`.
- `run.sh <binary>` — times `dotv --dump` over every graph (best of 3) and
  dumps per-phase `GD_TIMING` traces into `results/`.
- `baseline/` (gitignored) — `dotv --dump` references used as a byte-level
  regression oracle; the engine is deterministic, so any trusted build can
  regenerate them:
  `for f in tests/corpus/*.dot benches/graphs/*.dot; do ./dotv --dump $f > benches/baseline/$(basename $f .dot).json; done`
- A change must keep every dump byte-identical unless it deliberately alters
  layout semantics.
