# AB-IRRT\* — experiment code

Reference implementation of the planners compared in the manuscript

> **AB-IRRT\*: Two-Stage Path Planning with a Bug-APF Pre-Planner as a Warm Start**

AB-IRRT\* couples a reactive **Bug-APF pre-planner** with an **enhanced Informed-RRT\***:
the pre-planner returns a cheap, feasible polyline, a gated greedy shortcutting step
tightens it, and the resulting cost bound seeds both the informed sampling ellipsoid and a
path-point sampling bias. The reactive stage is written as an explicit three-state machine
(direct / avoidance / escape) with bounded forces, exact segment-level collision checking,
cycle detection and explicit termination conditions.

---

## Quick start

```bash
git clone https://github.com/jjl1-good/B-APF-IRRT.git
cd B-APF-IRRT/experiments
```

In MATLAB (R2024b or later, base MATLAB is enough, no toolboxes required):

```matlab
smoke_test          % ~10 s: one run of all eight algorithms in the 2-D general map
```

`smoke_test` prints one line per algorithm (success, path length, runtime) and is the
quickest way to check that the clone works. It does not write any file.

Then, to reproduce the paper:

```matlab
run_all_comparisons     % 2-D and 3-D main experiments (long: ~1-2 h on a laptop)
summarize_results2d ; summarize_results3d
make_revision_figures   % figures 1-6 of the manuscript
regen_all_figures       % everything, including the supplementary figures
```

Every script adds the two algorithm folders itself; no path configuration is needed and no
absolute path is used anywhere in the repository.

> **This repository contains code only.** The per-run result tables (`*.csv`), the anytime
> convergence histories (`*.mat`) and the figures are *not* committed: they are written by the
> scripts above and are ignored by `.gitignore`. Clone, run, and every number in the paper is
> re-derived; the published figures and tables are distributed as supplementary material with
> the article.

---

## Repository layout

| folder | contents |
|---|---|
| `algorithms_2d/` | 2-D planners and environments: `rrtstar2d`, `informed_rrtstar2d`, `prmstar_nd`, `fmtstar_nd`, `bitstar_nd`, `rrtsharp_nd`, `stable_apf2d` (APF-only), `bair_core2d` (AB-IRRT\*), `bair_env2d` (benchmark maps), `collisionChecking`, and the two published planners reimplemented for the comparison: `has_rrt2d` (skeleton-guided warm start, with the helpers `skeleton_build2d`, `skel_grid2d`, `skel_ridge2d`) and `apf_irrtstar2d` (APF-IRRT\*, discussed in the related work only) |
| `algorithms_3d/` | 3-D counterparts of the same planner set (`apf_irrtstar3d`, `has_rrt3d`, `skeleton_build3d`, `skel_grid3d`, `skel_ridge3d` included), plus the 3-D primitives (boxes, cylinders, spheres) and the five benchmark scenes |
| `experiments/` | the run, summary and figure scripts listed below. The result files they read and write (CSV tables and `anytime_*.mat` histories) are produced locally and are not tracked by git |

> **Scene names and seeds.** The five 3-D scenes are addressed as `general`, `narrow`, `suspended`,
> `ring` and `overhang`; `general_v2`, `wide`, `g` and `g2` are accepted as aliases of `general` and
> all of them resolve to the same scene.
> Every run script is invoked per scene with `opts.scenes`, so the seeds
> (`20260903 + 1000*c + r`, with `c` the position of the scene in the list) are identical
> across the main comparison, the warm-start group and the hybrid group.

### Scripts in `experiments/`

| script | what it does |
|---|---|
| `smoke_test.m` | one run of all eight algorithms in the 2-D general map (installation check) |
| `run_2d_comparison.m` | 8 algorithms × 3 maps × N paired repetitions (default 50) |
| `run_3d_comparison.m` | 8 algorithms × 5 scenes × N paired repetitions |
| `run_ablation2d.m`, `run_ablation3d.m` | seven pipeline variants, one mechanism removed at a time |
| `run_equal_time.m` | same wall-clock cap for every algorithm (budget raised to 10⁵) |
| `run_budget_sweep_2d.m` | 2-D sweep over 500 / 1000 / 2000 extensions |
| `run_verify_safety.m`, `verify_safety.m` | segment-by-segment re-validation of returned paths |
| `run_all_comparisons.m` | driver that runs the 2-D, 3-D and ablation experiments in order |
| `run_warmstart_comparison.m`, `warm_run2d.m`, `warm_run3d.m`, `warm_smoke.m` | warm-start equivalent baseline (the same Bug-APF seed used only as `c_best`) → `warmstart_2d_main.csv`, `warmstart_3d_main.csv` |
| `hybrid_run2d.m`, `hybrid_run3d.m`, `hybrid_run_all.m` | the dedicated published-planner batch behind Table S18 (APF-IRRT\* and HAS-RRT) → `hybrid_2d_main.csv`, `hybrid_3d_main.csv`. APF-IRRT\* appears in the paper only in the related-work discussion; HAS-RRT is reported with the warm-start methods. |
| `hybrid_smoke.m`, `hybrid_smoke2d.m`, `hybrid_smoke3d.m` | smoke tests for the two hybrid baselines |
| `summarize_results2d.m`, `summarize_results3d.m` | per-algorithm summaries (mean ± std, success rate) |
| `analyze_convergence.m` | convergence tables (time and budget to reach the ±2 % / ±5 % bands) |
| `analyze_first_solution.m` | time-to-first-solution tables from the anytime histories |
| `merge_anytime.m` | merges the per-scenario anytime `.mat` histories |
| `make_revision_figures.m`, `make_extra_figures.m`, `make_algorithm_figures.m` | manuscript figures |
| `make_routes_overview.m`, `make_scene_figure.m`, `make_method_figure.m`, `make_pareto_figure.m`, `make_convergence_figure.m` | the individual figures (path overlays, scenes, method schemes, trade-off, convergence) |
| `regen_all_figures.m`, `regen_summaries.m` | regenerate all figures / all summary tables |
| `export_firstsol_2d.m`, `export_firstsol_3d.m` | export the first-solution tables behind Figures S9 and S10 |
| `export_figure.m`, `style_figures_for_review.m` | shared figure export and typography helpers used by every figure script |
| `setup_bair_paths.m` | adds the algorithm folders to the MATLAB path |
| `log_progress.m` | progress logging helper used by the run scripts |

---

## Outputs produced by the scripts

Running the scripts writes their results next to them in `experiments/`, as plain CSV tables
(one row per algorithm × repetition, with success, length, total time, time to first solution,
the planner/shortcut/pre-planning/optimisation split, nodes, iterations, turn sum, minimum
clearance, straightness, escape entries, direction flips and the fallback flag), per-environment
summaries (success rate, mean/median length, standard deviation, mean time), the budget sweep,
the ablation, equal-time, convergence and first-solution tables, and the `anytime_*.mat`
convergence histories.

**None of these files are tracked by git** — `.gitignore` excludes `*.csv`, `*.mat` and
`figures/`, so a clone contains the code and nothing else. The tables and figures that appear in
the article and its supplementary material are distributed with the paper, and can also be
regenerated from scratch by the scripts above.

The scenes, start/goal pairs and obstacle sets are defined in code — `bair_env2d.m`
(modes `y`, `n`, `g`) and `bair_env3d.m` (modes `general`, `narrow`, `suspended`,
`ring`, `overhang`) — so no external data file is required to run the experiments.

## Experiment protocol (as reported in the paper)

- All eight algorithms share one code base, one collision checker, one environment
  description and one timing instrumentation.
- Every environment-repetition pair starts from the same random seed for every algorithm,
  which is what makes the runs paired.
- Budgets: 2000 tree extensions in 2-D, 6000 in 3-D, identical for all algorithms within an
  environment.
- 50 repetitions per algorithm and environment in the main experiments; 30 in the
  equal-time comparison.
- Every algorithm reports the raw path its own planning returns; no final-path shortcutting
  or smoothing is applied afterwards.


## Same-family warm-start planners (Section 3.5, Tables 4, S20 and S21)

Section 3.5 of the manuscript compares the pipeline with the planners that build a structure or a
first path before the optimisation starts. The implementations added for that comparison are:

| File | What it is |
|---|---|
| `algorithms_2d/ge_irrtstar2d.m`, `algorithms_3d/ge_irrtstar3d.m` | Reconstruction of Ge et al. (Electronics Optics & Control 2025, 32(1):48-53): APF-based sampling-point screening, dynamic step size, bidirectional greedy connection, A*-style parent selection, grandparent shortcut and B-spline post-processing. The 3-D file is our own promotion of the published 2-D method (obstacle masses become volumes, the Manhattan term becomes the 3-D L1 distance); it is listed in Table S21 as an extension. |
| `algorithms_2d/has_rrt2d.m`, `algorithms_3d/has_rrt3d.m` | The published HAS-RRT reconstruction, now with two extra switches that are **off by default**: `star` (best parent + rewiring, i.e. the "HAS-RRT*" variant of Table 4) and `useEllipse` (ellipsoidal sampling after the first solution; a further variant that is not part of the paper). Neither starred variant is defined in the original paper; they were built for this comparison. With both switches off the files reproduce the results reported in Tables 1-2 and S18. |
| `experiments/run_ge_2d.m`, `run_ge_3d.m`, `run_has_star_2d.m`, `run_has_star_3d.m` | Stand-alone runs of the same-family planners with the seeds of the main campaign. |
| `experiments/run_family_2d.m`, `run_family_3d.m` | The single-batch runs behind Table 4 (2-D) and Table S21 (3-D): AB-IRRT*, Ge et al. 2025, HAS-RRT and HAS-RRT* in one MATLAB session, so that the runtimes are comparable with each other. Their per-run tables and anytime histories are written locally and are not tracked. |
| `run_2d_comparison.m`, `run_3d_comparison.m` | Extended with the algorithm keys added for that comparison (`Ge-IRRT*`, `Ge-IRRT*-uniform`, `HAS-RRT*`); the result tables additionally carry `lenRaw`, `lenSmooth`, `smoothOK` and `firstSolIter`. |

Reproduce:

```matlab
run_family_2d(50);   % Table 4  (same batch, 2-D)
run_family_3d(50);   % Table S21 (same batch, 3-D, five scenes)
```

Caveats that also appear in the paper: both published methods are reimplemented here from their published
descriptions, so the comparison relies on our reconstruction rather than on the authors' code (a sensitivity
run with the point-selection stage of Ge et al. disabled is part of the code and can be reproduced); the
B-spline stage of that method cannot return a collision-free curve on these maps (the corners it cuts are of
the same order as the path clearance), so the reported length is the de-redundancy polyline it falls back to;
and the two starred HAS-RRT variants are ours, not the authors'.

## Requirements and runtime

- MATLAB R2024b (R2021a or later should work; the code uses only base MATLAB).
- No toolbox, no GPU, no external data.
- `smoke_test`: seconds. Full 2-D + 3-D main experiment: about 1–2 h on a
  laptop-class machine (single thread). Figure regeneration: a few minutes.

## Citation and contact

If you use this code, please cite the manuscript above (the final reference will be added
once it is published).

- Repository: <https://github.com/jjl1-good/B-APF-IRRT>
- Corresponding author: Bing Fu (13476039901@139.com)
- The repository ships the code only. Running the run scripts followed by the summary and
  figure scripts re-derives every number and every figure in the paper; the per-run data and
  the published figures are distributed with the article as supplementary material.

## License

Released for academic use; please contact the corresponding author before any commercial
use.
