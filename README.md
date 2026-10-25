# UAV Genetic Algorithm Strategy Engine

**Bahria University, Lahore Campus · Artificial Intelligence · BSCS Semester 5 · Assignment 2**  
**Team (3 members):** __________________ / __________________ / __________________  
**Roll numbers:** __________________ / __________________ / __________________

A complete, reproducible **object-oriented Python implementation** of a multi-objective Genetic Algorithm (GA) that plans a UAV fleet's **deployment positions, waypoint trajectories, speeds, and sensor activation/power** for disaster surveillance on a **50 × 50 hazard grid**. Source code, documented mathematical model, generated plots, sample results, automated tests, experiments, and a 3–5 page technical report are included.

## Upload this project to GitHub (important)

1. **Extract `UAV_GA_Assignment_GitHub.zip`** to a folder on your computer.
2. On github.com, create a **new repository** (for example, `UAV-GA-Assignment-2`).
3. Open your repository → **Add file → Upload files**. From the *extracted folder*, select/drag **all files and folders inside it**, including `uav_ga`, `outputs`, `config`, `tests`, `.github`, `README.md`, `REPORT.docx`, `requirements.txt`, and `UAV_GA_Colab.ipynb`.
4. Click **Commit changes**. Check that `README.md` appears on the repo home page.

**Do not upload only the `.zip`** if you want GitHub to display individual Python files and charts. The extracted folder contents are the repository root.

## Folder structure

```text
UAV_GA_Assignment/
├── outputs/
│   ├── simulation/                    # Executed GA run, baseline, PNG figures, CSV, JSON
│   ├── sensitivity/                   # Actual 9-run parameter sweep + chart
│   └── smoke/                         # Fast run for reproducibility
├── uav_ga/
│   ├── config.py                      # Settings, validation
│   ├── environment.py                 # Priority/hazard/no-fly grid, A* routing
│   ├── chromosome.py                  # Multi-segment chromosome + repair
│   ├── fitness.py                     # Five-objective fitness + penalties
│   ├── operators.py                   # Geometric crossover + spatial mutation
│   ├── ga.py                          # Population, tournament, elitism, termination
│   ├── baseline.py                    # Equal-evaluation random-search control
│   ├── experiments.py                 # Hyperparameter sensitivity sweeps
│   ├── visualization.py               # Paths, heatmaps, convergence, evolution
│   ├── cli.py                         # Command-line runner
│   └── __main__.py
├── config/mission.json                # Default mission and GA parameters
├── tests/test_engine.py               # Automated unit + integration checks
├── .github/workflows/ci.yml           # GitHub Actions test workflow
├── UAV_GA_Colab.ipynb                 # Jupyter / Google Colab companion
├── REPORT.docx                        # 3–5 page submitted technical report
├── requirements.txt
├── pyproject.toml
└── README.md
```

## Requirements and running locally

**Python 3.10+** recommended. Open a terminal at the extracted **project root**:

```bash
python -m pip install -r requirements.txt
python -m uav_ga simulate
```

The default command reads `config/mission.json` and writes new figures/results to `outputs/simulation/`. You can provide custom output, seed and configuration:

```bash
python -m uav_ga simulate --config config/mission.json --seed 42 --out outputs/my_run
python -m uav_ga simulate --quick --out outputs/quick_demo
python -m uav_ga sensitivity --out outputs/my_sensitivity --quick
python -m uav_ga sensitivity --out outputs/my_sensitivity_full
python -m pytest -q
```

`--quick` means 5 GA generations for a simulation, or 12 generations for each sensitivity variant. The generated results already inside `outputs/` are real executions, not illustrative dummy data. Charts can be opened directly as PNG files.

### Colab / Jupyter notebook

Run `UAV_GA_Colab.ipynb` with Jupyter while in the extracted folder. On Google Colab, first push the project to GitHub, open the notebook, and in its first code cell assign `GITHUB_REPO_URL = 'https://github.com/YOUR_USERNAME/YOUR_REPO.git'`. **Run all**: it clones your repo and imports the modular source code. This avoids copying all the algorithm code into the notebook.

## Chromosome design

Every individual is one strategy for **D** UAVs, each with **K** waypoint visits:

- **Position genes:** float `(x, y)` coordinates with shape `[D, K, 2]`; the first waypoint is each UAV's flexible deployment position.
- **Speed genes:** float flight speeds in `[1.0, 3.0]`, shape `[D, K]`.
- **Sensor genes:** normalized sensor power/schedule in `[0,1]`, shape `[D, K]`; near zero = off.

Fixed safe UAV launch bases are outside the genotype; UAVs travel from base → deployment/waypoints → base. For each leg the corresponding speed and sensor gene applies; the return leg uses the final control gene. Waypoints are snapped to legal grid cells. Direct segments are followed only when free of obstacles and forbidden diagonal corner cuts; otherwise **8-connected A\*** finds a collision-free route. A repair step pulls excessively long waypoints inward; a residual large range/time/battery violation attracts penalties.

## Fitness formula (maximize)

`F = 0.40*C + 0.25*D - 0.15*E - 0.10*T - 0.10*R - P`

| Symbol | What it measures | Normalization |
|:---|:---|:---|
| `C` | High-priority coverage: weighted sum of detected cell probabilities | Sum over grid priority / total priority |
| `D` | Detection: mean detection probability at simulated target cells | 0–1 |
| `E` | Energy: speed-dependent propulsion + sensor activity | Total fleet energy / (UAVs × battery budget) |
| `T` | Mission makespan: longest drone flight duration | Longest duration / per-UAV time limit |
| `R` | Hazard exposure: integrated flight-time-weighted hazard intensity | Total fleet exposure / (UAVs × time limit) |
| `P` | Constraint violations | Sum of excess ratios for per-UAV range, battery and time, weighted `1.5`, `1.3`, `1.3` |

A cell's repeated sensor observations accumulate a saturating detection probability. Negative fitness is valid when costs or penalties dominate. No-fly cells are handled as **hard geometric obstacles**; traversals are guaranteed legal by safe routing, while resource violations are handled by repair and penalties.

## Evolution workflow and termination

1. Build seeded reproducible disaster map, obstacles, 9 target cells, and three launch bases.
2. Randomly initialize each strategy, repair position, speed, sensor genes.
3. Score priority coverage, detection, fleet energy, flight time, hazards, resource violations.
4. Apply **tournament selection** (`tournament_size=3`), **elitism** (top 2), **tail segment swapping plus geometric coordinate blending**, independent **Gaussian spatial jitter**, and occasional waypoint swaps. Repair offspring.
5. Maintain population diversity with optional fresh random immigrants, adaptive mutation rate during stagnation, and log best/mean fitness and genetic diversity.
6. Stop after maximum generations (38 by default) or 15 consecutive nonimproving generations.
7. Compare GA to a **random-search baseline using the same number of evaluations** on the same mission; plot results.

## Visualizations and evidence

| Output | Description |
|:---|:---|
| `outputs/simulation/fitness_convergence.png` | Best and average fitness over all processed generations |
| `outputs/simulation/uav_paths.png` | Final UAV return-to-base paths, targets, hazards, blocked areas |
| `outputs/simulation/coverage_heatmap.png` | Sensor coverage probability and high-priority zone weights |
| `outputs/simulation/evolution_over_generations.png` | Coverage+flight paths at several optimization snapshots |
| `outputs/sensitivity/sensitivity.png` | Population, mutation and tournament pressure sweeps |
| `outputs/simulation/fitness_history.csv` | Per-generation fitness/diversity data |
| `outputs/simulation/summary.json` | Every metric, fitted chromosome, constraints, run seed |
| `outputs/simulation/baseline_comparison.json` | Random baseline metrics/equal-budget comparison |
| `outputs/sensitivity/sensitivity.json` | Individual sensitivity trials/evaluation counts |

The technical report describes **observed** trial results as well as the limitations of a synthetic one-seed experiment. A different run seed or Python/dependency version may lead to different paths/fitness; that is expected for stochastic optimization.

## Notes

- **Academic simulation only:** coordinates and sensor values do not command actual drones; there is no real-time wind, terrain elevation, propulsion calibration, or flight-control hardware.
- **Fairness:** baseline uses equal candidate evaluation budget; the sensitivity table lists evaluation counts, which vary with population size. Sensitivity comparisons thus describe empirical trial outcomes, not a controlled equal-budget causal test.
- **Optimization method:** a transparent scalarized multi-objective GA, not a Pareto-front method like NSGA-II. Weights express subjective disaster-monitoring trade-offs.
- **Team submission:** replace the blank team member fields in this README and `REPORT.docx` before submitting.
