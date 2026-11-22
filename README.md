[![make.fun](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/wasu-tenan/sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/wasu-tenan/sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

# Sokoban solver

A push-optimal Sokoban solver for make.fun's [Build the best Sokoban solver, 21 November 2026](https://make.fun/bounty/sokoban-solver-01-run-20261121) bounty ($100 in USDC).

## How it works

Every generated level is one hallway row with 1–4 rooms off it, and a box
pushed out of its room through a door can never come back — so each room is
an independent sub-problem. The solver:

1. splits the level into rooms by dropping the hallway row and taking the
   connected floor components, each with its own door and entry cell;
2. solves every room push-optimally with A* over push states — sorted box
   positions, the player canonicalised to its reachable region, a
   minimum-assignment (Hungarian) box-to-goal distance heuristic, dead-cell
   pruning (corners and goal-less wall runs) and 2×2 freeze detection — with
   a greedy best-first fallback if A* exhausts its work budget;
3. stitches the room solutions together with hallway walks between doors,
   and replays the full move string to verify every box ends on a goal.

The search budget is counted in expanded states, never in wall-clock time,
so two runs on the same input write identical bytes. It uses only Node
built-ins, no dependencies, no network, no clock reads.

## Results (practice set)

| Instance | Boxes | Pushes | Push-optimal (P*) | Points |
|----------|-------|--------|-------------------|--------|
| i001 | 3 | 13 | 13 | 1,000 |
| i002 | 4 | 16 | 16 | 1,000 |
| i003 | 8 | 29 | 29 | 1,000 |
| i004 | 12 | 47 | 47 | 1,000 |
| i005 | 16 | 60 | 60 | 1,000 |

Practice score: **1,000,000 / 1,000,000** in ~0.9 s for the whole set. The
house baseline (plain-astar 1.0.0) scores 200,000 on the same set — it solves
only i001. Three further freshly drawn sets (15 unseen levels) were also all
solved at exactly P*.

## Run

```sh
node main.ts
```

The program reads `/input/<id>/level.txt` and writes
`/output/<id>/solution.txt` for every instance. Locally:

```sh
mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

## License

MIT — see [LICENSE](LICENSE).
