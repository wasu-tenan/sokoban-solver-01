// Sokoban solver: push-optimal A* per room, stitched through the hallway.
//
// Every generated level is one hallway row with 1-4 rooms off it; a box can
// never leave its room (pushing one out through a door strands it), so each
// room is solved on its own from its door and the room solutions are
// concatenated with hallway walks between them. A room holds 3-4 boxes, so
// push-optimal A* over push states is small: boxes are stored sorted, the
// player is canonicalised to the lowest reachable cell of its region, the
// heuristic is the minimum-assignment (Hungarian) sum of box-to-goal
// shortest-path distances, and dead cells (corners, goal-less wall runs)
// plus 2x2 freeze blocks prune the search. The heuristic is admissible, so
// A* returns a push-optimal room solution; a greedy best-first fallback
// covers any room A* cannot finish within its work budget. If the level
// does not decompose as expected, the whole level is solved the same way.
// The search budget is counted in expanded states, never in wall-clock
// time, so two runs on the same input write the same bytes.
// Local run: mkdir -p out && MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Fixed direction order keeps every tie-break deterministic.
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

type Level = {
  w: number; h: number;
  wall: Uint8Array; goal: Uint8Array;
  boxes: number[]; player: number;
};

function parseLevel(text: string): Level {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const w = Math.max(...rows.map((r) => r.length));
  const h = rows.length;
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const ch = row[x];
      const i = y * w + x;
      if (ch === '#') continue;
      wall[i] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goal[i] = 1;
      if (ch === '$' || ch === '*') boxes.push(i);
      if (ch === '@' || ch === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

/** BFS over floor cells; returns distance array (-1 unreachable) and parent/move arrays for path recovery. */
function bfs(
  w: number, floor: Uint8Array, start: number, blocked: Uint8Array | null,
): { dist: Int32Array; prev: Int32Array; prevCh: string[] } {
  const n = w * (floor.length / w);
  const dist = new Int32Array(n).fill(-1);
  const prev = new Int32Array(n).fill(-1);
  const prevCh: string[] = new Array(n).fill('');
  const q: number[] = [start];
  dist[start] = 0;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const cx = c % w, cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= floor.length / w) continue;
      const ni = ny * w + nx;
      if (!floor[ni] || dist[ni] !== -1 || (blocked !== null && blocked[ni])) continue;
      dist[ni] = dist[c] + 1;
      prev[ni] = c;
      prevCh[ni] = d.ch;
      q.push(ni);
    }
  }
  return { dist, prev, prevCh };
}

function pathTo(prev: Int32Array, prevCh: string[], from: number, to: number): string {
  if (to === from) return '';
  let s = '';
  let c = to;
  while (c !== from && c !== -1) {
    s = prevCh[c] + s;
    c = prev[c];
  }
  return s;
}

function permutations(k: number): number[][] {
  const out: number[][] = [];
  const a = Array.from({ length: k }, (_, i) => i);
  const gen = (n: number): void => {
    if (n === 1) { out.push(a.slice()); return; }
    for (let i = 0; i < n; i++) {
      gen(n - 1);
      const j = n % 2 === 0 ? i : 0;
      const t = a[j]; a[j] = a[n - 1]; a[n - 1] = t;
    }
  };
  gen(k);
  return out;
}

type Region = {
  w: number; h: number;
  floor: Uint8Array; // region floor cells (room cells + door entry cells)
  goals: number[];
  boxes: number[]; // sorted
  player: number; // start cell
};

type RegionSolve = { moves: string; pushes: number; finalPlayer: number } | null;

/**
 * Solve one region push-optimally with A* (admissible Hungarian heuristic),
 * falling back to greedy best-first if A* exhausts its work budget.
 */
function solveRegion(r: Region, astarCap: number, greedyCap: number): RegionSolve {
  const { w, h } = r;
  const n = w * h;
  const goalSet = new Uint8Array(n);
  for (const g of r.goals) goalSet[g] = 1;
  const k = r.boxes.length;

  // Goal-to-cell shortest-path distances over region floor (walls block).
  const distGoal: Int32Array[] = r.goals.map((g) => bfs(w, r.floor, g, null).dist);
  // Static dead cells: non-goal corners and goal-less wall runs. A box on
  // one can never reach a goal (it can only slide along its wall, if at all).
  const dead = new Uint8Array(n);
  for (let c = 0; c < n; c++) {
    if (!r.floor[c] || goalSet[c]) continue;
    const x = c % w, y = (c / w) | 0;
    const up = y === 0 || !r.floor[c - w];
    const down = y === h - 1 || !r.floor[c + w];
    const left = x === 0 || !r.floor[c - 1];
    const right = x === w - 1 || !r.floor[c + 1];
    if ((up && left) || (up && right) || (down && left) || (down && right)) { dead[c] = 1; continue; }
    // Wall runs: for each side with a wall, walk the run of cells sharing
    // that wall. A box on the run can only slide along it, so it is dead
    // when the run holds no goal and neither end lets it off the wall line
    // (an end is open when the next cell past it is floor without that wall
    // beside it, so the box can step away from the wall there).
    const sides: [boolean, number, number][] = [
      [up, 1, 0], [down, 1, 0], [left, 0, 1], [right, 0, 1],
    ];
    for (const [hasWall, sx, sy] of sides) {
      if (!hasWall) continue;
      let hasGoal = false;
      let exitOpen = false;
      for (const sgn of [1, -1]) {
        let cx = x, cy = y;
        for (;;) {
          const cc = cy * w + cx;
          if (goalSet[cc]) hasGoal = true;
          const nx = cx + sx * sgn, ny = cy + sy * sgn;
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) break; // board edge: closed end
          const nc = ny * w + nx;
          if (!r.floor[nc]) break; // wall past this end: closed end
          const above = ny === 0 || !r.floor[nc - w];
          const below = ny === h - 1 || !r.floor[nc + w];
          const lft = nx === 0 || !r.floor[nc - 1];
          const rgt = nx === w - 1 || !r.floor[nc + 1];
          const sideWall = sx === 1 ? (up ? above : below) : (left ? lft : rgt);
          if (!sideWall) { exitOpen = true; break; } // the box can leave the wall line here
          cx = nx; cy = ny;
        }
        if (hasGoal || exitOpen) break;
      }
      if (!hasGoal && !exitOpen) { dead[c] = 1; break; }
    }
  }

  const perms = permutations(k);
  const heuristic = (boxes: number[]): number => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < k; i++) {
        const d = distGoal[p[i]][boxes[i]];
        if (d < 0) { s = Infinity; break; }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };

  const isGoalState = (boxes: number[]): boolean => boxes.every((b) => goalSet[b]);
  const boxSetOf = (boxes: number[]): Uint8Array => {
    const s = new Uint8Array(n);
    for (const b of boxes) s[b] = 1;
    return s;
  };
  // Canonical player cell: lowest reachable cell given the boxes.
  const canon = (boxes: number[], from: number): number => {
    const blocked = boxSetOf(boxes);
    const { dist } = bfs(w, r.floor, from, blocked);
    let m = Infinity;
    for (let c = 0; c < n; c++) if (dist[c] !== -1 && c < m) m = c;
    return m === Infinity ? from : m;
  };
  // 2x2 freeze: every cell of the block wall-or-box and a box off-goal in it.
  const frozen = (boxes: number[], cell: number): boolean => {
    const set = boxSetOf(boxes);
    const x = cell % w, y = (cell / w) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        const bx = x + ox, by = y + oy;
        if (bx < 0 || by < 0 || bx + 1 >= w || by + 1 >= h) continue;
        const cells = [by * w + bx, by * w + bx + 1, (by + 1) * w + bx, (by + 1) * w + bx + 1];
        let solid = true, offGoal = false;
        for (const cc of cells) {
          if (r.floor[cc] && !set[cc]) { solid = false; break; }
          if (set[cc] && !goalSet[cc]) offGoal = true;
        }
        if (solid && offGoal) return true;
      }
    }
    return false;
  };

  // Generic push search. mode 'astar' is optimal; 'greedy' is best-first on h.
  // A state's key canonicalises the player (lowest reachable cell), but the
  // node itself keeps the player's ACTUAL cell: the walk of each transition
  // is emitted from where the player really stands, so the reconstructed
  // move string is a legal playthrough.
  const search = (mode: 'astar' | 'greedy', cap: number): RegionSolve => {
    const startBoxes = r.boxes.slice();
    const startPlayer = r.player;
    const startKey = startBoxes.join(',') + '|' + canon(startBoxes, r.player);
    if (isGoalState(startBoxes)) return { moves: '', pushes: 0, finalPlayer: startPlayer };
    if (startBoxes.some((b) => dead[b])) return null;
    const h0 = heuristic(startBoxes);
    if (!Number.isFinite(h0)) return null;
    // Binary heap ordered by (f, h, seq) for A*, (h, g, seq) for greedy.
    type Node = { f: number; h: number; g: number; seq: number; key: string; boxes: number[]; player: number };
    const heap: Node[] = [];
    let seq = 0;
    const less = (a: Node, b: Node): boolean => {
      if (mode === 'astar') {
        if (a.f !== b.f) return a.f < b.f;
      } else if (a.h !== b.h) return a.h < b.h;
      if (a.g !== b.g) return a.g < b.g;
      return a.seq < b.seq;
    };
    const pushHeap = (nd: Node): void => {
      heap.push(nd);
      let i = heap.length - 1;
      while (i > 0) {
        const p = (i - 1) >> 1;
        if (!less(heap[i], heap[p])) break;
        const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
        i = p;
      }
    };
    const popHeap = (): Node => {
      const top = heap[0];
      const last = heap.pop() as Node;
      if (heap.length > 0) {
        heap[0] = last;
        let i = 0;
        for (;;) {
          const l = i * 2 + 1, rr = l + 1;
          let m = i;
          if (l < heap.length && less(heap[l], heap[m])) m = l;
          if (rr < heap.length && less(heap[rr], heap[m])) m = rr;
          if (m === i) break;
          const t = heap[i]; heap[i] = heap[m]; heap[m] = t;
          i = m;
        }
      }
      return top;
    };
    const came = new Map<string, { prev: string; moves: string; player: number }>();
    const bestG = new Map<string, number>();
    pushHeap({ f: h0, h: h0, g: 0, seq: seq++, key: startKey, boxes: startBoxes, player: startPlayer });
    bestG.set(startKey, 0);
    let expanded = 0;
    while (heap.length > 0) {
      const nd = popHeap();
      if ((bestG.get(nd.key) ?? Infinity) < nd.g) continue;
      if (isGoalState(nd.boxes)) {
        let moves = '';
        let key = nd.key;
        while (came.has(key)) {
          const e = came.get(key) as { prev: string; moves: string; player: number };
          moves = e.moves + moves;
          key = e.prev;
        }
        return { moves, pushes: nd.g, finalPlayer: nd.player };
      }
      if (++expanded > cap) return null;
      const blocked = boxSetOf(nd.boxes);
      const reach = bfs(w, r.floor, nd.player, blocked);
      for (let bi = 0; bi < k; bi++) {
        const b = nd.boxes[bi];
        const bx = b % w, by = (b / w) | 0;
        for (const d of DIRS) {
          const dx = bx + d.dx, dy = by + d.dy;
          if (dx < 0 || dy < 0 || dx >= w || dy >= h) continue;
          const dest = dy * w + dx;
          if (!r.floor[dest] || blocked[dest] || dead[dest]) continue;
          const px = bx - d.dx, py = by - d.dy;
          if (px < 0 || py < 0 || px >= w || py >= h) continue;
          const behind = py * w + px;
          if (!r.floor[behind] || blocked[behind] || reach.dist[behind] === -1) continue;
          const nb = nd.boxes.slice();
          nb[bi] = dest;
          nb.sort((x, y) => x - y);
          if (frozen(nb, dest)) continue;
          const hh = heuristic(nb);
          if (!Number.isFinite(hh)) continue;
          // The player ends the push standing on the box's old cell.
          const nplayer = b;
          const nkey = nb.join(',') + '|' + canon(nb, b);
          const ng = nd.g + 1;
          if ((bestG.get(nkey) ?? Infinity) <= ng) continue;
          bestG.set(nkey, ng);
          const walk = pathTo(reach.prev, reach.prevCh, nd.player, behind);
          came.set(nkey, { prev: nd.key, moves: walk + d.ch, player: nplayer });
          pushHeap({ f: ng + hh, h: hh, g: ng, seq: seq++, key: nkey, boxes: nb, player: nplayer });
        }
      }
    }
    return null;
  };

  return search('astar', astarCap) ?? search('greedy', greedyCap);
}

/** Walk the player on the full board (boxes as obstacles); null when unreachable. */
function walkOnBoard(lv: Level, boxes: number[], from: number, to: number): string | null {
  const blocked = new Uint8Array(lv.w * lv.h);
  for (const b of boxes) blocked[b] = 1;
  const floor = new Uint8Array(lv.w * lv.h);
  for (let i = 0; i < floor.length; i++) floor[i] = lv.wall[i] ? 0 : 1;
  const { dist, prev, prevCh } = bfs(lv.w, floor, from, blocked);
  if (dist[to] === -1) return null;
  return pathTo(prev, prevCh, from, to);
}

/** Replay moves on a level; true when every box ends on a goal. */
function replay(lv: Level, moves: string): boolean {
  const box = new Uint8Array(lv.w * lv.h);
  for (const b of lv.boxes) box[b] = 1;
  let p = lv.player;
  for (const ch of moves) {
    const d = DIRS.find((x) => x.ch === ch) as { dx: number; dy: number };
    const nx = (p % lv.w) + d.dx, ny = ((p / lv.w) | 0) + d.dy;
    if (nx < 0 || ny < 0 || nx >= lv.w || ny >= lv.h) return false;
    const ni = ny * lv.w + nx;
    if (lv.wall[ni]) return false;
    if (box[ni]) {
      const bx = nx + d.dx, by = ny + d.dy;
      if (bx < 0 || by < 0 || bx >= lv.w || by >= lv.h) return false;
      const bi = by * lv.w + bx;
      if (lv.wall[bi] || box[bi]) return false;
      box[ni] = 0; box[bi] = 1;
    }
    p = ni;
  }
  for (let i = 0; i < box.length; i++) if (box[i] && !lv.goal[i]) return false;
  return true;
}

/** The moves for a level in XSB: a string of u, d, l and r. */
const solve = (levelText: string): string => {
  try {
    const lv = parseLevel(levelText);
    if (lv.player < 0 || lv.boxes.length === 0) return '';
    const totalGoals = (() => { let c = 0; for (let i = 0; i < lv.goal.length; i++) c += lv.goal[i]; return c; })();
    if (totalGoals !== lv.boxes.length) return '';

    // Decompose: drop the hallway row (the player's row); each remaining
    // floor component holding boxes is one room, with its door cells
    // adjacent to hallway floor and its entry cells in the hallway row.
    const hallRow = (lv.player / lv.w) | 0;
    const seen = new Uint8Array(lv.w * lv.h);
    type Room = { cells: number[]; boxes: number[]; goals: number[]; doors: number[]; entries: number[] };
    const rooms: Room[] = [];
    let allCovered = true;
    for (let y = 0; y < lv.h; y++) {
      for (let x = 0; x < lv.w; x++) {
        const c = y * lv.w + x;
        if (y === hallRow || lv.wall[c] || seen[c]) continue;
        const q = [c];
        seen[c] = 1;
        const cells: number[] = [];
        for (let qi = 0; qi < q.length; qi++) {
          const cc = q[qi];
          cells.push(cc);
          const cx = cc % lv.w, cy = (cc / lv.w) | 0;
          for (const d of DIRS) {
            const nx = cx + d.dx, ny = cy + d.dy;
            if (nx < 0 || ny < 0 || nx >= lv.w || ny >= lv.h) continue;
            const ni = ny * lv.w + nx;
            if (ny === hallRow || lv.wall[ni] || seen[ni]) continue;
            seen[ni] = 1;
            q.push(ni);
          }
        }
        const cellSet = new Set(cells);
        const boxes = lv.boxes.filter((b) => cellSet.has(b));
        if (boxes.length === 0) continue;
        const goals = [];
        for (const cc of cells) if (lv.goal[cc]) goals.push(cc);
        const doors = cells.filter((cc) => {
          const cx = cc % lv.w, cy = (cc / lv.w) | 0;
          return DIRS.some((d) => {
            const nx = cx + d.dx, ny = cy + d.dy;
            return ny === hallRow && nx >= 0 && nx < lv.w && !lv.wall[ny * lv.w + nx];
          });
        });
        const entries = [...new Set(doors.flatMap((cc) => {
          const cx = cc % lv.w, cy = (cc / lv.w) | 0;
          return DIRS.filter((d) => cy + d.dy === hallRow)
            .map((d) => (cy + d.dy) * lv.w + (cx + d.dx))
            .filter((ni) => ni >= 0 && ni < lv.w * lv.h && !lv.wall[ni]);
        }))].sort((a, b) => a - b);
        if (boxes.length !== goals.length || doors.length === 0 || entries.length === 0) allCovered = false;
        rooms.push({ cells, boxes, goals, doors, entries });
      }
    }
    const coveredBoxes = rooms.reduce((a, r) => a + r.boxes.length, 0);
    if (!allCovered || coveredBoxes !== lv.boxes.length || rooms.length === 0) {
      // Fall back to solving the whole level as one region.
      const floor = new Uint8Array(lv.w * lv.h);
      for (let i = 0; i < floor.length; i++) floor[i] = lv.wall[i] ? 0 : 1;
      const goals = [];
      for (let i = 0; i < lv.goal.length; i++) if (lv.goal[i]) goals.push(i);
      const r = solveRegion({ w: lv.w, h: lv.h, floor, goals, boxes: lv.boxes.slice(), player: lv.player }, 800000, 400000);
      return r !== null && replay(lv, r.moves) ? r.moves : '';
    }

    rooms.sort((a, b) => a.entries[0] - b.entries[0]);
    let moves = '';
    let boxes = lv.boxes.slice();
    let player = lv.player;
    for (const room of rooms) {
      if (room.boxes.every((b) => lv.goal[b])) continue; // already solved
      const entry = room.entries[0];
      const walk = walkOnBoard(lv, boxes, player, entry);
      if (walk === null) return moves; // legal prefix only; cannot reach this room
      // Current box positions inside this room (boxes never leave it).
      const cellSet = new Set(room.cells);
      const curBoxes = boxes.filter((b) => cellSet.has(b)).sort((a, b) => a - b);
      const floor = new Uint8Array(lv.w * lv.h);
      for (const cc of room.cells) floor[cc] = 1;
      for (const e of room.entries) floor[e] = 1;
      const solved = solveRegion(
        { w: lv.w, h: lv.h, floor, goals: room.goals.slice(), boxes: curBoxes, player: entry },
        800000, 400000,
      );
      if (solved === null) return moves; // legal prefix; this room is unbeaten
      moves += walk + solved.moves;
      // Apply the room solution to the full board.
      const boxArr = new Uint8Array(lv.w * lv.h);
      for (const b of boxes) boxArr[b] = 1;
      let p = entry;
      // The walk already moved the player to the entry; replay room moves.
      for (const ch of solved.moves) {
        const d = DIRS.find((x) => x.ch === ch) as { dx: number; dy: number };
        const nx = (p % lv.w) + d.dx, ny = ((p / lv.w) | 0) + d.dy;
        const ni = ny * lv.w + nx;
        if (boxArr[ni]) {
          const bi = (ny + d.dy) * lv.w + (nx + d.dx);
          boxArr[ni] = 0; boxArr[bi] = 1;
        }
        p = ni;
      }
      boxes = [];
      for (let i = 0; i < boxArr.length; i++) if (boxArr[i]) boxes.push(i);
      player = p;
    }
    return replay(lv, moves) ? moves : '';
  } catch {
    return '';
  }
};

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
