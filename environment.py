import csv
import os
import sys
import time
from collections import deque

import cv2
import numpy as np

# ---------- Configuration ----------
WIDTH, HEIGHT = 640, 480
CELL = 20
COLS, ROWS = WIDTH // CELL, HEIGHT // CELL

WHITE  = (255, 255, 255)
RED    = (0, 0, 255)
BLUE   = (255, 0, 0)
GREEN  = (0, 200, 0)
ORANGE = (0, 140, 255)
GRAY   = (220, 220, 220)
YELLOW = (0, 255, 255)
PURPLE = (200, 0, 150)

START = (40, 40)
GOAL  = (580, 430)
STEP = 10
AUTO_SPEED = 4


# ---------- Day 12: test scenarios ----------
def cell_box(r1, c1, r2, c2):
    """Pixel box (x, y, w, h) that covers exactly cells r1..r2 and c1..c2.
    cv2.rectangle draws edge pixels inclusively, so w and h are 1 px short of a
    whole number of cells; that keeps the last pixel inside the last cell."""
    return (c1 * CELL, r1 * CELL, (c2 - c1 + 1) * CELL - 1, (r2 - r1 + 1) * CELL - 1)


ORIGINAL_OBSTACLES = [(200, 100, 80, 120), (360, 260, 100, 60), (120, 340, 60, 80)]

SCENARIOS = {
    "1 Empty": dict(
        obstacles=[], start=START, goal=GOAL, expect_route=True),
    "2 Long wall": dict(          # gap only above the start row -> real detour
        obstacles=[cell_box(2, 15, 23, 15)],
        start=START, goal=GOAL, expect_route=True),
    "3 Original": dict(           # the three obstacles used since Day 1
        obstacles=list(ORIGINAL_OBSTACLES),
        start=START, goal=GOAL, expect_route=True),
    "4 Corridor": dict(           # one-cell-wide, three-cell-long passage
        obstacles=[cell_box(0, 14, 10, 16), cell_box(12, 14, 23, 16)],
        start=START, goal=GOAL, expect_route=True),
    "5 Zigzag": dict(             # two walls with gaps on opposite sides
        obstacles=[cell_box(0, 16, 9, 16), cell_box(13, 16, 23, 16),
                   cell_box(3, 23, 23, 23)],
        start=START, goal=GOAL, expect_route=True),
    "6 Goal sealed": dict(        # goal in a closed pocket -> no route exists
        obstacles=[(561, 380, 78, 19), (540, 400, 19, 79)],
        start=START, goal=GOAL, expect_route=False),
}
SCENARIO_NAMES = list(SCENARIOS)


# ---------- Coordinate system ----------
def pixel_to_cell(x, y):
    return (y // CELL, x // CELL)

def cell_to_pixel(row, col):
    return (col * CELL + CELL // 2, row * CELL + CELL // 2)


# ---------- Rendering ----------
def render(drone_pos, obstacles, start, goal, show_grid=False):
    frame = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)

    if show_grid:
        for c in range(0, WIDTH, CELL):
            cv2.line(frame, (c, 0), (c, HEIGHT), GRAY, 1)
        for r in range(0, HEIGHT, CELL):
            cv2.line(frame, (0, r), (WIDTH, r), GRAY, 1)

    for (x, y, w, h) in obstacles:
        cv2.rectangle(frame, (x, y), (x + w, y + h), RED, -1)

    cv2.circle(frame, start, 8, GREEN, -1)
    cv2.putText(frame, "START", (start[0] - 20, start[1] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREEN, 1)

    cv2.circle(frame, goal, 8, ORANGE, -1)
    cv2.putText(frame, "GOAL", (goal[0] - 18, goal[1] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, ORANGE, 1)

    cv2.circle(frame, drone_pos, 7, BLUE, -1)

    return frame


# ---------- Detection (works on pixels only) ----------
def detect_obstacles(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask1 = cv2.inRange(hsv, np.array([0, 120, 70]),   np.array([10, 255, 255]))
    mask2 = cv2.inRange(hsv, np.array([170, 120, 70]), np.array([180, 255, 255]))
    mask = mask1 | mask2
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    boxes = []
    for cnt in contours:
        if cv2.contourArea(cnt) > 100:
            boxes.append(cv2.boundingRect(cnt))
    return boxes


def check_detection(detected, obstacles):
    print("Detected boxes (x, y, w, h):")
    for box in sorted(detected):
        print("  ", box)
    print("True obstacles:")
    for box in sorted(obstacles):
        print("  ", box)


def hits_obstacle(x, y, obstacles):
    for (ox, oy, w, h) in obstacles:
        if ox <= x <= ox + w and oy <= y <= oy + h:
            return True
    return False


# ---------- Occupancy grid ----------
def build_occupancy_grid(detected, rows=ROWS, cols=COLS, cell=CELL):
    grid = [[0 for _ in range(cols)] for _ in range(rows)]
    for (x, y, w, h) in detected:
        x_end = x + w - 1
        y_end = y + h - 1
        r1, c1 = pixel_to_cell(x, y)
        r2, c2 = pixel_to_cell(x_end, y_end)
        for r in range(min(r1, r2), max(r1, r2) + 1):
            for c in range(min(c1, c2), max(c1, c2) + 1):
                if 0 <= r < rows and 0 <= c < cols:
                    grid[r][c] = 1
    return grid


def print_grid(grid):
    for row in grid:
        print(''.join('#' if cell else '.' for cell in row))


# ---------- A* path planning (from scratch) ----------
def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(grid, start_cell, goal_cell):
    rows = len(grid)
    cols = len(grid[0])

    def neighbors(cell):
        r, c = cell
        candidates = [(r-1, c), (r+1, c), (r, c-1), (r, c+1)]
        result = []
        for nr, nc in candidates:
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                result.append((nr, nc))
        return result

    open_list = [start_cell]
    came_from = {}
    g_score = {start_cell: 0}
    f_score = {start_cell: heuristic(start_cell, goal_cell)}
    closed_set = set()

    while open_list:
        current = min(open_list, key=lambda cell: f_score.get(cell, float('inf')))

        if current == goal_cell:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        open_list.remove(current)
        closed_set.add(current)

        for neighbor in neighbors(current):
            if neighbor in closed_set:
                continue
            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal_cell)
                if neighbor not in open_list:
                    open_list.append(neighbor)

    return None


def step_towards(current, target, speed):
    cx, cy = current
    tx, ty = target
    dx, dy = tx - cx, ty - cy
    dist = (dx ** 2 + dy ** 2) ** 0.5

    if dist <= speed:
        return (tx, ty), True

    nx = cx + speed * dx / dist
    ny = cy + speed * dy / dist
    return (int(nx), int(ny)), False


# ---------- Day 12: headless test harness ----------
def bfs_shortest(grid, start, goal):
    """Independent shortest-path check (breadth-first search).
    Returns the number of moves, or None if the goal is unreachable."""
    rows, cols = len(grid), len(grid[0])
    dist = {start: 0}
    queue = deque([start])
    while queue:
        cur = queue.popleft()
        if cur == goal:
            return dist[cur]
        r, c = cur
        for nr, nc in ((r - 1, c), (r + 1, c), (r, c - 1), (r, c + 1)):
            if (0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0
                    and (nr, nc) not in dist):
                dist[(nr, nc)] = dist[cur] + 1
                queue.append((nr, nc))
    return None


def dynamic_test(obstacles, start, goal, goal_cell, path):
    """Same rule as the N key: a third of the way along the path, drop a 40x40
    obstacle 5 cells ahead, re-detect, and replan if the path is now blocked.
    Returns (label, replans)."""
    if path is None:
        return "n/a (no route)", 0
    idx = len(path) // 3
    ahead = idx + 5
    if ahead > len(path) - 4:
        return "skipped (short path)", 0

    ax, ay = cell_to_pixel(*path[ahead])
    obstacles2 = obstacles + [(ax - 20, ay - 20, 40, 40)]
    frame2 = render(cell_to_pixel(*path[idx]), obstacles2, start, goal, False)
    grid2 = build_occupancy_grid(detect_obstacles(frame2))

    if not any(grid2[r][c] for (r, c) in path[idx:]):
        return "no replan needed", 0

    current = path[idx]
    new_path = astar(grid2, current, goal_cell)
    ref = bfs_shortest(grid2, current, goal_cell)
    if new_path is None:
        return ("no route (correct)" if ref is None else "MISMATCH"), 1
    if ref is None:
        return "MISMATCH", 1
    return ("OK" if len(new_path) - 1 == ref else "SUBOPTIMAL"), 1


def run_scenario(name, sc):
    obstacles, start, goal = sc["obstacles"], sc["start"], sc["goal"]

    # Pipeline: render -> detect -> occupancy grid -> A*
    frame = render(start, obstacles, start, goal, False)
    t0 = time.perf_counter()
    detected = detect_obstacles(frame)
    grid = build_occupancy_grid(detected)
    detect_ms = (time.perf_counter() - t0) * 1000

    # Ground truth: cells touched by the drawn pixels (drawn size is w+1, h+1)
    truth = build_occupancy_grid([(x, y, w + 1, h + 1) for (x, y, w, h) in obstacles])
    grid_match = (grid == truth)

    start_cell = pixel_to_cell(*start)
    goal_cell = pixel_to_cell(*goal)

    t0 = time.perf_counter()
    path = astar(grid, start_cell, goal_cell)
    plan_ms = (time.perf_counter() - t0) * 1000

    ref = bfs_shortest(grid, start_cell, goal_cell)
    optimal = (path is None and ref is None) or \
              (path is not None and ref is not None and len(path) - 1 == ref)
    outcome_ok = (path is not None) == sc["expect_route"]

    dyn_label, replans = dynamic_test(obstacles, start, goal, goal_cell, path)
    dyn_ok = dyn_label not in ("MISMATCH", "SUBOPTIMAL")

    passed = outcome_ok and grid_match and optimal and dyn_ok
    return dict(
        scenario=name,
        obstacles_found=f"{len(detected)}/{len(obstacles)}",
        grid_matches_truth="ok" if grid_match else "DIFF",
        expected="route" if sc["expect_route"] else "no route",
        path_cells=len(path) if path else "none",
        manhattan_min_cells=heuristic(start_cell, goal_cell) + 1,
        optimal_vs_bfs="yes" if optimal else "NO",
        detect_ms=round(detect_ms, 2),
        plan_ms=round(plan_ms, 2),
        dynamic_replan=dyn_label,
        replans=replans,
        result="PASS" if passed else "FAIL",
    )


def run_tests():
    rows = [run_scenario(name, SCENARIOS[name]) for name in SCENARIO_NAMES]

    print()
    print(f"{'Scenario':<15}{'Obst':>6}{'Grid':>6}{'Path':>6}{'Min':>5}{'Opt':>5}"
          f"{'Plan ms':>9}  {'Dynamic replan':<22}{'Result':>7}")
    print("-" * 83)
    for r in rows:
        print(f"{r['scenario']:<15}{r['obstacles_found']:>6}{r['grid_matches_truth']:>6}"
              f"{str(r['path_cells']):>6}{r['manhattan_min_cells']:>5}"
              f"{r['optimal_vs_bfs']:>5}{r['plan_ms']:>9}  "
              f"{r['dynamic_replan']:<22}{r['result']:>7}")
    passed = sum(1 for r in rows if r["result"] == "PASS")
    print("-" * 83)
    print(f"{passed}/{len(rows)} scenarios passed")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_results.csv")
    with open(out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"Saved: {out}")


# ---------- Interactive simulation ----------
def new_world(name):
    sc = SCENARIOS[name]
    world = {
        "name": name,
        "obstacles": list(sc["obstacles"]),
        "start": sc["start"],
        "goal": sc["goal"],
        "drone": list(sc["start"]),
        "auto": False,
        "path": None,
        "pixel_points": [],
        "index": 0,
        "reached": False,
        "replans": 0,
    }

    frame = render(tuple(world["drone"]), world["obstacles"],
                   world["start"], world["goal"], False)
    detected = detect_obstacles(frame)
    print(f"\n=== Scenario: {name} ===")
    check_detection(detected, world["obstacles"])

    occupancy = build_occupancy_grid(detected)
    print("Occupancy grid (24 rows x 32 cols):")
    print_grid(occupancy)

    start_cell = pixel_to_cell(*world["start"])
    goal_cell = pixel_to_cell(*world["goal"])
    path = astar(occupancy, start_cell, goal_cell)

    if path is None:
        print("No path found from START to GOAL!")
    else:
        print(f"Path found with {len(path)} cells")
        world["path"] = path
        world["pixel_points"] = [cell_to_pixel(r, c) for (r, c) in path]
    return world


def main():
    world = new_world("3 Original")
    show_grid = True

    while True:
        drone = world["drone"]
        obstacles = world["obstacles"]
        goal_cell = pixel_to_cell(*world["goal"])

        frame = render(tuple(drone), obstacles, world["start"], world["goal"], show_grid)

        detected = detect_obstacles(frame)
        for (x, y, w, h) in detected:
            cv2.rectangle(frame, (x, y), (x + w, y + h), YELLOW, 2)

        # Recompute occupancy every frame so a new obstacle is noticed at once
        occupancy_now = build_occupancy_grid(detected)

        pixel_points = world["pixel_points"]
        if pixel_points:
            for i in range(len(pixel_points) - 1):
                cv2.line(frame, pixel_points[i], pixel_points[i + 1], PURPLE, 2)
            for p in pixel_points:
                cv2.circle(frame, p, 2, PURPLE, -1)

        r, c = pixel_to_cell(drone[0], drone[1])
        mode_text = "AUTO-FOLLOW" if world["auto"] else "MANUAL"
        cv2.putText(frame, f"Scenario: {world['name']}", (10, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)
        cv2.putText(frame, f"cell=({r},{c})  mode={mode_text}  replans={world['replans']}",
                    (10, HEIGHT - 26), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)
        cv2.putText(frame, "[WASD] [F]=auto [N]=new obstacle [1-6]=scenario [R]=reset [G]=grid [Q]=quit",
                    (10, HEIGHT - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 0), 1)

        cv2.imshow("Drone Simulation", frame)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('g'):
            show_grid = not show_grid
        elif key == ord('r'):
            world = new_world(world["name"])
            continue
        elif ord('1') <= key <= ord('0') + len(SCENARIO_NAMES):
            world = new_world(SCENARIO_NAMES[key - ord('1')])
            continue
        elif key == ord('f'):
            if not world["auto"] and not pixel_points:
                print("No path to follow in this scenario.")
            else:
                world["auto"] = not world["auto"]
                print("Auto-follow ON" if world["auto"] else "Auto-follow OFF")
        elif key == ord('n'):
            # Drop a new obstacle a few cells ahead of the drone on the current path
            if pixel_points and world["index"] < len(pixel_points):
                ahead_index = min(world["index"] + 5, len(pixel_points) - 1)
                ax, ay = pixel_points[ahead_index]
                new_box = (ax - 20, ay - 20, 40, 40)
                obstacles.append(new_box)
                print(f"New obstacle spawned ahead at {new_box}")
            else:
                print("Cannot spawn obstacle -- no active path to spawn ahead on.")

        # Check if the remaining path is still clear; replan if not
        path = world["path"]
        if world["auto"] and path and world["index"] < len(path):
            remaining_cells = path[world["index"]:]
            blocked_ahead = any(occupancy_now[rr][cc] == 1 for (rr, cc) in remaining_cells)

            if blocked_ahead:
                current_cell = pixel_to_cell(drone[0], drone[1])
                new_path = astar(occupancy_now, current_cell, goal_cell)
                if new_path:
                    world["path"] = new_path
                    world["pixel_points"] = [cell_to_pixel(rr, cc) for (rr, cc) in new_path]
                    world["index"] = 0
                    world["replans"] += 1
                    print(f"Obstacle blocked the path -- replanned successfully "
                          f"(replan #{world['replans']}).")
                else:
                    print("Obstacle blocked the path -- no alternative route found. "
                          "Stopping auto-follow.")
                    world["auto"] = False

        pixel_points = world["pixel_points"]
        if world["auto"] and pixel_points and world["index"] < len(pixel_points):
            target = pixel_points[world["index"]]
            new_pos, reached = step_towards(tuple(drone), target, AUTO_SPEED)
            drone[0], drone[1] = new_pos[0], new_pos[1]
            if reached:
                world["index"] += 1
            if world["index"] >= len(pixel_points) and not world["reached"]:
                print("Reached GOAL via planned path!")
                world["reached"] = True

        elif not world["auto"]:
            dx = dy = 0
            if key == ord('w'):   dy = -STEP
            elif key == ord('s'): dy = STEP
            elif key == ord('a'): dx = -STEP
            elif key == ord('d'): dx = STEP

            nx = min(max(drone[0] + dx, 0), WIDTH - 1)
            ny = min(max(drone[1] + dy, 0), HEIGHT - 1)
            if not hits_obstacle(nx, ny, obstacles):
                drone[0], drone[1] = nx, ny

    cv2.destroyAllWindows()


if __name__ == "__main__":
    if "--test" in sys.argv:
        run_tests()
    else:
        main()
