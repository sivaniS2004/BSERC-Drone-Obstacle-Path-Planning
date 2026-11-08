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

OBSTACLES = [(200, 100, 80, 120), (360, 260, 100, 60), (120, 340, 60, 80)]

START = (40, 40)
GOAL  = (580, 430)
STEP = 10
AUTO_SPEED = 4


def pixel_to_cell(x, y):
    return (y // CELL, x // CELL)

def cell_to_pixel(row, col):
    return (col * CELL + CELL // 2, row * CELL + CELL // 2)


def render(drone_pos, show_grid=False):
    frame = np.full((HEIGHT, WIDTH, 3), 255, dtype=np.uint8)

    if show_grid:
        for c in range(0, WIDTH, CELL):
            cv2.line(frame, (c, 0), (c, HEIGHT), GRAY, 1)
        for r in range(0, HEIGHT, CELL):
            cv2.line(frame, (0, r), (WIDTH, r), GRAY, 1)

    for (x, y, w, h) in OBSTACLES:
        cv2.rectangle(frame, (x, y), (x + w, y + h), RED, -1)

    cv2.circle(frame, START, 8, GREEN, -1)
    cv2.putText(frame, "START", (START[0] - 20, START[1] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, GREEN, 1)

    cv2.circle(frame, GOAL, 8, ORANGE, -1)
    cv2.putText(frame, "GOAL", (GOAL[0] - 18, GOAL[1] - 14),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, ORANGE, 1)

    cv2.circle(frame, drone_pos, 7, BLUE, -1)

    return frame


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


def check_detection(detected):
    print("Detected boxes (x, y, w, h):")
    for box in sorted(detected):
        print("  ", box)
    print("True obstacles:")
    for box in sorted(OBSTACLES):
        print("  ", box)


def hits_obstacle(x, y):
    for (ox, oy, w, h) in OBSTACLES:
        if ox <= x <= ox + w and oy <= y <= oy + h:
            return True
    return False


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


def main():
    drone = list(START)
    show_grid = True
    auto_follow = False

    detected_at_start = detect_obstacles(render(tuple(drone), False))
    check_detection(detected_at_start)

    occupancy = build_occupancy_grid(detected_at_start)
    print("Occupancy grid (24 rows x 32 cols):")
    print_grid(occupancy)

    start_cell = pixel_to_cell(START[0], START[1])
    goal_cell = pixel_to_cell(GOAL[0], GOAL[1])
    path = astar(occupancy, start_cell, goal_cell)

    if path is None:
        print("No path found from START to GOAL!")
        pixel_points = []
    else:
        print(f"Path found with {len(path)} cells:")
        pixel_points = [cell_to_pixel(r, c) for (r, c) in path]

    path_index = 0
    reached_goal_via_path = False

    while True:
        frame = render(tuple(drone), show_grid)

        detected = detect_obstacles(frame)
        for (x, y, w, h) in detected:
            cv2.rectangle(frame, (x, y), (x + w, y + h), YELLOW, 2)

        # Day 11: recompute occupancy every frame so we can notice a new obstacle
        occupancy_now = build_occupancy_grid(detected)

        if pixel_points:
            for i in range(len(pixel_points) - 1):
                cv2.line(frame, pixel_points[i], pixel_points[i + 1], PURPLE, 2)
            for p in pixel_points:
                cv2.circle(frame, p, 2, PURPLE, -1)

        r, c = pixel_to_cell(drone[0], drone[1])
        mode_text = "AUTO-FOLLOW" if auto_follow else "MANUAL"
        cv2.putText(frame,
                    f"cell=({r},{c})  mode={mode_text}  [WASD] [F]=auto [N]=new obstacle [G]=grid [Q]=quit",
                    (10, HEIGHT - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 0), 1)

        cv2.imshow("Drone Simulation", frame)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('g'):
            show_grid = not show_grid
        elif key == ord('f'):
            auto_follow = not auto_follow
            print("Auto-follow ON" if auto_follow else "Auto-follow OFF")
        elif key == ord('n'):
            # Day 11: drop a new obstacle a few cells ahead of the drone on the current path
            if pixel_points and path_index < len(pixel_points):
                ahead_index = min(path_index + 5, len(pixel_points) - 1)
                ax, ay = pixel_points[ahead_index]
                new_box = (ax - 20, ay - 20, 40, 40)
                OBSTACLES.append(new_box)
                print(f"New obstacle spawned ahead at {new_box}")
            else:
                print("Cannot spawn obstacle -- no active path to spawn ahead on.")

        # Day 11: check if the remaining path is still clear; replan if not
        if auto_follow and path and path_index < len(path):
            remaining_cells = path[path_index:]
            blocked_ahead = any(occupancy_now[rr][cc] == 1 for (rr, cc) in remaining_cells)

            if blocked_ahead:
                current_cell = pixel_to_cell(drone[0], drone[1])
                new_path = astar(occupancy_now, current_cell, goal_cell)
                if new_path:
                    path = new_path
                    pixel_points = [cell_to_pixel(rr, cc) for (rr, cc) in path]
                    path_index = 0
                    print("Obstacle blocked the path -- replanned successfully.")
                else:
                    print("Obstacle blocked the path -- no alternative route found. Stopping auto-follow.")
                    auto_follow = False

        if auto_follow and pixel_points and path_index < len(pixel_points):
            target = pixel_points[path_index]
            new_pos, reached = step_towards(tuple(drone), target, AUTO_SPEED)
            drone = [new_pos[0], new_pos[1]]
            if reached:
                path_index += 1
            if path_index >= len(pixel_points) and not reached_goal_via_path:
                print("Reached GOAL via planned path!")
                reached_goal_via_path = True

        elif not auto_follow:
            dx = dy = 0
            if key == ord('w'):   dy = -STEP
            elif key == ord('s'): dy = STEP
            elif key == ord('a'): dx = -STEP
            elif key == ord('d'): dx = STEP

            nx = min(max(drone[0] + dx, 0), WIDTH - 1)
            ny = min(max(drone[1] + dy, 0), HEIGHT - 1)
            if not hits_obstacle(nx, ny):
                drone = [nx, ny]

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()