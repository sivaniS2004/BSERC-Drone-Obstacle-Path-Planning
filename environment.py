import cv2
import numpy as np

# ---------- Configuration ----------
WIDTH, HEIGHT = 640, 480
CELL = 20                                    # each grid cell = 20x20 px
COLS, ROWS = WIDTH // CELL, HEIGHT // CELL   # 32 columns x 24 rows

# Colors (BGR order in OpenCV)
WHITE  = (255, 255, 255)
RED    = (0, 0, 255)
BLUE   = (255, 0, 0)
GREEN  = (0, 200, 0)
ORANGE = (0, 140, 255)
GRAY   = (220, 220, 220)

# Obstacles as (x, y, w, h) in pixels
OBSTACLES = [(200, 100, 80, 120), (360, 260, 100, 60), (120, 340, 60, 80)]

START = (40, 40)      # (x, y) in pixels
GOAL  = (580, 430)
STEP = 10             # drone moves 10 px per keypress


# ---------- Coordinate system ----------
# Pixels use (x, y): x = horizontal, y = vertical
# Grid cells use (row, col): row = vertical, col = horizontal
def pixel_to_cell(x, y):
    return (y // CELL, x // CELL)

def cell_to_pixel(row, col):
    return (col * CELL + CELL // 2, row * CELL + CELL // 2)  # cell center


# ---------- Rendering ----------
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


def hits_obstacle(x, y):
    for (ox, oy, w, h) in OBSTACLES:
        if ox <= x <= ox + w and oy <= y <= oy + h:
            return True
    return False


def main():
    drone = list(START)
    show_grid = True

    while True:
        frame = render(tuple(drone), show_grid)

        r, c = pixel_to_cell(drone[0], drone[1])
        cv2.putText(frame, f"cell (row,col)=({r},{c})  [WASD]=move [G]=grid [Q]=quit",
                    (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

        cv2.imshow("Drone Simulation", frame)
        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('g'):
            show_grid = not show_grid

        dx = dy = 0
        if key == ord('w'):   dy = -STEP
        elif key == ord('s'): dy = STEP
        elif key == ord('a'): dx = -STEP
        elif key == ord('d'): dx = STEP

        nx = min(max(drone[0] + dx, 0), WIDTH - 1)
        ny = min(max(drone[1] + dy, 0), HEIGHT - 1)
        if not hits_obstacle(nx, ny):
            drone = [nx, ny]

        if abs(drone[0] - GOAL[0]) < 10 and abs(drone[1] - GOAL[1]) < 10:
            print("Goal reached!")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()