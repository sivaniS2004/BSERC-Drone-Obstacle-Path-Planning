import cv2
import numpy as np

WIDTH, HEIGHT = 800, 600
STEP = 10  # pixels moved per keypress

def main():
    canvas = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255

    # Drone starting position (mutable, so we can update it)
    drone_x, drone_y = 100, 100

    # Obstacle and goal stay fixed for now
    obstacle_tl = (300, 200)   # top-left corner
    obstacle_br = (400, 300)   # bottom-right corner
    goal = (700, 500)

    while True:
        frame = canvas.copy()

        # Draw obstacle
        cv2.rectangle(frame, obstacle_tl, obstacle_br, (0, 0, 255), -1)

        # Draw goal
        cv2.circle(frame, goal, 15, (0, 255, 0), -1)

        # Draw drone at its current position
        cv2.circle(frame, (drone_x, drone_y), 15, (255, 0, 0), -1)

        cv2.imshow("Drone Simulation - Environment", frame)

        key = cv2.waitKey(30) & 0xFF

        if key == ord('q'):
            break
        elif key == ord('w'):  # up
            drone_y = max(15, drone_y - STEP)
        elif key == ord('s'):  # down
            drone_y = min(HEIGHT - 15, drone_y + STEP)
        elif key == ord('a'):  # left
            drone_x = max(15, drone_x - STEP)
        elif key == ord('d'):  # right
            drone_x = min(WIDTH - 15, drone_x + STEP)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()