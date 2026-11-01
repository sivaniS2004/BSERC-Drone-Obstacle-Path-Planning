import cv2
import numpy as np

WIDTH, HEIGHT = 800, 600

def main():
    canvas = np.ones((HEIGHT, WIDTH, 3), dtype=np.uint8) * 255

    while True:
        frame = canvas.copy()

        # Obstacle (red rectangle)
        cv2.rectangle(frame, (300, 200), (400, 300), (0, 0, 255), -1)

        # Drone (blue circle, top-left start position)
        cv2.circle(frame, (100, 100), 15, (255, 0, 0), -1)

        # Goal (green circle, bottom-right)
        cv2.circle(frame, (700, 500), 15, (0, 255, 0), -1)

        cv2.imshow("Drone Simulation - Environment", frame)

        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()