# AI-Based Obstacle Detection and Path Planning for a Simulated Autonomous Drone

BSERC Defence-Space Tech Internship Project.

## Tech Stack

Python + OpenCV (single technology, per project scope)

## Progress

- [x] Day 1: Simulated environment - canvas, obstacle, drone, goal rendering
- [x] Day 2: Drone movement via keyboard controls (W/A/S/D)
- [x] Day 3: Start/goal markers, pixel-to-grid coordinate mapping
- [x] Day 4: Render loop, WASD control with obstacle collision, cell readout
- [x] Day 5: Cleanup, first GitHub commit, README status
- [x] Day 6: Obstacle detection from pixels (HSV mask + cv2.findContours)
- [x] Day 7: Verified detected boxes against true obstacles (all 3 matched)
- [x] Day 8: Detected boxes converted to a 24x32 occupancy grid
- [x] Day 9: A* path planning implemented from scratch (47-cell path START to GOAL)
- [x] Day 10: Drone auto-follows the A* path (F key) and reaches GOAL exactly
- [x] Day 11: Dynamic replanning when new obstacles appear mid-flight (N key)
- [x] Day 12: Six test scenarios, BFS cross-check of A*, headless test mode
- [ ] Day 13: Scenario demo videos, final documentation
- [ ] Final polish and supervisor update

## How It Works

1. A simulated environment is drawn on an OpenCV canvas (640x480, 20 px cells, 24 x 32 grid).
2. Red obstacles are detected from the rendered frame only (HSV mask + cv2.findContours).
3. Detected boxes become a binary occupancy grid (0 = free, 1 = blocked).
4. A* (written from scratch, Manhattan heuristic, 4-directional) plans a path from START to GOAL.
5. The drone follows the path. If a new obstacle blocks the path ahead, A* is re-run from the drone's current cell.

## How to Run

Interactive window:

```
python environment.py
```

Keys: F = auto-follow, WASD = manual, N = new obstacle, 1-6 = load scenario, R = reset, G = grid, Q = quit.

Headless tests (writes test_results.csv):

```
python environment.py --test
```

## Test Results (Day 12)

Every A* path is checked against an independent breadth-first search (BFS).

| # | Scenario | Path (cells) | Result |
|---|----------|--------------|--------|
| 1 | Empty | 47 | PASS, optimal |
| 2 | Long wall | 49 | PASS, optimal |
| 3 | Original | 47 | PASS, optimal |
| 4 | Corridor | 47 | PASS, optimal |
| 5 | Zigzag | 63 | PASS, optimal |
| 6 | Goal sealed | none | PASS, correctly reports no route |

6 of 6 scenarios passed. Full details are in test_results.csv.

## Notes

Detected width and height are 1 px larger than the drawn values because OpenCV draws both edge pixels. The occupancy grid uses x + w - 1 and y + h - 1 as the last pixel.
