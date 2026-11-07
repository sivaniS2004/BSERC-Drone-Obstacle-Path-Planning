\# AI-Based Obstacle Detection and Path Planning for a Simulated Autonomous Drone



BSERC Defence-Space Tech Internship Project.



\## Tech Stack

Python + OpenCV (single technology, per project scope)



\## Progress

\- \[x] Day 1: Simulated environment - canvas, obstacle, drone, goal rendering

\- \[x] Day 2: Drone movement via keyboard controls (W/A/S/D)

\- \[x] Day 3: Start/goal markers, pixel-to-grid coordinate mapping

\- \[x] Day 4: Render loop, WASD control with obstacle collision, cell readout

\- \[x] Day 5: Cleanup, first GitHub commit, README status

\- \[x] Day 6: Obstacle detection from pixels (HSV mask + cv2.findContours)

\- \[x] Day 7: Verified detected boxes against true obstacles (all 3 matched)

\- \[x] Day 8: Obstacle boxes converted to a 24x32 occupancy grid (verified against true obstacle positions)

\- \[ ] A\* path planning (hand-implemented)

\- \[ ] Integration + dynamic replanning

\- \[ ] Testing + demo video



\## Status



Phase 1 complete: simulated environment, start/goal markers, grid mapping, WASD control.



Phase 2 in progress: red obstacles are detected from the rendered frame using HSV

thresholding and contours. Detection was verified against the true obstacle list:

all 3 obstacles found at the correct positions. Detected width and height are

1 px larger than the drawn values because OpenCV counts both edge pixels, so the

occupancy grid will use x + w - 1 and y + h - 1 as the last pixel.



Next: convert detections into a 24 x 32 occupancy grid for A\*.

