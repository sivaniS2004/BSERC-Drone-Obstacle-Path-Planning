\# AI-Based Obstacle Detection and Path Planning for a Simulated Autonomous Drone



BSERC Defence-Space Tech Internship Project.



\## Tech Stack

Python + OpenCV (single technology, per project scope)



\## Progress

\- \[x] Day 1: Simulated environment - canvas, obstacle, drone, goal rendering

\- \[x] Day 2: Drone movement via keyboard controls (W/A/S/D)

\- \[x] Day 3-4: Start/goal markers, grid mapping, WASD control with collision

\- \[x] Day 6: Obstacle detection from pixels (HSV mask + cv2.findContours)

\- \[ ] Occupancy grid from detected obstacles

\- \[ ] A\* path planning (hand-implemented)

\- \[ ] Integration + dynamic replanning

\- \[ ] Testing + demo video



\## Status



Phase 1 complete: simulated environment, start/goal markers, grid mapping, WASD control.



Phase 2 in progress: red obstacles are detected from the rendered frame using HSV

thresholding and contours, with yellow bounding boxes drawn on detections.



Next: convert detections into an occupancy grid for A\*.

