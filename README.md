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

\- \[ ] Day 7: Verify detected boxes against true obstacles

\- \[ ] Day 8: Occupancy grid from detected obstacles

\- \[ ] A\* path planning (hand-implemented)

\- \[ ] Integration + dynamic replanning

\- \[ ] Testing + demo video



\## Status



Phase 1 complete: simulated environment, start/goal markers, grid mapping, WASD control.



Phase 2 in progress: red obstacles are detected from the rendered frame using HSV

thresholding and contours, with yellow bounding boxes drawn on detections.



Next: verify detection accuracy, then convert detections into an occupancy grid for A\*.

