# RealSense D435i — Terrain Detection and Rover Navigation

## Camera

- **Camera:** [Intel RealSense Depth Camera D435i](https://www.realsenseai.com/products/depth-camera-d435i/)
- **Preferred configuration:** Front and rear cameras.
- **Additional localization:** AprilTags.

## Navigation Goals

- Build a **local elevation grid** to identify holes, hills, and other terrain features.
- Use the grid for **pathfinding and obstacle avoidance**.
- Support **preprogrammed targets**.
- Combine camera-based terrain perception with localization and planning.

## Reference Approaches

- **Purdue (2024):** Front and rear cameras with SLAM feeding a path planner.
- **College of DuPage:** Two RealSense depth cameras plus a Livox Mid-360 feeding Nav2.
- **Utah:** Occupancy grid generated from RealSense data, with D* path planning.

> **Source note:** The material below was moved from the [RealSense D435i task](https://app.notion.com/p/RealSense-D435i-3edc5b61d5c380f5b50bfa317d9c98f5?pvs=21) on October 21, 2026.

## Objectives from Friday Meeting

- Create or find a model for object detection.
- Evaluate RAM usage and feasibility on the **Jetson Orin Nano (8 GB)**.

## October 5 — YOLO Object Detection Testing

**Test:** `realsense_detect.py` runs object detection on the camera feed and measures RAM usage.

**Model:** `yolo26n.pt` (`n` = nano, the smallest variant).

| Test stage | Script RAM | System RAM used |
| --- | ---: | ---: |
| Start | 45 MB | 22,488 MB |
| After importing YOLO | 232 MB | 22,678 MB |
| After loading model | 259 MB | 22,704 MB |
| After starting camera | 268 MB | 22,730 MB |
| After first inference | 434 MB | 22,911 MB |

**Reported RAM usage:** Approximately 460 MB (script measurement after first inference: 434 MB).

**Files referenced:**
- `realsense_detect.py`
- `yolo26n.pt`

## October 15 — Depth-Based Terrain Detection

**Team:** Madison, Jackson, Marty

### Why depth-based detection?

- Lunar models found on [Hugging Face](https://huggingface.co/) were trained on orbital imagery and were not suitable for a rover-mounted camera view.
- General-purpose COCO models such as YOLO26n do not directly recognize rocks and craters as the required terrain classes.
- The team therefore selected **depth-only detection** using the D435i.

### Guidebook requirements (2026, v3)

- At least **3 boulders**, approximately **30–40 cm**.
- At least **3 craters**, up to approximately **40–50 cm wide/deep**.
- Berm target area: **2.2 × 0.9 m**, scored by volume above grade.
- The onboard D435i is not subject to the camera penalty, which applies to arena cameras.
- The D435i IMU has no compass and is allowed.

### Terrain detection script

**File:** `terrain_detect.py`

The script:
1. Finds the ground plane.
2. Builds a **5 cm-resolution height map**.
3. Reports bumps, dips, and hidden areas.
4. Estimates feature **size, height, volume, and position** where observable.

### Simulated test

**Camera setup:** 0.55 m above ground, tilted 30°.

| Feature | True value | Measured result | Notes |
| --- | --- | --- | --- |
| Berm dimensions | 0.90 × 0.40 × 0.15 m | 0.85 × 0.45 × 0.15 m | — |
| Berm volume | 54 L | 47 L | 86% of true volume |
| Boulder height | 0.20 m | 0.20 m | Position correct within 3 cm |
| Boulder footprint | 0.36 m (reported dimension) | 0.25 × 0.15 m | Underestimated because only the front half is visible in one view |
| Crater location | 1.78 m ahead | Hidden area at 1.85 m ahead | Near wall out of view; depth not measured |

### RAM comparison

- **Depth-based terrain detection:** Approximately **130 MB**.
- **YOLO26n object detection:** **434 MB** after first inference.

### Terrain map legend

- **Red — Bumps:** Boulders and berms.
- **Blue — Dips:** Craters.
- **Yellow — Hidden areas:** Ground regions the camera cannot see (occluded or otherwise unobserved).

**Original image reference:** `image.png` (not included in the supplied text).

## Next Steps

1. Test `terrain_detect.py` with the **physical D435i** on representative terrain.
2. Measure actual RAM usage on the **Jetson Orin Nano (8 GB)**.
3. Combine successive depth frames as the rover moves to observe boulders and craters from multiple angles.
4. Develop a local elevation map for obstacle avoidance and path planning.
5. Evaluate front-and-rear camera coverage and AprilTag-assisted localization.
6. Connect terrain information to target-based navigation and the selected path planner.

---

*This document organizes the supplied project notes; the referenced scripts and image were not attached.*
