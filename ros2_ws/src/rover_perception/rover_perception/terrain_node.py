"""PLACEHOLDER: depth-only terrain hazard node (Phase 3 of docs/ROADMAP.md).

The team's terrain_detect.py (ground plane -> 5 cm height map -> bumps / dips / hidden
areas, about 130 MB RAM on a desktop) is not in this repo yet. Drop it into tools/ and
port it here. Planned design:

  1. Subscribe to emitter-on depth (/camera/camera/depth/image_rect_raw, profile:=terrain)
     and the IMU (/camera/camera/imu) for the gravity vector ("up").
  2. Place points with the odometry pose (odom -> camera TF) into a rolling, robot-centred
     grid in the odom frame: 5 cm cells, about 6 x 6 m (120 x 120). Per cell keep min/max
     height, hit count, last-seen time.
  3. Classify: bump (above max step), hole (below max step, or an unseen shadow past an
     observed edge), slope (3x3 plane fit steeper than the rover can climb), unknown.
     Trust depth to about 2.5 m; mark farther cells provisional. Min depth is about 0.3 m.
  4. Publish nav_msgs/OccupancyGrid (and later a custom Nav2 costmap layer): lethal for
     bump/hole/steep, scaled for moderate slope, unknown left unknown.

Open inputs from the team: rover max step height and max slope.
"""


def main():
    raise SystemExit("terrain_node is a placeholder; see the docstring in rover_perception/terrain_node.py")
