# RGB-D Grasp Estimator

English · [日本語](docs/README.ja.md)

Estimate object clusters, centroids, normals and candidate grasp poses from a
detected region and aligned depth. PyLoT's bottle/bowl cluster-grasp pipelines
are generalized to remove dependencies on an object detector or Luna arm.
Object detection is external: supply a bounding box, or a mask through the
Python API.

## Geometry pipeline

1. Project valid depth within the box/mask into 3D using camera intrinsics.
2. Cluster points by distance using a spatial grid and BFS.
3. Select the largest supported cluster and use the covariance matrix's smallest-eigenvalue vector as a candidate normal.
4. Orient the normal toward the camera, offset the centroid and return an approach pose.

Subsampling is deterministic. Configure the point limit, cluster distance,
minimum cluster size and planarity threshold. Insufficient points, invalid depth,
collinear degeneracy or insufficient planarity produce no candidate. The largest
cluster is not guaranteed to be the target object: a region/mask that excludes
the background matters.

## Offline use

```bash
rgbd-grasp-estimator estimate depth.npy --unit mm --roi 10 10 100 100 \
  --intrinsics 600 600 320 240 --output candidate.json
```

Depth is a 2D `.npy` array. Bounding boxes are `[x1,y1,x2,y2]`, excluding the
right and bottom boundaries. Specify `m` or `mm` explicitly. fx/fy/cx/cy are in
pixels. Output positions are in metres in the camera optical frame; quaternions
use xyzw order. Local +Z points along the approach direction, opposite the normal.
Python's `rgbd_grasp.core.estimate` also accepts an image-sized boolean mask.
CLI exit codes are 0 for a candidate, 1 for no candidate and 2 for invalid input.

## ROS interface

Publish aligned depth and CameraInfo, then send String JSON on `/rgbd_grasp/roi`:

```json
{"bbox":[10,10,100,100],"stamp":123.4,"frame_id":"camera_color_optical_frame"}
```

The timestamp is the corresponding image's ROS time. Depth, ROI and CameraInfo
must share the optical frame and pixel coordinates. The adapter checks image
size against CameraInfo, depth freshness and ROI/depth timestamp separation.
It uses supplied camera intrinsics rather than an assumed field of view.
16UC1 is interpreted as millimetres; 32FC1 as metres.

- `/rgbd_grasp/pose` (`PoseStamped`): published only when a candidate exists; preserves the depth header.
- `/rgbd_grasp/result` (`String` JSON): validity, position, normal, orientation and point count, or a failure reason.

Invalid, stale or unsynchronized input does not republish an old pose. Input
loss does not erase messages already received on the pose topic: consumers
must also check `header.stamp`.

Synthetic-plane tests cover units, normals, orientation and subsampling.
ROS integration tests cover depth-to-pose output and rejection of unsynchronized
input. Grasp success, collision avoidance, reachability and real-arm operation
are not evaluated. This package does not command an arm.

## Installation and checks

Requires Python 3.10+. Run these commands from the repository root.
The core works in a regular Python environment without ROS.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[test]'
python -m pytest -q
python -m rgbd_grasp.cli demo
```

For ROS 2 Jazzy, place the repository in a regular workspace's `src` directory
and build it in a ROS environment:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select rgbd_grasp
source install/setup.bash
```

Adapt `config/params.yaml` to your inputs and outputs. Parameters are read at startup.
Start the node with
`ros2 launch rgbd_grasp rgbd_grasp.launch.py params_file:=/absolute/path/to/params.yaml`.
CI runs core tests and the synthetic demo on Python 3.10 and 3.12. ROS integration
tests run when ROS is available and are skipped otherwise. These checks do not
certify model, physical-sensor or real-arm performance.

## License and source

This repository is currently private. Apache-2.0. See
[PROVENANCE.md](PROVENANCE.md) for the source commit, files and extraction changes.
There is no dependency on the original monorepo.
