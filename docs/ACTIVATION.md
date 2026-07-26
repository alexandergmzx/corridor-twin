# Hardware and Isaac activation record

| Field | Value |
|---|---|
| Record date | 2026-07-26 |
| Host | Linux Mint 22.3 Zena, kernel 6.8.0-136-generic |
| CPU / RAM | Ryzen 9 5950X, 16 cores / 32 threads, 48 GB |
| Current GPU | GeForce RTX 5060, 8151 MiB |
| Driver / API | 580.173.02 / Vulkan |
| Installed simulator | Isaac Sim 5.1.0, Python 3.11 |
| Installed Isaac Lab | 2.3.2 |
| Planned demo GPU | GeForce RTX 5070 Ti, 16 GB |

## Current evidence

- The installed compatibility checker recognizes the RTX 5060 and driver but
  returns `FAILED`: it reports 8 GB as below its internal 10 GB threshold and
  identifies Linux Mint as unsupported.
- The published Isaac Sim 5.1 requirements are stricter: Ubuntu 22.04/24.04,
  GeForce RTX 4080, and 16 GB VRAM are the x86_64 minimum. The planned 5070 Ti
  meets the published VRAM floor, but Mint remains outside the supported OS list.
- The generated corridor stage passes `tools/isaac_5_1_smoke.py` on the 5060
  using Vulkan and real-time `RaytracedLighting`. The smoke sees one authored
  robot camera, all three corridor variants, and both building colliders.
- Isaac Sim 5.1 documentation is now marked unsupported upstream. Keep the
  installed version pinned for the interview demo; schedule an upgrade as a
  separate, tested change rather than mixing it with the GPU swap.
- IOMMU is enabled and reported as a warning. It did not prevent the small stage
  from composing, but it should remain in the recorded risk list.

## After the 5070 Ti is installed

1. Confirm the card, driver, and VRAM with `nvidia-smi`.
2. Run the installed compatibility checker and save the complete result.
3. Rebuild the USDA and occlusion certificate from a clean shell.
4. Repeat the Isaac smoke command from the README.
5. Open one GUI viewport with real-time rendering; do not enable path tracing.
6. Record idle, loaded-stage, and one-camera steady-state VRAM. Stop if the soft
   ceiling of 14 GB is exceeded.
7. Repeat the ROS synthetic demo before adding an Isaac camera graph.

Do not move `~/isaac`: the environment contains editable/path-sensitive installs.
The repo consumes it through an explicit command and keeps the ROS/OpenUSD Python
3.12 environment separate from Isaac's Python 3.11 environment.

## Official references used

- [Isaac Sim 5.1 requirements](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html)
- [Isaac Sim 5.1 ROS 2 installation and bridge](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/install_ros.html)
- [Isaac Sim 5.1 ROS camera tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera.html)
- [Isaac Sim 5.1 ROS clock tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.html)
- [OpenUSD `UsdVariantSet` API](https://openusd.org/release/api/class_usd_variant_set.html)
- [OpenUSD physics schema](https://openusd.org/release/api/usd_physics_page_front.html)
- [ROS 2 Jazzy sensor-data QoS](https://docs.ros.org/en/ros2_packages/jazzy/api/rclcpp/generated/classrclcpp_1_1SensorDataQoS.html)
- [OpenCV ArUco detection and pose estimation](https://docs.opencv.org/4.7.0/d5/dae/tutorial_aruco_detection.html)
