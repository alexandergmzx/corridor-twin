"""The two ROS domains the demonstration runs in, and the topics allowed between.

These constants exist so the launch files, the demonstration script, and the
documentation quote one set of numbers instead of four. The bridge configuration
in ``config/corridor_domain_bridge.yaml`` restates them, and
``test_gateway_config.py`` fails if the two ever disagree -- the restatement is
the point, not an oversight: the YAML is what the bridge actually reads, so a
test that imported the numbers from here would prove only that Python agrees
with itself.

Neither value is 0. The default domain is what every unconfigured ROS process
joins, so if a node ever falls back to it the mistake is silent -- the two halves
would simply find each other again and every isolation claim in this repository
would be false while all the tests still passed. Non-default numbers make that
failure visible: nothing is on domain 0, so a stray participant there talks to
nobody.
"""

from __future__ import annotations

#: A's domain: the Isaac adapter or the synthetic publisher, plus simulator
#: truth. Since ADR 0021 the camera the adapter publishes is *P's* roadside
#: enforcement instrument rather than a sensor on A, which is camera-less. It
#: originates here only because this is the plane the simulator renders in, and
#: it is relayed straight out to its owner.
ROBOT_DOMAIN_ID = 42

#: P's domain: the camera-only observer, the enforcement display, and RViz.
POLICE_DOMAIN_ID = 43

#: Every topic the gateway is permitted to carry, and the direction it carries
#: it. Stated as a mapping from topic to message type so a reader sees the whole
#: sanctioned surface in one place; the direction is uniform and one-way.
RELAYED_TOPICS: dict[str, str] = {
    "/p_cam/image_raw": "sensor_msgs/msg/Image",
    "/p_cam/camera_info": "sensor_msgs/msg/CameraInfo",
    "/clock": "rosgraph_msgs/msg/Clock",
}
