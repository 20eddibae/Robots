# COSC 81/281 robot project

Start with [Setup and testing](docs/SETUP.md). This repository supplies the ROS 2 baseline students build, modify and extend, together with a starter navigation program and a PC viewer.

## Two computers, two workspaces

You will work on two computers. Each one gets its own copy of this repository, but builds and runs a different folder inside it:

| Computer | What it is | Folder it builds and runs | What that folder does |
| --- | --- | --- | --- |
| **The Jetson** | The small computer mounted on the robot. You log into it over SSH. | `robot/` | Runs the robot: sensors, motors, safety checks and your navigation code. |
| **Your PC** | Your own laptop or lab computer. | `viewer/` | Shows the robot's live camera and telemetry in RViz. It does not drive the robot. |

Each of those two folders is a ROS workspace: a folder with a `src/` directory that you build with colcon. You build `robot/` only on the Jetson and `viewer/` only on your PC.

```text
a2r-student/                    # Clone of this repository
├── README.md
├── robot/                      # Build and run on the Jetson
│   ├── src/
│   │   ├── a2r_robot/          # Hardware access, motor control and stopping rules
│   │   ├── a2r_robot_interfaces/
│   │   ├── course_robot_interfaces/
│   │   └── team_navigation/    # Student starting point
│   ├── runtime/               # Source for installed camera/OLED/device helpers
│   └── README.md
├── viewer/                     # Build and run on your PC, not the Jetson
│   ├── src/
│   ├── config/
│   └── README.md
├── docs/                       # Setup guide and quick reference
├── tests/                      # Software checks you can run without a robot
└── LICENSE                     # MIT
```

Both computers clone the same repository, so the Jetson will also have a `viewer/` folder and your PC will also have a `robot/` folder. Leave the other computer's folder unbuilt. Never run colcon from the repository root: `robot/` and `viewer/` each contain their own copy of the interface packages, and building them together fails.

When built, each workspace creates its own `build/`, `install/` and `log/` folders inside itself. Git ignores these generated folders. There is no separate `~/ros2_ws` and no source-copy step.

The viewer is optional: only one person per team needs to be able to build it. Everyone else can work entirely over SSH, saving or recording data on the Jetson and copying files to their PC with `scp`. RViz is worth having on one machine because seeing the camera and telemetry live makes debugging much easier.

## Get the repository

In a terminal on each machine:

```bash
cd ~
git clone https://github.com/plancherb1/cosc-281-f26-project.git a2r-student
```

The repository is public, so no GitHub sign-in is needed. If the Jetson has no internet access, the [setup guide](docs/SETUP.md#2-clone-the-repository-on-the-jetson) shows how to copy your PC clone to it instead. If `~/a2r-student` already exists, inspect it before cloning or updating; keep your own edits.

## Get Started with docs/SETUP.md

1. Follow [wiring and connection checks](docs/SETUP.md#hardware-connections-and-pinout).
2. On the **Jetson**, follow [the robot build](docs/SETUP.md#build-the-robot-workspace-on-the-jetson). This builds the baseline, both message packages and student program together in `robot/`.
3. Follow [the disabled-first launch](docs/SETUP.md#first-launch-motors-disabled), then [sensor and communication checks](docs/SETUP.md#run-the-student-program).
4. On **one team member's PC**, follow [viewer setup](viewer/README.md) and verify live robot data over the assigned router. This step is optional for everyone else.
5. Complete [controlled movement/stopping](docs/SETUP.md#controlled-movement-and-stopping-check) and [calibration](docs/SETUP.md#encoder-calibration-and-geometry).
6. If you have time, try [making your first autonomous node](docs/SETUP.md#getting-started-with-autonomy).
7. Finish with [stop, restore and record](docs/SETUP.md#stop-restore-and-record) and the [end of session shutdown](docs/SETUP.md#end-of-session-shutdown).

The lab Jetsons already have their commissioned OS, firmware, ROS Jazzy, `~/ros-venv`, identity/network settings and standalone camera/OLED services. [Prerequisite checks](docs/SETUP.md#check-the-commissioned-prerequisites) verify those before building. You build the ROS baseline yourself. If a prerequisite check fails, ask the teaching staff; it is not something to fix by rebuilding your code.

## Team navigation (submitted code)

Submitted version: `main` of this repository. The team package is `robot/src/team_navigation`. Parameters live in `config/navigation.yaml`. Detail and the file list: [robot/src/team_navigation/README.md](robot/src/team_navigation/README.md). Day-to-day commands: [docs/CHEATSHEET.md](docs/CHEATSHEET.md).

`navigation` is the only team node. Every 0.05 s it masks the camera for yellow, traces a path, maps pixels to the floor with the homography in `geometry.py`, and publishes `navigation/request` (forward m/s, left-turn rad/s). The supplied safety node is the only motor publisher. Hold RB to drive; A or releasing the bumper stops. Images older than 0.3 s, a lost path, or an all-zero homography request zero motion. The IMU is not used.

Default in `navigation.yaml`: `perception: yellow` (yellow-only trace in `vision_yellow.py`) and `steering: taut` (aim along the longest straight line that stays inside the road; otherwise pure pursuit at `lookahead_m`). `perception:=blue_aware` uses `vision.py` to stay off blue paper and other cars and to keep the same side of a fork. `footprint_check:=true` (blue-aware only) stops if the treads cannot fit. No extra Python packages beyond the Jetson venv (OpenCV, NumPy, ROS 2 Jazzy).

Build and run on the Jetson, after [setup](docs/SETUP.md). In a fresh terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/robot/install/setup.bash
source ~/ros-venv/bin/activate
source /etc/a2r-robot/robot.sh
cd ~/a2r-student/robot
python -m colcon build --symlink-install --packages-select team_navigation
```

Terminal A (RVR on; motors stay off until you set `motion_enabled:=true` and hold RB):

```bash
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=false
```

Terminal B:

```bash
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

One-run overrides (blank means the yaml value): `perception:=blue_aware`, `steering:=center`, `footprint_check:=true`, `speed:=0.25`, `max_yaw:=3.0`, `explore:=false`.

Representative check, motion still disabled: the node logs the mode and speeds, `ros2 topic echo /$ROBOT_NAMESPACE/navigation/request` shows finite speeds or zeros, and `navigation/debug_image` shows magenta on yellow and a green trail. With a saved photo and no ROS, from `robot/src`:

```bash
python3 -c "import cv2; from team_navigation import vision_yellow as vy; m=vy.yellow_mask(cv2.imread('photo.jpg'),(18,60,140),(35,255,255)); print(len(vy.corridor(m, jump_px=80)))"
```

Limitations: the homography in the yaml is a rough placeholder, so metre lookahead and turn rate are approximate. Yellow mode does not treat blue as forbidden and, while both fork branches are one region, aims between them. `robot_half_width_px` is an estimate. There is no lap memory. The course tests of the unmodified zero-motion starter fail once this node drives; that is expected.

## Connection details

| Connection | Name / username | Password |
| --- | --- | --- |
| 2.4 GHz Wi-Fi | Robot 1: `a2r-projects`; robots 2–5: `a2r-projects-N` | See Canvas |
| 5 GHz Wi-Fi | Assigned SSID with `_5G` appended | See Canvas |
| SSH, all robots | `a2r-lab-jetson` | See Canvas |

Replace `N` with the robot number. Use the IP on the robot's OLED:

```bash
ssh a2r-lab-jetson@ROBOT_IP
```

The passwords are posted on Canvas, not in this public repository. Keep them, and your personal GitHub keys, out of commits and off shared robots.

## References

- [Robot baseline and student program](robot/README.md)
- [PC viewer](viewer/README.md)
- [Quick reference](docs/QUICK_REFERENCE.md): the everyday commands on one page, for after you have completed setup once.
- [Software tests](robot/README.md#run-the-software-tests)

This repository is maintained by the [A2R Lab](https://a2r-lab.org) and released under the [MIT license](LICENSE). Retain the [course attribution](robot/NOTICE.md) when you copy or modify the starter. Automatic navigation remains student work; the supplied starter requests zero movement.

## AI Disclosure
We used LLM tools, including Codex and Claude Code, to assist with software implementation, drafting, proofreading, and review. All final text, figures, code, and references were edited, revised, and reviewed by humans.
