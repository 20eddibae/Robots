# team_navigation: yellow road following

Our ROS 2 navigation node for the Sphero RVR + Jetson. It follows the yellow
paper road, treats blue paper as out of bounds, and drives through the
course safety node (hold RB on the gamepad).

Quick commands and troubleshooting: [docs/CHEATSHEET.md](../../../docs/CHEATSHEET.md).

## How it works

Every 0.05 s, `navigation.py` runs:

1. **Masks** the camera image (scaled to 320x180) for yellow, and blue/cars.
2. **Traces a path** in pixels, near to far (see *Modes* below).
3. **Converts to floor metres** with the camera homography (`geometry.py`).
4. **Steers** with pure pursuit (`controller.py`): drive the arc through a
   target point, `yaw = v * 2y / d^2`.
5. **Publishes** `navigation/request` (forward m/s, yaw rad/s) and a debug
   image for the browser viewer (`stream.py`).

It requests zero motion if the image is stale, the path is lost, or the
homography is all zeros.

## Modes

Chosen in `config/navigation.yaml`, or per run with launch arguments.

| Setting | Options |
| --- | --- |
| `perception` | `yellow` (default): yellow-only tracing (`vision_yellow.py`). Fast and simple; ignores blue for steering.<br>`blue_aware`: path search that keeps off blue and other cars and maps both ways round a split (`vision.py`). |
| `steering` | `taut` (default): aim at the farthest point reachable in a straight line inside the road, so it doesn't follow every wiggle.<br>`center`: aim at the path point `lookahead_m` ahead. |
| `footprint_check` | `blue_aware` only. `true`: keep both treads (not just the centre) off blue and stop with `BLOCKED (treads)` if they can't fit. Off by default. |

Also:

- **Gaps between sheets** (carpet, black tape) are crossed when yellow
  continues in line beyond them: `path_jump_px` (yellow mode, default 80) and
  `max_gap_px` (blue-aware, default 180). Both are pixels at the bottom of the
  image and shrink with distance.
- **Explore**: if there's no usable path and blue is in view, the robot turns
  right on the spot until it finds yellow again, for at most 15 s
  (`explore_when_blocked`). The viewer shows `EXPLORING`.

## Speed

The gamepad's Y button picks the profile; our code reads it from
`safety/status`.

| Profile | `profile_speeds_m_s` | Actually drives at |
| --- | --- | --- |
| Low | 0.15 | 0.15 m/s |
| Medium | 0.75 | 0.60 m/s (safety node caps Medium treads at 0.60) |
| High | 0.8 | 0.8 m/s |

In turns the outer tread goes faster, and the safety node slows both treads
together to stay within the profile, keeping the turn's shape.

## Running

Laptop on the robot's Wi-Fi. Copy the code to the robot (use `--build` after
adding files or changing the launch file):

```bash
./deploy.sh --build
```

Three robot terminals (`ssh a2r-lab-jetson@<robot IP>`, then
`source ~/a2r-student/robot/setup_terminal.sh` in each):

**A, base** (RVR awake; keep running):

```bash
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=true track_width_m:=0.2197
```

**B, navigation:**

```bash
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

Optional launch arguments (left out = `navigation.yaml`):

| Argument | Example |
| --- | --- |
| `perception` | `perception:=blue_aware` |
| `steering` | `steering:=center` |
| `footprint_check` | `footprint_check:=true` |
| `speed` | `speed:=0.3` (overall cap for this run) |
| `max_yaw` | `max_yaw:=4.0` (turn-rate cap, default 3.0) |
| `yellow_gap` / `blue_aware_gap` | `yellow_gap:=120` |
| `explore` | `explore:=false` |

The first log line shows the active mode and speeds.

**C, viewer** (log in with `ssh -L 8081:localhost:8081 ...`):

```bash
python3 ~/a2r-student/robot/src/team_navigation/team_navigation/stream.py
```

Open http://localhost:8081/. Left: camera, HSV of the bottom-centre box,
speed profile and tread speeds. Right: magenta = yellow, red = blue,
orange = car, green dots = path, white circle = target, top lines = status
and mode.

Hold **RB** to drive; release to stop.

## Files

| File | Role |
| --- | --- |
| `team_navigation/navigation.py` | ROS node: modes, speed, explore, debug image |
| `team_navigation/vision_yellow.py` | `yellow` mode: mask, gap bridging, corridor, taut point |
| `team_navigation/vision.py` | `blue_aware` mode: colour masks, path search, footprint, taut point |
| `team_navigation/geometry.py` | Pixel <-> floor homography |
| `team_navigation/controller.py` | Pure pursuit |
| `team_navigation/stream.py` | Browser viewer (port 8081) |
| `config/navigation.yaml` | All tuning parameters, with comments |
| `launch/team.launch.py` | Launch file and per-run overrides |

## Known limitations

- **Homography is a rough placeholder** (about 4x too large in scale), so
  distances and turn strength are approximate. Calibrate with floor marks.
- `robot_half_width_px` (tread check) is an estimate from holding arms
  along the treads.
- Yellow mode does not avoid blue; it can jump a gap that has blue in it.
- Pale yellow vs. beige/tan objects: if those light up magenta, raise the
  S minimum in `yellow_hsv_low` (now 60).

## Offline testing

The vision, geometry and controller modules have no ROS imports. With the
laptop venv (`.venv`), they can be run on saved photos, e.g.:

```python
import cv2
from team_navigation import vision_yellow as vy
mask = vy.yellow_mask(cv2.imread('photo.jpg'), (18, 60, 140), (35, 255, 255))
print(vy.corridor(mask, jump_px=80))
```
