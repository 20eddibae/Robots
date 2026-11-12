# Team cheatsheet

Robot 3: IP `192.168.0.167` (check the OLED), Wi-Fi `a2r-projects-3`, namespace `jetson3`. Passwords are on Canvas.
Full details: [SETUP.md](SETUP.md) and the course [QUICK_REFERENCE.md](QUICK_REFERENCE.md).

## Laptop

Push code to the robot (laptop on the robot's Wi-Fi):

```bash
cd ~/Desktop/college/class_projects/Robots && ./deploy.sh
```

Push and rebuild (after adding files or changing `setup.py`/`package.xml`):

```bash
cd ~/Desktop/college/class_projects/Robots && ./deploy.sh --build
```

SSH in:

```bash
ssh a2r-lab-jetson@192.168.0.167
```

Run the offline tests (no robot needed):

```bash
cd ~/Desktop/college/class_projects/Robots && .venv/bin/python -m pytest -q tests
```

Python environment (for testing vision/geometry/controller on photos):

```bash
source ~/Desktop/college/class_projects/Robots/.venv/bin/activate
```

## Jetson: every new terminal

```bash
source ~/a2r-student/robot/setup_terminal.sh
```

Should print `robot=jetson3 domain=3`. Never source this in a terminal you build in.

## Jetson: terminals

| Terminal | Runs                     | Close it?                                   |
| -------- | ------------------------ | ------------------------------------------- |
| A        | Base (hardware + safety) | No, keep open all session                   |
| B        | Our navigation code      | Yes after Ctrl+C; open a new one to restart |
| C        | Checks                   | Any time                                    |

**A: base, motors disabled** (RVR on, gamepad switch on X, controls untouched):

```bash
systemctl --user stop rvr-camera.service
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=false
```

**A: base, motors enabled** (only after the stopping checks and with a measured track width):

```bash
ros2 launch a2r_robot robot.launch.py namespace:="$ROBOT_NAMESPACE" motion_enabled:=true track_width_m:=0.2197
```

**B: our code:**

```bash
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

## Calibration record (jetson3)

**Track width: `0.2197` m.** Pass `track_width_m:=0.2197` on every base launch; it isn't saved on the robot, and the default 0 blocks autonomy. With motors disabled, the safety line should show `separation=0.220 m`.

Measured 2026-11-11, one full turn per spin, hardware_session `613e2cdd535842aeaf9d03f540a797c3`:

| Read                 | left_distance_m | right_distance_m |
| -------------------- | --------------- | ---------------- |
| 1 (start)            | 0.0             | 0.0              |
| 2 (after left spin)  | -0.6377         | 0.7084           |
| 3 (after right spin) | 0.1614          | 0.0927           |

Left spin gave 0.2142 m and right spin 0.2252 m (5% apart, within the 10% limit). The robot drifted forward during the spins, so redo it with 2 turns per spin if steering seems off.

**Encoder calibration:** saved on the robot in `/etc/a2r-robot/encoder_calibration.json`, so nothing to pass.

**Camera homography:** still TODO. `homography` in `navigation.yaml` is the temporary rough value.

## Live view in your browser (terminal C)

SSH into terminal C with a tunnel:

```bash
ssh -L 8081:localhost:8081 a2r-lab-jetson@192.168.0.167
```

Then, after `source ~/a2r-student/robot/setup_terminal.sh`:

```bash
python3 ~/a2r-student/robot/src/team_navigation/team_navigation/stream.py
```

Open **`http://localhost:8081/`** in your laptop browser (not the robot IP; macOS may block browsers from the robot LAN unless allowed under System Settings → Privacy & Security → Local Network). Left: camera with the HSV colour of the boxed bottom-centre area. Right: what our code sees (magenta = counted as yellow, green dots = trail, white circle = steering target, top line = status, speed, yaw). Ctrl+C to stop. It only views; it never sends motion.

## Jetson: checks (terminal C, one at a time, Ctrl+C to stop)

```bash
ros2 node list
```

```bash
ros2 topic echo "/$ROBOT_NAMESPACE/hardware/status" --qos-reliability best_effort --once
```

```bash
ros2 topic echo "/$ROBOT_NAMESPACE/navigation/request" --qos-reliability best_effort
```

```bash
ros2 topic echo "/$ROBOT_NAMESPACE/safety/status"
```

```bash
ros2 topic hz "/$ROBOT_NAMESPACE/navigation/debug_image"
```

```bash
ros2 topic hz "/$ROBOT_NAMESPACE/camera/image_raw"
```

```bash
ros2 topic echo "/$ROBOT_NAMESPACE/encoders" --qos-reliability best_effort --once
```

## Gamepad

| Button                       | Does                                                 |
| ---------------------------- | ---------------------------------------------------- |
| RB (hold)                    | Autonomy: robot follows our code                     |
| LB (hold)                    | Manual tank driving with the sticks                  |
| Release, A, or both bumpers  | Stop                                                 |
| Y (stopped, sticks centered) | Speed profile Low 0.15 / Medium 0.60 / High 1.40 m/s |

To go again after a stop: release everything, center the sticks, then hold RB.

## When to rebuild

| Change                                                    | Do                                        |
| --------------------------------------------------------- | ----------------------------------------- |
| Edit an existing `.py`, `navigation.yaml`, or launch file | `./deploy.sh`, restart terminal B         |
| Add a file, change `setup.py` / `package.xml`             | `./deploy.sh --build`, restart terminal B |
| Change a baseline package or `.msg`                       | `./deploy.sh --build`, restart A and B    |

## Tuning (`robot/src/team_navigation/config/navigation.yaml`)

| Parameter                            | Effect                                                                                                                                                                                         |
| ------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `requested_forward_speed_m_s`        | Forward speed                                                                                                                                                                                  |
| `lookahead_m`                        | Zig-zags: increase. Cuts corners: decrease.                                                                                                                                                    |
| `max_yaw_rate_rad_s`                 | Turn rate limit                                                                                                                                                                                |
| `yellow_hsv_low` / `yellow_hsv_high` | Yellow threshold (H 0-179), now `[18, 120, 140]`-`[35, 255, 255]`. Use the live view's box HSV readout on the paper. Misses yellow: lower S/V minimums. Carpet counts as yellow: raise S minimum. Wall counts as yellow: raise V minimum (lab wall V ~96, paper ~205). |
| `path_gap_px` | Merge yellow groups closer than `[sideways, along-path]` px before tracing, now `[9, 25]`. Trail stops at gaps between sheets: raise the 2nd number. Separate paths merge: lower the 1st. `[0, 0]` = off. |
| `homography`                         | Camera calibration, 9 floats. All zeros means zero motion.                                                                                                                                     |
| `max_image_age_s`                    | Ignore older camera frames                                                                                                                                                                     |

## Expected `navigation/request` with a path in view

| Path                                 | `yaw_rate_rad_s` |
| ------------------------------------ | ---------------- |
| Left of center                       | > 0              |
| Right of center                      | < 0              |
| Centered                             | about 0          |
| No path / uncalibrated / stale image | both fields 0    |

## `safety/status` meanings

| Starts with                            | Means                                                  |
| -------------------------------------- | ------------------------------------------------------ |
| `ready for deliberate selection`       | Waiting; hold RB                                       |
| `autonomous`                           | Our requests reach the treads                          |
| `autonomous (speed limited)`           | Request over the speed profile; treads scaled down     |
| `center sticks and release bumpers`    | Let go of everything, then hold RB again               |
| `autonomous conversion not configured` | `track_width_m` not set in the base launch             |
| `navigation request expired`           | Terminal B crashed or stopped; check its log           |
| `hardware or joystick stale`           | Base unhealthy; check terminal A and `hardware/status` |

## Base won't start: troubleshooting

**`BlockingIOError: Resource temporarily unavailable`** (on `fcntl.flock`): another program already owns the RVR connection, usually a base launch left running by you or a teammate. Check with the team before stopping anything.

See what's running:

```bash
ps aux | grep -E "ros2 launch|a2r_robot|team_navigation|rvr_io" | grep -v grep
```

Leave `jetson_oled.py` (the OLED display service) running; it's always there and doesn't block the base.

```bash
tmux ls
```

If it's in tmux, stop it cleanly: `tmux attach -t NAME`, Ctrl+C in each window, then Ctrl+b `d` to leave.

Otherwise stop all ROS launches (same as Ctrl+C, so the base shuts down properly):

```bash
pkill -INT -f "ros2 launch"
```

Wait ~10 s and run the `ps` line again. Leftover processes whose launch is already gone are not caught by that. Stop them by name:

```bash
pkill -f "a2r_robot/lib/a2r_robot"
```

```bash
pkill -f "team_navigation/lib/team_navigation"
```

Or stop one specific process by its PID (2nd column of `ps`). Use `kill`, not `pkill`: `pkill 3830` looks for a process _named_ 3830.

```bash
kill PID
```

Still there after a few seconds: `kill -9 PID`. Then relaunch terminal A.

If our navigation node keeps reappearing after a reboot with no terminal attached, something is auto-starting it; check `systemctl --user list-units --type=service --no-pager` and `crontab -l`.

**`TimeoutError`** (traceback through `probe` / `get_main_application_version`): the RVR isn't answering over the UART.

1. RVR off or asleep (it sleeps when idle): press its power button, check its lights, relaunch.
2. RVR battery low: charge or swap.
3. Something else owns the UART (another base or an `rvr_io` demo): run the `ps` line above.
4. Wiring: power everything off before checking TX/RX and ground.

A good start shows `UART communication OK; motors commanded OFF.` then `ready for deliberate selection`.

## End of session

1. Terminal B: Ctrl+C.
2. Terminal A: Ctrl+C, then:

```bash
systemctl --user start rvr-camera.service
```

3. Close all terminals.

## tmux (one SSH login, several windows)

```bash
tmux new -s robot
```

Ctrl+b then `c` opens a window, Ctrl+b then `0`/`1`/`2` switches. After a dropped connection:

```bash
tmux attach -t robot
```
