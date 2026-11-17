"""Follow a yellow floor path: camera -> mask -> trail -> floor -> pure pursuit.

Switchable with parameters (navigation.yaml, or launch arguments):
  perception       'yellow': best-path's yellow-only tracing (vision_yellow);
                   'blue_aware': yellow road, blue and other cars no-go (vision)
  steering         'taut' or 'center'
  footprint_check  blue_aware only: keep both treads off blue/cars and stop
                   (BLOCKED) when they cannot get past just ahead

Pipeline (perception: blue_aware) (each step is a ROS-free module that can be tested offline):
  process_image   vision.color_masks + vision.trace_paths -> trail in pixels
                  (yellow = road, blue and other cars = no-go; maps both ways
                  round a split and keeps following the one chosen before)
  update_estimate geometry.pixel_to_floor                  -> trail in metres
  choose_action   follow if enough trail is visible, otherwise stop
  compute_motion  'taut': controller.pursue toward the farthest trail point
                  reachable in a straight line (vision.straight_reach);
                  'center' (or taut point too near): controller.pure_pursuit
                                                           -> (forward, yaw)

Requests zero motion whenever the image is stale, the path is lost, or the
homography has not been calibrated. Motion still passes through the supplied
safety node (RB held, measured track_width_m).
"""
import math
import re
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String
from cv_bridge import CvBridge
from course_robot_interfaces.msg import MotionRequest
from team_navigation.vision import (color_masks, obstacle_mask, trace_paths, choose_path,
                                    straight_reach, body_px, drop_specks)
from team_navigation import vision_yellow
from team_navigation.geometry import pixel_to_floor, homography_from_list
from team_navigation.controller import pure_pursuit, pursue


class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')
        param = lambda name, default: self.declare_parameter(name, default).value
        # Forward speed: the controller's profile limit (Low/Medium/High, read
        # from safety/status), but never more than this.
        # Speed for each gamepad profile (Low, Medium, High), never above the
        # profile's own tread limit, nor above requested_forward_speed_m_s.
        self.profile_speeds = dict(zip(('Low', 'Medium', 'High'),
                                       param('profile_speeds_m_s', [.15, .75, .8])))
        self.target_speed = param('requested_forward_speed_m_s', 1.4)
        self.tread_limit = None  # from safety/status; None until heard
        self.profile = None  # 'Low' / 'Medium' / 'High' from safety/status
        # Explore: when no usable path is found and blue is in the way,
        # turn slowly clockwise on the spot until a path appears.
        self.explore = bool(param('explore_when_blocked', True))
        self.explore_yaw = param('explore_yaw_rate_rad_s', .5)
        self.explore_min_blue = param('explore_min_blue_frac', .02)
        self.explore_max_s = param('explore_max_s', 15.)
        self.explore_started = None  # time.monotonic() when exploring began
        self.exploring = False
        self.lookahead_m = param('lookahead_m', .30)
        self.max_yaw = param('max_yaw_rate_rad_s', 1.0)
        self.min_points = param('min_trail_points', 3)
        self.max_image_age = param('max_image_age_s', .3)
        self.image_scale = param('image_scale', .25)
        self.yellow_low = tuple(param('yellow_hsv_low', [18, 60, 140]))
        self.yellow_high = tuple(param('yellow_hsv_high', [35, 255, 255]))
        self.blue_low = tuple(param('blue_hsv_low', [95, 60, 40]))
        self.blue_high = tuple(param('blue_hsv_high', [130, 255, 255]))
        self.car_low = tuple(param('car_hsv_low', [0, 0, 150]))
        self.car_high = tuple(param('car_hsv_high', [179, 80, 255]))
        self.perception = param('perception', 'yellow')  # 'yellow' or 'blue_aware'
        if self.perception not in ('yellow', 'blue_aware'):
            raise ValueError(f"perception must be 'yellow' or 'blue_aware', not {self.perception!r}")
        self.footprint_check = bool(param('footprint_check', False))
        # perception: yellow (best-path) settings
        self.path_gap_px = tuple(param('path_gap_px', [9, 25]))
        self.path_jump_px = param('path_jump_px', 80)
        self.edge_margin_frac = param('edge_margin_frac', .15)
        # perception: blue_aware settings
        self.horizon_frac = param('horizon_frac', .28)
        self.blue_min_area_px = param('blue_min_area_px', 150.)
        self.path_settings = dict(
            horizon_frac=self.horizon_frac,
            max_gap_px=param('max_gap_px', 180.),
            blue_margin_px=param('blue_margin_px', 6.),
            obstacle_margin_px=param('car_margin_px', 15.),
            blue_hug_gain=param('blue_hug_gain', .3),
            turn_cost=param('turn_cost', .15),
            robot_half_width_px=(param('robot_half_width_px', 150.)
                                 if self.footprint_check else 0.),
            footprint_skip_frac=param('footprint_skip_frac', .1))
        self.steering = param('steering', 'taut')  # 'taut' or 'center'
        if self.steering not in ('taut', 'center'):
            raise ValueError(f"steering must be 'taut' or 'center', not {self.steering!r}")
        self.taut_min_support = param('taut_min_support', .75)
        # Rows (at image_scale) the path must reach beyond the footprint
        # region before the robot moves; fewer means blocked -> stop.
        self.min_clear_rows = param('footprint_min_clear_rows', 10)
        H = param('homography', [0.] * 9)  # all zeros means "not calibrated"
        self.H = homography_from_list(H) if any(H) else None
        publish_debug = param('publish_debug_image', True)

        self.bridge = CvBridge()
        self.image = self.image_stamp = self.imu = None
        self.mask = self.blue = self.cars = self.trail_px = self.target = None
        self.paths = []  # every path seen this frame; trail_px is the one followed
        self.taut_px = None  # straight-line aim point for 'taut' steering
        self.used_taut = False
        self.blocked = False  # the footprint cannot get past the rows right ahead
        self.previous_error = None
        self.create_subscription(Image, 'camera/image_raw', self.image_cb, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', self.imu_cb, qos_profile_sensor_data)
        self.create_subscription(String, 'safety/status', self.status_cb, 1)
        self.publisher = self.create_publisher(
            MotionRequest, 'navigation/request',
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.debug_pub = (self.create_publisher(Image, 'navigation/debug_image', 1)
                          if publish_debug else None)
        self.create_timer(.05, self.tick)
        self.get_logger().info(f'mode: {self.mode_text()}, speed: controller '
                               f'Low/Medium/High {self.profile_speeds} m/s, '
                               f'at most {self.target_speed} m/s')
        if self.H is None:
            self.get_logger().warning(
                'homography not calibrated: requesting zero motion '
                '(the debug image still shows the mask and trail)')

    def image_cb(self, msg):
        try:
            # OpenCV uses BGR channel order. The image is an H x W x 3 array.
            self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_stamp = msg.header.stamp
        except Exception as exc:
            self.image = self.image_stamp = None
            self.get_logger().error(f'Image conversion failed: {exc}')

    def imu_cb(self, msg):
        self.imu = msg

    def status_cb(self, msg):
        """Remember the speed profile and its tread limit, from e.g.
        'speed_profile=Medium; tread_limit=0.60 m/s'."""
        found = re.search(r'tread_limit=([0-9.]+)', msg.data)
        if found:
            self.tread_limit = float(found.group(1))
        found = re.search(r'speed_profile=(\w+)', msg.data)
        if found:
            self.profile = found.group(1)

    def speed(self):
        """Forward speed to request: profile_speeds_m_s for the gamepad's
        profile, but no more than the profile's tread limit (the safety node
        would cap it there anyway) or requested_forward_speed_m_s. Low (0.15)
        until the safety node is heard. In a turn the outer tread goes faster
        than this and the safety node slows both treads together, keeping
        the turn's shape."""
        limit = self.tread_limit if self.tread_limit is not None else .15
        wanted = self.profile_speeds.get(self.profile, limit)
        return min(wanted, limit, self.target_speed)

    def image_is_fresh(self):
        """True if the latest image is no older than max_image_age_s."""
        if self.image is None or self.image_stamp is None:
            return False
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = self.image_stamp.sec * 10**9 + self.image_stamp.nanosec
        age = (now_ns - stamp_ns) / 1e9
        return -.05 <= age <= self.max_image_age

    def mode_text(self):
        """Short description of the active settings, e.g. for the viewer."""
        treads = ('treads on' if self.footprint_check else 'treads off'
                  ) if self.perception == 'blue_aware' else ''
        return ' '.join(p for p in (self.perception, self.steering, treads) if p)

    def process_image(self, image):
        """Return the path centerline as [(u, v), ...] mask pixels, or None.

        None means no usable image (missing or older than max_image_age_s).
        An empty list means the image is fine but no path was found.
        """
        previous = self.trail_px
        self.mask = self.blue = self.cars = self.trail_px = self.taut_px = None
        self.paths = []
        self.blocked = False
        if image is None or not self.image_is_fresh():
            return None
        if self.perception == 'yellow':
            return self.process_yellow(image)
        self.mask, self.blue, car_coloured = color_masks(
            image, self.yellow_low, self.yellow_high, self.blue_low, self.blue_high,
            self.car_low, self.car_high, self.image_scale)
        self.blue = drop_specks(self.blue, self.horizon_frac, self.blue_min_area_px)
        self.cars = obstacle_mask(car_coloured, self.horizon_frac)
        self.paths = trace_paths(self.mask, self.blue, self.cars, **self.path_settings)
        # At a split, stay with the path followed on the last frame.
        self.trail_px = choose_path(self.paths, previous)
        # Blocked: the path stops within a few rows of where the robot's
        # footprint starts being checked, i.e. the treads cannot get past
        # blue/cars just ahead. Stop rather than drive the short stub.
        h = self.mask.shape[0]
        footprint_row = h * (1 - self.path_settings['footprint_skip_frac'])
        self.blocked = bool(self.path_settings['robot_half_width_px'] > 0 and self.trail_px
                            and self.trail_px[-1][1] > footprint_row - self.min_clear_rows)
        if self.steering == 'taut' and self.trail_px and not self.blocked:
            found = straight_reach(self.trail_px, self.mask, self.blue, self.cars,
                                   self.taut_min_support, **self.path_settings)
            self.taut_px = found[1] if found else None
        return self.trail_px

    def process_yellow(self, image):
        """perception: yellow -- exactly the best-path branch's tracing."""
        self.mask = vision_yellow.yellow_mask(image, self.yellow_low, self.yellow_high,
                                              self.image_scale)
        # Blue is not avoided in this mode, only detected for exploring.
        _, blue, _ = color_masks(image, self.yellow_low, self.yellow_high, self.blue_low,
                                 self.blue_high, self.car_low, self.car_high, self.image_scale)
        self.blue = drop_specks(blue, self.horizon_frac, self.blue_min_area_px)
        self.cars = np.zeros_like(self.mask)
        rows = vision_yellow.corridor(self.mask, gap_px=self.path_gap_px,
                                      jump_px=self.path_jump_px,
                                      horizon_frac=self.horizon_frac)
        self.trail_px = [(c, v) for c, v, _, _ in rows]
        self.paths = [self.trail_px] if self.trail_px else []
        if self.steering == 'taut' and rows:
            h, w = self.mask.shape
            found = vision_yellow.farthest_visible(rows, (w / 2, h), self.edge_margin_frac)
            self.taut_px = found[1] if found else None
        return self.trail_px

    def update_estimate(self, observation, imu):
        """Return (trail, taut) in floor metres (x fwd, y left), or None.

        trail: N x 2 array of the followed path's points. taut: (x, y) of the
        farthest path point reachable in a straight line, or None.
        Road-relative only: positions relative to the robot right now, not a
        global position. The IMU is not used yet.
        """
        if not observation or self.H is None:
            return None
        taut = None
        if self.taut_px is not None:
            taut = tuple(pixel_to_floor([self.taut_px], self.H)[0])
        return pixel_to_floor(observation, self.H), taut

    def choose_action(self, estimate):
        """Follow the trail if enough of it is visible; None means stop.

        Also stops when the robot's footprint is blocked just ahead.
        """
        if estimate is None or self.blocked or len(estimate[0]) < self.min_points:
            return None
        return estimate

    def compute_motion(self, action):
        """Return (forward m/s, left-turn rad/s); no action means zero motion.

        'taut': head for the farthest path point reachable in a straight line
        on the road and off no-go, so bends in the path are not followed
        one by one and turns use the road's width. Falls back to 'center'
        when that point is nearer than lookahead_m or not found.
        'center': pure pursuit toward the path point lookahead_m ahead.
        The safety node converts this to treads using the measured track
        width and scales both treads together if either exceeds the profile.
        """
        self.target, self.used_taut = None, False
        if action is None:
            return 0., 0.
        trail, taut = action
        if taut is not None and math.hypot(*taut) >= self.lookahead_m:
            forward, yaw, self.target = pursue(taut, self.speed(), self.max_yaw)
            self.used_taut = self.target is not None
            if self.used_taut:
                return forward, yaw
        forward, yaw, self.target = pure_pursuit(
            trail, self.speed(), self.lookahead_m, self.max_yaw, self.min_points)
        return forward, yaw

    def blue_ahead(self):
        """True if at least explore_min_blue_frac of the floor below the
        horizon is blue."""
        if self.blue is None:
            return False
        h = self.blue.shape[0]
        floor = self.blue[int(h * self.horizon_frac):]
        return floor.size > 0 and floor.mean() >= self.explore_min_blue

    def explore_motion(self, forward, yaw):
        """Turn slowly clockwise on the spot while stuck because of blue.

        Starts when there is a fresh, calibrated image but no usable path
        (NO PATH / PATH TOO SHORT / BLOCKED) and blue is in view. Keeps
        turning, even once the blue has turned out of view, until a path is
        followed again, for at most explore_max_s (then stops until a path
        is found). Otherwise returns (forward, yaw) unchanged.
        """
        stuck = (self.explore and self.target is None and self.mask is not None
                 and self.H is not None)
        if not stuck:
            self.exploring, self.explore_started = False, None
            return forward, yaw
        if not self.exploring and self.blue_ahead():
            self.exploring = True
            if self.explore_started is None:
                self.explore_started = time.monotonic()
        if self.exploring and time.monotonic() - self.explore_started > self.explore_max_s:
            self.exploring = False  # gave up; explore_started stays set until a path is found
        if self.exploring:
            return 0., -abs(self.explore_yaw)  # negative yaw = clockwise (right)
        return forward, yaw

    def tick(self):
        forward, yaw = 0., 0.
        try:
            observation = self.process_image(self.image)
            estimate = self.update_estimate(observation, self.imu)
            action = self.choose_action(estimate)
            forward, yaw = self.explore_motion(*self.compute_motion(action))
            if not all(math.isfinite(v) for v in (forward, yaw)):
                raise ValueError('controller returned a nonfinite command')
            self.previous_error = None
        except Exception as exc:
            forward, yaw = 0., 0.
            error = str(exc)
            if error != self.previous_error:
                self.get_logger().error(f'Navigation failed; requesting zero motion: {error}')
                self.previous_error = error
        msg = MotionRequest()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.forward_speed_m_s, msg.yaw_rate_rad_s = float(forward), float(yaw)
        self.publisher.publish(msg)
        try:
            self.publish_debug_image(msg.forward_speed_m_s, msg.yaw_rate_rad_s)
        except Exception as exc:  # debugging must never affect motion
            self.get_logger().error(f'Debug image failed: {exc}')

    def publish_debug_image(self, forward, yaw):
        """Publish the camera view with what the code sees drawn on top.

        Magenta tint: pixels counted as yellow. Red: blue (no-go). Orange,
        labelled CAR: another car (no-go). Green dots: the trail being
        followed. Cyan dots: other paths seen (the other way round a split).
        Yellow line: the straight line used by 'taut' steering. White circle:
        steering target. Grey dots either side of the followed path: where
        the treads' outer edges would be (footprint_check on). Top line:
        status and the request sent; second line: the active mode.
        """
        if self.debug_pub is None or self.mask is None or self.image is None:
            return
        h, w = self.mask.shape
        view = cv2.resize(self.image, (w, h), interpolation=cv2.INTER_AREA)
        yellow = self.mask.astype(bool)
        view[yellow] = (.4 * view[yellow] + .6 * np.array([255, 0, 255])).astype(np.uint8)
        view[self.blue.astype(bool)] = (0, 0, 255)
        view[self.cars.astype(bool)] = (0, 140, 255)
        for contour in cv2.findContours(self.cars, cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[-2]:
            x, y = contour[:, 0].min(axis=0)
            cv2.putText(view, 'CAR', (int(x), max(int(y) - 2, 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, .35, (0, 140, 255), 1, cv2.LINE_AA)
        for path in self.paths:
            if path is not self.trail_px:
                for u, v in path[::2]:
                    cv2.circle(view, (int(u), int(v)), 1, (255, 255, 0), -1)
        body = body_px(h, self.horizon_frac, self.path_settings['robot_half_width_px'],
                       self.path_settings['footprint_skip_frac'])
        for u, v in (self.trail_px or [])[::3]:
            half = body[min(int(v), h - 1)]
            if half > 0:
                for edge in (u - half, u + half):
                    cv2.circle(view, (int(edge), int(v)), 1, (200, 200, 200), -1)
        for u, v in self.trail_px or []:
            cv2.circle(view, (int(u), int(v)), 2, (0, 255, 0), -1)
        if self.target is not None and self.H is not None:
            u, v = pixel_to_floor([self.target], np.linalg.inv(self.H))[0]
            if self.used_taut and self.trail_px:
                # yellow: the line starts at the robot (bottom centre);
                # blue_aware: at the start of the traced path.
                su, sv = ((w / 2, h - 1) if self.perception == 'yellow'
                          else self.trail_px[0])
                cv2.line(view, (int(su), int(sv)), (int(u), int(v)), (0, 255, 255), 2)
            cv2.circle(view, (int(u), int(v)), 6, (255, 255, 255), 2)
        if self.H is None:
            status = 'NO CALIBRATION'
        elif self.exploring:
            status = 'EXPLORING (blue: turning right)'
        elif not self.trail_px:
            status = 'NO PATH'
        elif self.blocked:
            status = 'BLOCKED (treads)'
        elif self.target is None:
            status = 'PATH TOO SHORT'
        else:
            status = 'FOLLOWING (taut)' if self.used_taut else 'FOLLOWING'
        lines = (f'{status}  v={forward:.2f} yaw={yaw:+.2f}', self.mode_text())
        for y, text in zip((14, 28), lines):
            for color, thickness in (((0, 0, 0), 3), ((255, 255, 255), 1)):
                cv2.putText(view, text, (4, y), cv2.FONT_HERSHEY_SIMPLEX, .4,
                            color, thickness, cv2.LINE_AA)
        msg = self.bridge.cv2_to_imgmsg(view, encoding='bgr8')
        msg.header.stamp = self.image_stamp
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    node = Navigation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
