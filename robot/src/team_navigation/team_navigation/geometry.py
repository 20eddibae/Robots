"""Pixel (u, v) -> floor (x, y) in metres, robot frame. No ROS imports.

Robot frame (ROS convention): x is forward, y is left, origin at the robot's
chosen reference point (e.g. midway between the treads). Only valid for points
on the floor plane.

Calibrate once at the robot with the camera in its final mount:
  1. Tape 4+ marks on the floor, spread near/far and left/right.
  2. Measure each mark's (x, y) in metres.
  3. Read each mark's pixel (u, v) in a camera frame AT THE MASK RESOLUTION
     (the same scale passed to vision.yellow_mask).
  4. H = compute_homography(pixels, floor); store H's 9 numbers in config.
"""
import cv2
import numpy as np


def compute_homography(pixel_pts, floor_pts):
    """Return the 3x3 matrix H mapping pixel (u, v) to floor (x, y).

    pixel_pts, floor_pts: matching sequences of 4+ (u, v) and (x, y) pairs.
    With more than 4 pairs, RANSAC rejects a badly clicked or measured mark.
    """
    pixel_pts = np.asarray(pixel_pts, np.float64).reshape(-1, 2)
    floor_pts = np.asarray(floor_pts, np.float64).reshape(-1, 2)
    if len(pixel_pts) < 4 or len(pixel_pts) != len(floor_pts):
        raise ValueError('need 4+ matching pixel/floor points')
    method = cv2.RANSAC if len(pixel_pts) > 4 else 0
    H, _ = cv2.findHomography(pixel_pts, floor_pts, method)
    if H is None:
        raise ValueError('homography failed; are 3 of the points in a line?')
    return H


def pixel_to_floor(points, H):
    """Convert pixel points [(u, v), ...] to floor points; returns an N x 2 array.

    Returns an empty (0, 2) array when given no points.
    """
    points = np.asarray(points, np.float64).reshape(-1, 1, 2)
    if len(points) == 0:
        return np.empty((0, 2))
    return cv2.perspectiveTransform(points, np.asarray(H, np.float64)).reshape(-1, 2)


def homography_from_list(values):
    """Build H from 9 numbers in row order, e.g. a parameter list from YAML."""
    H = np.asarray(values, np.float64)
    if H.size != 9 or not np.all(np.isfinite(H)):
        raise ValueError('homography must be 9 finite numbers')
    return H.reshape(3, 3)
