"""Image -> binary yellow mask. No ROS imports, so it can be tested offline."""
import cv2
import numpy as np

# OpenCV HSV ranges: H 0-179, S 0-255, V 0-255. Yellow sits around H 20-35.
# Tune these on real frames from the robot's camera and lighting.
YELLOW_LOW = (18, 120, 140)
YELLOW_HIGH = (35, 255, 255)


def yellow_mask(image, low=YELLOW_LOW, high=YELLOW_HIGH, scale=0.25):
    """Return an H x W uint8 grid: 1 where the pixel is yellow, 0 otherwise.

    image: BGR array (H x W x 3) as produced by cv_bridge with 'bgr8'.
    scale: resize factor applied first (0.25 turns 1280x720 into 320x180).
    Returns None if image is None.
    """
    if image is None:
        return None
    if scale != 1:
        image = cv2.resize(image, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    return (mask > 0).astype(np.uint8)


def close_gaps(mask, gap_px=(9, 25)):
    """Merge yellow groups separated by small gaps (morphological closing).

    gap_px: (sideways, along-path) gap sizes in mask pixels to bridge. Dilates
    then erodes by the same rectangle, so gaps narrower than the rectangle
    fill in while the path keeps its width. Taller than wide by default:
    bridges gaps along the direction of travel (e.g. between sheets of paper)
    without merging separate paths that run side by side. (0, 0) disables it.
    """
    gap_x, gap_y = (int(g) for g in gap_px)
    if gap_x <= 1 and gap_y <= 1:
        return mask
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (max(gap_x, 1), max(gap_y, 1)))
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def path_region(mask, min_area=200, start_frac=0.33):
    """Keep only the connected yellow region the robot is on (drops noise).

    Picks the largest region of at least min_area pixels whose lowest row
    lies in the bottom start_frac of the image, i.e. the yellow nearest the
    robot. Returns a 0/1 mask; all zeros if no region qualifies.
    """
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    bottom_limit = mask.shape[0] * (1 - start_frac)
    best, best_area = None, min_area - 1
    for i in range(1, count):  # label 0 is background
        area = stats[i, cv2.CC_STAT_AREA]
        lowest_row = stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] - 1
        if lowest_row >= bottom_limit and area > best_area:
            best, best_area = i, area
    if best is None:
        return np.zeros_like(mask)
    return (labels == best).astype(np.uint8)


def centerline(mask, step=5, min_width=10, max_jump=40, start_frac=0.33,
               min_area=200, gap_px=(9, 25)):
    """Trace the middle of the yellow path from the bottom of the mask upward.

    Small gaps are bridged first (see close_gaps). Only the connected yellow
    region nearest the robot is then traced (see path_region), so patches of
    yellow-ish floor cannot start or hijack it.

    Each row's centre is the midpoint of the region's outer left and right
    edges in that row, so holes inside the path (tape, seams between sheets)
    do not move it. At a fork the outer edges span both branches, so the trail
    heads between them until they separate into different regions.

    Returns a list of (u, v) pixel points ordered near -> far (empty if the
    path is not found). Pixels are in the mask's own resolution.

    step:       rows between samples.
    min_width:  rows where the path is narrower than this (px) count as no path.
    max_jump:   largest sideways move (px) allowed between samples; a bigger
                jump ends the trail.
    start_frac: the trail must start within this bottom fraction of the image.
    min_area:   smallest region (px) that counts as path.
    gap_px:     (sideways, along-path) gaps to bridge; (0, 0) disables.
    """
    if mask is None:
        return []
    mask = path_region(close_gaps(mask, gap_px), min_area, start_frac)
    height = mask.shape[0]
    points, prev = [], None
    for v in range(height - 1, -1, -step):
        xs = np.flatnonzero(mask[v])
        if len(xs) == 0 or xs[-1] - xs[0] + 1 < min_width:
            if prev is None and v >= height * (1 - start_frac):
                continue  # still looking for where the path begins
            break  # path ended (or was never found near the robot)
        center = (xs[0] + xs[-1]) / 2
        if prev is not None and abs(center - prev) > max_jump:
            break
        prev = float(center)
        points.append((prev, v))
    return points
