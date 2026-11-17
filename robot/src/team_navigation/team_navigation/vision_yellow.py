"""Yellow-only path tracing from the best-path branch (perception: yellow).

Image -> binary yellow mask -> corridor -> taut aim point. Ignores blue and
cars; vision.py has the blue/car-aware version. No ROS imports, so it can
be tested offline.
"""
import cv2
import numpy as np

# OpenCV HSV ranges: H 0-179, S 0-255, V 0-255. Yellow sits around H 20-35.
# Tune these on real frames from the robot's camera and lighting.
YELLOW_LOW = (18, 60, 140)
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


def _start_label(stats, height, min_area=200, start_frac=0.33):
    """Label of the largest region of at least min_area pixels whose lowest
    row lies in the bottom start_frac of the image, or None."""
    bottom_limit = height * (1 - start_frac)
    best, best_area = None, min_area - 1
    for i in range(1, len(stats)):  # label 0 is background
        area = stats[i, cv2.CC_STAT_AREA]
        lowest_row = stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] - 1
        if lowest_row >= bottom_limit and area > best_area:
            best, best_area = i, area
    return best


def path_region(mask, min_area=200, start_frac=0.33):
    """Keep only the connected yellow region the robot is on (drops noise).

    Picks the largest region of at least min_area pixels whose lowest row
    lies in the bottom start_frac of the image, i.e. the yellow nearest the
    robot. Returns a 0/1 mask; all zeros if no region qualifies.
    """
    _, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    best = _start_label(stats, mask.shape[0], min_area, start_frac)
    if best is None:
        return np.zeros_like(mask)
    return (labels == best).astype(np.uint8)


def _row_span(region_row, min_width):
    """(left, right) outer edges of a 0/1 row, or None if narrower than min_width."""
    xs = np.flatnonzero(region_row)
    if len(xs) == 0 or xs[-1] - xs[0] + 1 < min_width:
        return None
    return int(xs[0]), int(xs[-1])


def _jump(labels, stats, v, max_rows, center, min_area, min_width, max_jump):
    """Look up to max_rows rows above row v for a yellow region (possibly the
    one already followed) that carries on where the trail was heading: its
    outer edges in that row centred within max_jump px of `center`.
    Returns (label, row) or None."""
    for row in range(v, max(v - int(max_rows), 0) - 1, -1):
        for label in np.unique(labels[row]):
            if label == 0 or stats[label, cv2.CC_STAT_AREA] < min_area:
                continue
            span = _row_span(labels[row] == label, min_width)
            if span and abs((span[0] + span[1]) / 2 - center) <= max_jump:
                return int(label), row
    return None


def centerline(mask, step=5, min_width=10, max_jump=40, start_frac=0.33,
               min_area=200, gap_px=(9, 25), jump_px=0, horizon_frac=.28):
    """Trace the middle of the yellow path; [(u, v), ...] ordered near -> far.

    Same as corridor() but returns only each row's centre point.
    """
    return [(c, v) for c, v, _, _ in corridor(mask, step, min_width, max_jump,
                                              start_frac, min_area, gap_px,
                                              jump_px, horizon_frac)]


def corridor(mask, step=5, min_width=10, max_jump=40, start_frac=0.33,
             min_area=200, gap_px=(9, 25), jump_px=0, horizon_frac=.28):
    """Trace the yellow path from the bottom of the mask upward.

    Small gaps are bridged first (see close_gaps). Only the connected yellow
    region nearest the robot is then traced (see path_region), so patches of
    yellow-ish floor cannot start or hijack it.

    Each row's centre is the midpoint of the region's outer left and right
    edges in that row, so holes inside the path (tape, seams between sheets)
    do not move it. At a fork the outer edges span both branches, so the trail
    heads between them until they separate into different regions.

    Returns a list of (center_u, v, left_u, right_u) per sampled row, ordered
    near -> far (empty if the path is not found): the row's centre and the
    path's outer left and right edges. Pixels are in the mask's resolution.

    step:       rows between samples.
    min_width:  rows where the path is narrower than this (px) count as no path.
    max_jump:   largest sideways move (px) allowed between samples; a bigger
                jump ends the trail.
    start_frac: the trail must start within this bottom fraction of the image.
    min_area:   smallest region (px) that counts as path.
    gap_px:     (sideways, along-path) gaps to bridge; (0, 0) disables.
    jump_px:    where the trail would end (the region stops, or its centre
                swings more than max_jump sideways, e.g. because another
                sheet joins it at the image edge), look up to this many rows
                further (at the bottom row, shrinking with distance towards
                the horizon at horizon_frac of the image height) for yellow
                in line with the trail (within max_jump), and carry on along
                it: crosses wider gaps between sheets than gap_px, without
                merging regions side by side. 0 = off.
    """
    if mask is None:
        return []
    closed = close_gaps(mask, gap_px)
    height = closed.shape[0]
    _, labels, stats, _ = cv2.connectedComponentsWithStats(closed, connectivity=8)
    current = _start_label(stats, height, min_area, start_frac)
    if current is None:
        return []
    horizon = horizon_frac * height
    points, prev = [], None
    v = height - 1
    while v >= 0:
        span = _row_span(labels[v] == current, min_width)
        if span is None and prev is None:
            if v >= height * (1 - start_frac):
                v -= step
                continue  # still looking for where the path begins
            break  # never found near the robot
        if prev is not None and (span is None or abs((span[0] + span[1]) / 2 - prev) > max_jump):
            scale = min(max((v - horizon) / max(height - 1 - horizon, 1.), 0.), 1.)
            found = _jump(labels, stats, v, jump_px * scale, prev,
                          min_area, min_width, max_jump) if jump_px > 0 else None
            if found is None:
                break  # path ended
            current, v = found
            continue
        center = (span[0] + span[1]) / 2
        prev = float(center)
        points.append((prev, v, span[0], span[1]))
        v -= step
    return points


def farthest_visible(rows, start, margin_frac=0.15):
    """Farthest centre point reachable in a straight line inside the path.

    The "taut string": instead of following every wiggle of the centreline,
    aim at the farthest centre point such that the straight segment from
    `start` (the robot, e.g. bottom centre of the image) to it stays inside
    the path's outer edges, minus margin_frac of the path's width on each
    side, at every sampled row in between. On a straight path this aims far
    ahead; on a curve the segment hugs the inside of the turn.

    A straight line on the floor is a straight line in the image, so this
    works in pixels without camera calibration.

    rows:  corridor() output, near -> far.
    start: (u, v) pixel the segment starts from.
    Returns (index into rows, (u, v)) or None if not even the nearest
    centre point can be reached (e.g. the robot is off the path).
    """
    su, sv = start
    for i in range(len(rows) - 1, -1, -1):
        tu, tv = rows[i][0], rows[i][1]
        if tv >= sv:
            continue
        clear = True
        for cu, v, left, right in rows[:i + 1]:
            if v > sv:
                continue
            u = su + (tu - su) * (sv - v) / (sv - tv)
            margin = margin_frac * (right - left)
            if not left + margin <= u <= right - margin:
                clear = False
                break
        if clear:
            return i, (tu, tv)
    return None
