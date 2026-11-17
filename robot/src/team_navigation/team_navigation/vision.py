"""Image -> yellow/blue/obstacle masks -> candidate paths. No ROS imports
(so it can be tested offline).

What the camera sees is sorted into four kinds of surface:
  yellow    the road. Path states are estimated here.
  blue      absolutely no-go. A path never touches it or its safety margin.
  obstacle  another car (a bright, colourless object standing on the floor).
            Also no-go, with a wider margin, and labelled separately.
  other     (grey carpet, tape, seams) allowed, but only to cross a gap from
            one piece of yellow to the next.

Paths are estimated on a grid instead of with a few hundred sampled
particles: every floor pixel is one possible state ("the path passes through
here"), so nothing depends on random numbers and the same image always gives
the same paths. Working up the image from the robot one row at a time, each
pixel keeps the best total reward of any path reaching it from the row below
(a Viterbi / dynamic-programming pass, the exact version of what a particle
filter approximates). Reward is earned on yellow, paid to cross a gap or to
move sideways, and no-go can never be entered or crossed. The best path is
read back from the best-scoring pixel. To map a split in the road, the side
the best path took around a no-go area is closed and the pass repeated,
which gives the best path round the other side.

Perspective: a sheet far from the robot looks much smaller and flatter than
one at its treads. floor_scale() gives the apparent size of the floor in each
image row (1 at the bottom, 0 at the horizon); the road width, the no-go
margins, the allowed gap, the sideways-move price and the reward per row are
all scaled with it, so they mean the same floor distance near and far.
"""
import cv2
import numpy as np

# OpenCV HSV ranges: H 0-179, S 0-255, V 0-255. Yellow sits around H 20-35,
# blue around H 95-130. The other cars' bodies are light grey: any hue, low
# saturation, bright (the carpet is darker, the paper far more saturated).
# Tune on real frames from the robot's camera and lighting.
YELLOW_LOW = (18, 60, 140)
YELLOW_HIGH = (35, 255, 255)
BLUE_LOW = (95, 60, 40)
BLUE_HIGH = (130, 255, 255)
CAR_LOW = (0, 0, 150)
CAR_HIGH = (179, 80, 255)

MIN_SCALE = .05     # rows this close to the horizon are too squashed to trust
ON_ROAD = .3        # yellow support at or above this counts as "on the road"
GAP_COST = 1.       # crossing a gap costs this much yellow road of equal length
START_COST = 3.     # price per pixel of starting away from the bottom centre
FAR_SCALE = .25     # rows beyond this scale earn no more per row than rows at it
MAX_SHIFT = 12      # most columns a path may move sideways per image row
MIN_PATH_POINTS = 4
MAX_DETOURS = 4     # most no-go areas tried for a way round the other side
REACH_FRAC = .6     # other paths must get this far towards the horizon, relative to the best


def _hsv(image, scale):
    if scale != 1:
        image = cv2.resize(image, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)


def _in_range(hsv, low, high):
    mask = cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    return (mask > 0).astype(np.uint8)


def yellow_mask(image, low=YELLOW_LOW, high=YELLOW_HIGH, scale=0.25):
    """Return an H x W uint8 grid: 1 where the pixel is yellow, 0 otherwise.

    image: BGR array (H x W x 3) as produced by cv_bridge with 'bgr8'.
    scale: resize factor applied first (0.25 turns 1280x720 into 320x180).
    Returns None if image is None.
    """
    if image is None:
        return None
    return _in_range(_hsv(image, scale), low, high)


def color_masks(image, yellow_low=YELLOW_LOW, yellow_high=YELLOW_HIGH,
                blue_low=BLUE_LOW, blue_high=BLUE_HIGH,
                car_low=CAR_LOW, car_high=CAR_HIGH, scale=0.25):
    """Return (yellow, blue, car_coloured) 0/1 masks from one HSV conversion.

    Same conventions as yellow_mask. car_coloured is every pixel with a car
    body's colour, including walls and glare; pass it through obstacle_mask
    to keep only objects on the floor. Returns three Nones if image is None.
    """
    if image is None:
        return None, None, None
    hsv = _hsv(image, scale)
    yellow = _in_range(hsv, yellow_low, yellow_high)
    # Pale paper is also bright and weakly coloured: yellow is never a car.
    return (yellow, _in_range(hsv, blue_low, blue_high),
            _in_range(hsv, car_low, car_high) & (1 - yellow))


def floor_scale(height, horizon_frac=.28):
    """Apparent size of the floor in each image row: 1 at the bottom row,
    falling linearly to 0 at the horizon (and 0 above it).

    horizon_frac: the horizon's row as a fraction of the image height, i.e.
    where the floor would vanish if it went on forever (0 = top of image).
    """
    horizon = horizon_frac * height
    rows = np.arange(height, dtype=np.float64)
    return np.clip((rows - horizon) / max(height - 1 - horizon, 1.), 0., 1.)


def obstacle_mask(car_coloured, horizon_frac=.28, min_area_px=600., reach_px=12.):
    """Keep the car-coloured blobs that are objects standing on the floor.

    A blob counts when its lowest point is on the floor, at least reach_px
    rows (x the floor scale there) below the horizon, and it is big for that
    distance: at least min_area_px pixels x scale^2 (a car twice as far away
    covers a quarter of the pixels). Each kept blob is filled out to its
    convex outline so dark parts (treads, cables) inside it count too.
    Returns a 0/1 mask the same shape as car_coloured (None if that is None).
    """
    if car_coloured is None:
        return None
    result = np.zeros_like(car_coloured)
    if not car_coloured.any():
        return result
    h = car_coloured.shape[0]
    scale = floor_scale(h, horizon_frac)
    horizon = horizon_frac * h
    solid = cv2.morphologyEx(car_coloured, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours = cv2.findContours(solid, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    for contour in contours:
        bottom = int(contour[:, 0, 1].max())
        size = scale[bottom]
        if (bottom - horizon >= reach_px * size and size >= MIN_SCALE
                and cv2.contourArea(contour) >= min_area_px * size * size):
            cv2.fillConvexPoly(result, cv2.convexHull(contour), 1)
    return result


def drop_specks(mask, horizon_frac=.28, min_area_px=150.):
    """Remove blobs too small to be real paper at their distance.

    A blob is kept when it has at least min_area_px pixels x scale^2 at its
    lowest row (a sheet twice as far away covers a quarter of the pixels;
    at least 2 px). Carpet and tape produce scattered blue specks of a few
    pixels, which would otherwise count as no-go and, with the footprint
    check, block a whole row. Returns a 0/1 mask (None if mask is None).
    """
    if mask is None or min_area_px <= 0 or not mask.any():
        return mask
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    scale = floor_scale(mask.shape[0], horizon_frac)
    bottom = stats[1:, cv2.CC_STAT_TOP] + stats[1:, cv2.CC_STAT_HEIGHT] - 1
    needed = np.maximum(min_area_px * scale[bottom] ** 2, 2.)
    keep = np.zeros(count, np.uint8)
    keep[1:] = stats[1:, cv2.CC_STAT_AREA] >= needed
    return keep[labels]


def _off_road_distance(on_road, scale):
    """Floor distance (bottom-row px) from each pixel to the nearest road
    pixel in its own row or column. Far away a pixel sideways is 1/scale
    bottom-row pixels, a pixel up or down about 1/scale^2."""
    h, w = on_road.shape
    cols, rows = np.arange(w)[None, :], np.arange(h)[:, None]
    far = 10 * (h + w)
    left = np.maximum.accumulate(np.where(on_road, cols, -far), axis=1)
    right = np.minimum.accumulate(np.where(on_road, cols, far)[:, ::-1], axis=1)[:, ::-1]
    above = np.maximum.accumulate(np.where(on_road, rows, -far), axis=0)
    below = np.minimum.accumulate(np.where(on_road, rows, far)[::-1], axis=0)[::-1]
    size = np.maximum(scale, MIN_SCALE)[:, None]
    return np.minimum(np.minimum(cols - left, right - cols) / size,
                      np.minimum(rows - above, below - rows) / (size * size))


def _window_support(yellow, radius):
    """Fraction of yellow in a sideways window of `radius` px (per row) around
    each pixel; beyond the image edge counts as not yellow."""
    h, w = yellow.shape
    radius = np.asarray(radius).reshape(-1, 1)
    cols = np.arange(w)[None, :]
    total = np.zeros((h, w + 1), np.float32)
    total[:, 1:] = np.cumsum(yellow, axis=1)
    inside = (np.take_along_axis(total, np.clip(cols + radius + 1, 0, w), 1)
              - np.take_along_axis(total, np.clip(cols - radius, 0, w), 1))
    return inside / (2 * radius + 1)


def _row_distance(mask):
    """Sideways distance (px) from each pixel to the nearest mask pixel in the
    same row (large where the row has none)."""
    w = mask.shape[1]
    cols = np.arange(w)[None, :]
    on = mask.astype(bool)
    left = np.maximum.accumulate(np.where(on, cols, -10 * w), axis=1)
    right = np.minimum.accumulate(np.where(on, cols, 11 * w)[:, ::-1], axis=1)[:, ::-1]
    return np.minimum(cols - left, right - cols).astype(np.float32)


def body_px(height, horizon_frac=.28, robot_half_width_px=0., footprint_skip_frac=.1):
    """Robot half-width in pixels for each image row (0 where not applied).

    robot_half_width_px is measured at the bottom row and shrinks with
    distance like every other size (see floor_scale). Rows in the bottom
    footprint_skip_frac of the image get 0: the robot is already there and
    cannot steer out of it, and its treads are mostly outside the image.
    """
    scale = floor_scale(height, horizon_frac)
    active = np.arange(height) < height * (1 - footprint_skip_frac)
    return robot_half_width_px * scale * active


def footprint_support(yellow, horizon_frac=.28, robot_half_width_px=150.,
                      footprint_skip_frac=.1):
    """H x W fraction (0..1) of the robot's width that would be on yellow if
    the robot's centre were at each pixel, i.e. how much of the span between
    its treads is on the road. Rows the footprint is not applied to get 1."""
    radius = np.rint(body_px(yellow.shape[0], horizon_frac, robot_half_width_px,
                             footprint_skip_frac)).astype(int)
    support = _window_support(yellow, radius)
    support[radius == 0, :] = 1.
    return support


def floor_fields(yellow, blue=None, obstacle=None, horizon_frac=.28,
                 half_width_px=40., blue_margin_px=6., obstacle_margin_px=15.,
                 blue_hug_px=20., blue_hug_gain=.3, max_gap_px=120.,
                 robot_half_width_px=0., footprint_skip_frac=.1):
    """Return (scale, support, nogo, reward) describing where a path may be.

    A path is where the robot's CENTRE goes. With robot_half_width_px > 0
    (the robot's half-width at the bottom row, see body_px) blue and
    obstacles are also widened SIDEWAYS, within each row, by the robot's
    half-width there, so a path keeps both treads, not just the centre, off
    blue and away from cars. (Sideways only: the width is across the robot,
    so it must not reach forward to block rows the blue is not in.)

    scale:   floor_scale for each row.
    support: H x W floats 0..1, the fraction of yellow in a sideways window
             around each pixel. The window is half_width_px either side at the
             bottom row and shrinks with distance. It peaks in the middle of
             the road and ignores seams, tape and small holes.
    nogo:    H x W bools, True on blue and obstacles, within their margins
             (bottom-row px, shrinking with distance), and at or above the
             horizon.
    reward:  H x W floats, what a path gains by passing through each pixel,
             per unit of floor distance covered:
               on the road    support^2 (most mid-road), times
                              (1 + blue_hug_gain) within blue_hug_px of blue's
                              margin, so a path rounding blue takes the tight
                              inside line; with robot_half_width_px > 0,
                              support x footprint_support instead of
                              support^2, so paths that would put a tread off
                              the yellow earn less (a preference, not a ban,
                              so gaps between sheets can still be crossed);
               off the road   -GAP_COST, if road is within max_gap_px / 2 (a
                              gap of up to max_gap_px can be crossed, at a
                              price);
               anywhere else, and all no-go: -infinity (never entered).
    """
    h, w = yellow.shape
    scale = floor_scale(h, horizon_frac)
    radius = np.rint(half_width_px * scale).astype(int)
    support = _window_support(yellow, radius)
    on_road = support >= ON_ROAD
    on_road_reward = support * support
    if robot_half_width_px > 0:
        on_road_reward = support * footprint_support(yellow, horizon_frac,
                                                     robot_half_width_px, footprint_skip_frac)
    reward = np.where(on_road, on_road_reward, -GAP_COST).astype(np.float32)
    nogo = np.repeat(scale[:, None] < MIN_SCALE, w, axis=1)
    body = body_px(h, horizon_frac, robot_half_width_px, footprint_skip_frac)[:, None]
    for mask, margin_px in ((blue, blue_margin_px), (obstacle, obstacle_margin_px)):
        if mask is not None and mask.any():
            away = cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2, 3)
            margin = margin_px * scale[:, None]
            near = away <= margin
            if body.any():
                side = _row_distance(mask)
                near |= side <= body + margin
            nogo |= near
            if mask is blue:
                hug_px = blue_hug_px * scale[:, None]
                hug = away <= margin + hug_px
                if body.any():
                    hug |= side <= body + margin + hug_px
                hug &= on_road
                reward[hug] *= 1. + blue_hug_gain
    # Reward is per unit of floor: a far row covers more floor than a near
    # one (up to a limit, so the far end does not outweigh the road nearby).
    reward /= np.maximum(scale, FAR_SCALE)[:, None] ** 2
    reward[nogo | (~on_road & (_off_road_distance(on_road, scale) > max_gap_px / 2))] = -np.inf
    return scale, support, nogo, reward


def _sweep(best, step, reward, move, first_row):
    """Fill best/step for rows first_row, first_row - 1, ... up to row 0.

    best[r, c]: the highest total reward of any path from the robot to pixel
    (r, c). step[r, c]: which sideways move (index into the move offsets) that
    path made coming up from row r + 1. move[r] is the (W, moves) price of
    each sideways move into row r (-inf where it would cross no-go).
    """
    reach = (move.shape[2] - 1) // 2
    padded = np.full(best.shape[1] + 2 * reach, -np.inf, np.float32)
    windows = np.lib.stride_tricks.as_strided(
        padded, (best.shape[1], 2 * reach + 1), padded.strides * 2)
    for row in range(first_row, -1, -1):
        padded[reach:-reach] = best[row + 1]
        options = windows + move[row]
        step[row] = options.argmax(axis=1)
        best[row] = options.max(axis=1) + reward[row]


def _walk_back(best, step, row, reach):
    """Return the best path ending at the best pixel of rows 0..row, as an
    N x 2 array of (u, v) from the bottom row up, with its total reward.
    The path is None if nothing there is worth reaching."""
    rows = best.shape[0]
    end = np.unravel_index(np.argmax(best[:row + 1]), best[:row + 1].shape)
    total, (r, c) = float(best[end]), end
    if not np.isfinite(total) or total <= 0:
        return None, total
    path = [(c, r)]
    while r < rows - 1:
        c = c + int(step[r, c]) - reach
        r += 1
        path.append((c, r))
    return np.array(path[::-1], np.float64), total


def trace_paths(yellow, blue=None, obstacle=None, horizon_frac=.28,
                half_width_px=40., blue_margin_px=6., obstacle_margin_px=15.,
                blue_hug_px=20., blue_hug_gain=.3, max_gap_px=120.,
                turn_cost=.15, max_paths=3, robot_half_width_px=0.,
                footprint_skip_frac=.1):
    """Estimate the paths leading away from the robot; the best one first.

    robot_half_width_px, footprint_skip_frac: see floor_fields / body_px
    (keep both treads, not just the centre, clear of blue and cars).

    Returns a list of paths, each a list of (u, v) mask pixels ordered
    near -> far (one point per image row), starting at the robot (bottom
    centre). Empty if no yellow is reachable. No returned point is on blue,
    on an obstacle or inside their margins.

    The first path is the one with the highest total reward (see the module
    docstring and floor_fields). Each further path is the best one that
    passes on the other side of a no-go area (blue island, car) which the
    first path goes around, i.e. the other way through a split in the road.

    yellow, blue, obstacle: 0/1 masks (blue/obstacle=None: none present).
    horizon_frac:   see floor_scale.
    half_width_px:  about half the road's width at the bottom row.
    blue_margin_px, obstacle_margin_px: clearance kept, in bottom-row pixels.
    blue_hug_px, blue_hug_gain: pull towards blue (gain 0 = none).
    max_gap_px:     widest stretch of non-yellow floor a path may cross, in
                    bottom-row pixels (far-away gaps count for more).
    turn_cost:      price of moving sideways instead of straight up the image.
                    Higher = straighter, smoother paths that cut corners more.
    max_paths:      most paths returned.
    """
    if yellow is None or not yellow.any():
        return []
    h, w = yellow.shape
    scale, support, nogo, reward = floor_fields(
        yellow, blue, obstacle, horizon_frac, half_width_px, blue_margin_px,
        obstacle_margin_px, blue_hug_px, blue_hug_gain, max_gap_px,
        robot_half_width_px, footprint_skip_frac)
    top =int(np.argmax(scale >= MIN_SCALE))  # first usable row; work on rows top..h-1
    scale, support, nogo, reward = scale[top:], support[top:], nogo[top:], reward[top:]
    rows = h - top

    # Price of each sideways move (columns shifted going up one row):
    # turn_cost x shift^2, so a steady diagonal is cheaper than a sudden
    # sidestep; impossible if it crosses no-go.
    offsets = np.arange(-MAX_SHIFT, MAX_SHIFT + 1)
    cols = np.arange(w)[:, None]
    source = cols + offsets[None, :]
    low = np.clip(np.minimum(cols, source), 0, w - 1)
    high = np.clip(np.maximum(cols, source), 0, w - 1) + 1
    closed_before = np.zeros((rows, w + 1), np.int32)
    closed_before[:, 1:] = np.cumsum(np.isinf(reward), axis=1)
    move = np.where(closed_before[:, high] == closed_before[:, low],
                    (-turn_cost * offsets * offsets).astype(np.float32), -np.inf)

    # Every pixel is one possible state. Start: the bottom row, at a steep
    # price for being away from its centre (the robot).
    best = np.full((rows, w), -np.inf, np.float32)
    step = np.zeros((rows, w), np.uint8)
    best[-1] = reward[-1] - START_COST * np.abs(np.arange(w) - (w - 1) / 2)
    _sweep(best, step, reward, move, rows - 2)
    first, total = _walk_back(best, step, rows - 1, MAX_SHIFT)
    if first is None:
        return []
    found = [(total, first)]

    # Other ways round: for each no-go area beside the first path, close the
    # side the path used (at the row where it first comes level with the
    # area) and look again from that row up.
    for row, start, stop in _detours(first, nogo)[:MAX_DETOURS]:
        other_side = reward[row, :start] if stop == w else reward[row, stop:]
        if row > rows - 2 or row < 1 or not np.isfinite(other_side).any():
            continue
        other_reward = reward.copy()
        other_reward[row, start:stop] = -np.inf
        other_best, other_step = best.copy(), step.copy()
        _sweep(other_best, other_step, other_reward, move, row)
        path, total = _walk_back(other_best, other_step, row - 1, MAX_SHIFT)
        if path is not None:
            found.append((total, path))

    paths, reach = [], []
    for total, path in sorted(found, key=lambda item: -item[0]):
        path = _finish_path(path, support, nogo)
        if len(path) >= MIN_PATH_POINTS and not any(_same_path(path, p) for p in paths):
            paths.append(path)
            reach.append(1. - scale[int(round(path[-1, 1]))])
    return [[(float(u), float(v) + top) for u, v in path]
            for path, far in zip(paths, reach) if far >= REACH_FRAC * reach[0]][:max_paths]


def _detours(path, nogo):
    """List the no-go areas the path passes beside, biggest first.

    Each entry is (row, start, stop): row is where the path first comes level
    with the area, and columns start..stop-1 of it are the side the path
    went (from the area's edge to the image edge). Closing them forces a path
    to pass the area on its other side.
    """
    w = nogo.shape[1]
    _, labels, stats, _ = cv2.connectedComponentsWithStats(nogo.astype(np.uint8),
                                                           connectivity=8)
    cols = np.arange(w)[None, :]
    left = np.maximum.accumulate(np.where(nogo, cols, -1), axis=1)
    right = np.minimum.accumulate(np.where(nogo, cols, w)[:, ::-1], axis=1)[:, ::-1]
    u, v = path[:, 0].astype(int), path[:, 1].astype(int)  # ordered near -> far
    detours, seen = [], set()
    for edge, on_left in ((left[v, u], True), (right[v, u], False)):
        for i in np.flatnonzero((edge >= 0) & (edge < w)):
            area = (labels[v[i], edge[i]], on_left)
            if area not in seen:
                seen.add(area)
                size = int(stats[area[0], cv2.CC_STAT_AREA])
                detours.append((size, int(v[i]), int(edge[i]) + 1, w) if on_left
                               else (size, int(v[i]), 0, int(edge[i])))
    return [detour[1:] for detour in sorted(detours, reverse=True)]


def _finish_path(path, support, nogo):
    """End a traced path at its last yellow, smooth it, and stop before no-go."""
    h, w = support.shape

    def look(field):
        cols = np.clip(np.rint(path[:, 0]).astype(int), 0, w - 1)
        rows = np.clip(np.rint(path[:, 1]).astype(int), 0, h - 1)
        return field[rows, cols]

    on_road = np.flatnonzero(look(support) >= ON_ROAD)
    if len(on_road) == 0:
        return np.empty((0, 2))
    path = path[:on_road[-1] + 1]
    if len(path) >= 7:  # 7-point moving average, end points held in place
        raw = path
        padded = np.concatenate([path[:1]] * 3 + [path] + [path[-1:]] * 3)
        path = np.stack([np.convolve(padded[:, k], np.full(7, 1 / 7), 'valid')
                         for k in (0, 1)], axis=1)
        # Smoothing can pull a path that hugs no-go (e.g. blue widened by the
        # robot's footprint) slightly into it; keep the traced point there
        # instead of cutting the whole path short.
        pulled_in = look(nogo)
        path[pulled_in] = raw[pulled_in]
    blocked = np.flatnonzero(look(nogo))
    return path[:blocked[0]] if len(blocked) else path


def _same_path(a, b, tolerance_px=10.):
    """True if two paths coincide over the length they share."""
    shared = min(len(a), len(b))
    return np.hypot(*(a[:shared] - b[:shared]).T).mean() < tolerance_px


def choose_path(paths, previous=None):
    """Pick the path to follow from trace_paths' result ([] if there is none).

    Without a previous path: the first (best-scoring). With one (the
    path chosen on the last frame): the candidate lying closest to it, so the
    choice at a fork does not flip from frame to frame.
    """
    if not paths:
        return []
    if not previous:
        return paths[0]
    previous = np.asarray(previous, np.float64)

    def distance(path):
        apart = np.asarray(path, np.float64)[:, None] - previous[None]
        return np.hypot(apart[..., 0], apart[..., 1]).min(axis=1).mean()
    return min(paths, key=distance)


def centerline(mask, blue=None, obstacle=None, previous=None, **settings):
    """Trace the yellow path to follow, from the robot outward.

    Returns a list of (u, v) pixel points ordered near -> far (empty if the
    path is not found). Pixels are in the mask's own resolution.

    mask:     yellow 0/1 mask. blue, obstacle: no-go 0/1 masks (optional).
    previous: the path returned for the last frame, if any (see choose_path).
    settings: passed on to trace_paths.
    """
    return choose_path(trace_paths(mask, blue, obstacle, **settings), previous)


def straight_reach(path, yellow, blue=None, obstacle=None, min_support=.5,
                   horizon_frac=.28, half_width_px=40., blue_margin_px=6.,
                   obstacle_margin_px=15., max_gap_px=120., robot_half_width_px=0.,
                   footprint_skip_frac=.1, **_unused):
    """Farthest point of a path the robot can head for in a straight line.

    The "taut string" used by 'taut' steering: instead of following every
    bend of the path, aim at its farthest point such that the straight line
    from the path's first point (the robot) to it stays on the road and off
    no-go. On a straight road this aims far ahead; round a bend the line
    hugs the inside. A straight line on the floor is a straight line in the
    image, so this works in pixels without camera calibration.

    On the road means yellow support (see floor_fields) of at least
    min_support: higher keeps the line further from the road's edges. With
    robot_half_width_px > 0, support is measured across the robot's whole
    width (footprint_support), so min_support is the fraction of the span
    between the treads that must be on yellow. No-go (blue, cars and their
    margins, widened by the robot's half-width) is never crossed. The stretch before the
    path first reaches allowed ground (e.g. the robot starting off the road)
    is not checked.

    path: trace_paths/choose_path output, (u, v) near -> far.
    Other arguments: as for floor_fields (extra settings are ignored).
    Returns (index into path, (u, v)) or None if no point beyond the start
    can be reached.
    """
    if path is None or len(path) < 2:
        return None
    _, support, nogo, _ = floor_fields(
        yellow, blue, obstacle, horizon_frac, half_width_px, blue_margin_px,
        obstacle_margin_px, max_gap_px=max_gap_px,
        robot_half_width_px=robot_half_width_px, footprint_skip_frac=footprint_skip_frac)
    if robot_half_width_px > 0:
        # Judge "on the road" over the robot's whole width, tread to tread.
        support = footprint_support(yellow, horizon_frac, robot_half_width_px,
                                    footprint_skip_frac)
    allowed = (support >= min_support) & ~nogo
    h, w = allowed.shape
    pts = np.asarray(path, np.float64)
    cols = np.clip(np.rint(pts[:, 0]).astype(int), 0, w - 1)
    rows = np.clip(np.rint(pts[:, 1]).astype(int), 0, h - 1)
    on = np.flatnonzero(allowed[rows, cols])
    if len(on) == 0:
        return None
    entry_row = pts[on[0], 1]  # only check the line from here up
    su, sv = pts[0]
    for i in range(len(pts) - 1, max(on[0], 1) - 1, -1):
        tu, tv = pts[i]
        n = int(max(abs(tu - su), abs(tv - sv))) + 1
        us = np.rint(np.linspace(su, tu, n)).astype(int)
        vs = np.rint(np.linspace(sv, tv, n)).astype(int)
        check = vs <= entry_row
        us, vs = us[check], vs[check]
        if ((us >= 0) & (us < w) & (vs >= 0) & (vs < h)).all() and allowed[vs, us].all():
            return i, (float(tu), float(tv))
    return None
