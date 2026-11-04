'''
Rule-based camera checks on the front camera (/camera2), no ROS. mission_model has no
traffic-light or barrier class, so these two are colour rules.

ROIs are fractions of the image (x0, y0, x1, y1). Defaults are guesses: tune them on the debug
image (/mission/debug/compressed) in Gazebo and on the real robot.
'''
import cv2
import numpy as np

LIGHT_DEFAULTS = {
    'roi': [0.35, 0.0, 1.0, 0.75],   # traffic light stands right of the start lane
    'v_min': 170,                    # lit lens is bright; unlit lenses are dark
    's_min': 120,
    'min_area': 0.0004,              # blob area / image area
    'max_area': 0.03,
    'min_fill': 0.45,                # blob area / bounding-box area (round lens ~0.78)
    'max_aspect': 2.5,
}

BAR_DEFAULTS = {
    'roi': [0.0, 0.15, 1.0, 0.85],
    'v_min': 70,
    's_min': 110,
    'min_width': 0.35,               # red columns across >= 35 % of the image width -> lowered bar
    'min_px': 0.004,                 # red pixel count / ROI area
}

# OpenCV hue 0..179
HUES = {'red': ((0, 10), (170, 180)), 'yellow': ((15, 35),), 'green': ((45, 95),)}


def _roi(img, roi):
    h, w = img.shape[:2]
    x0, y0, x1, y1 = [int(round(f * s)) for f, s in zip(roi, (w, h, w, h))]
    return img[y0:y1, x0:x1], (x0, y0, x1, y1)


def _hue_mask(hsv, colour, s_min, v_min):
    m = np.zeros(hsv.shape[:2], bool)
    for lo, hi in HUES[colour]:
        m |= (hsv[..., 0] >= lo) & (hsv[..., 0] < hi)
    return m & (hsv[..., 1] >= s_min) & (hsv[..., 2] >= v_min)


def classify_light(img, p=None):
    '''Returns (colour | None, info). colour is the lit lens: red, yellow or green.'''
    p = dict(LIGHT_DEFAULTS, **(p or {}))
    sub, box = _roi(img, p['roi'])
    if sub.size == 0:
        return None, {'box': box}
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    area_img = float(img.shape[0] * img.shape[1])
    best, best_area, blobs = None, 0.0, []
    for colour in ('red', 'yellow', 'green'):
        m = _hue_mask(hsv, colour, p['s_min'], p['v_min']).astype(np.uint8)
        n, _, stats, _ = cv2.connectedComponentsWithStats(m, 8)
        for i in range(1, n):
            x, y, w, h, a = stats[i]
            frac = a / area_img
            if not (p['min_area'] <= frac <= p['max_area']):
                continue
            if a / float(w * h) < p['min_fill'] or max(w, h) / float(min(w, h)) > p['max_aspect']:
                continue
            blobs.append((colour, int(x + box[0]), int(y + box[1]), int(w), int(h)))
            if a > best_area:
                best, best_area = colour, a
    return best, {'box': box, 'blobs': blobs}


def bar_state(img, p=None):
    '''Returns ('down' | 'up', info). down = red spans a wide part of the image (lowered arm).'''
    p = dict(BAR_DEFAULTS, **(p or {}))
    sub, box = _roi(img, p['roi'])
    if sub.size == 0:
        return 'up', {'box': box, 'width': 0.0}
    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)
    m = _hue_mask(hsv, 'red', p['s_min'], p['v_min'])
    if m.sum() < p['min_px'] * m.size:
        return 'up', {'box': box, 'width': 0.0}
    # widest horizontal span of red in any band of 10 % ROI height (a raised, vertical arm is narrow).
    # The arm is red/white striped, so red covers only part of the span: require >= 25 % coverage
    # so that two separate red things (e.g. the STOP sign and something else) do not count as one span.
    h = m.shape[0]
    band = max(2, h // 10)
    width = 0.0
    for y0 in range(0, h - band + 1, max(1, band // 2)):
        cols = np.flatnonzero(m[y0:y0 + band].any(axis=0))
        if cols.size < 2:
            continue
        span = cols[-1] - cols[0] + 1
        if cols.size >= 0.25 * span:
            width = max(width, span / float(m.shape[1]))
    return ('down' if width >= p['min_width'] else 'up'), {'box': box, 'width': width}


def draw(img, light=None, light_info=None, bar=None, bar_info=None, text=''):
    out = img.copy()
    if light_info:
        x0, y0, x1, y1 = light_info['box']
        cv2.rectangle(out, (x0, y0), (x1, y1), (255, 255, 0), 1)
        for colour, x, y, w, h in light_info.get('blobs', []):
            cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.putText(out, 'light: %s' % light, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if bar_info:
        x0, y0, x1, y1 = bar_info['box']
        cv2.rectangle(out, (x0, y0), (x1, y1), (0, 128, 255), 1)
        cv2.putText(out, 'bar: %s w=%.2f' % (bar, bar_info['width']), (8, 44),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if text:
        cv2.putText(out, text, (8, out.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    return out
