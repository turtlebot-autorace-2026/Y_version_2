'''
Which /dev/video device a camera node opens.

The senior's publishers opened cv2.VideoCapture(0) (lane) and cv2.VideoCapture(2) (front). Those
numbers are handed out by USB enumeration order: after a reboot or a re-plug the two cameras can
swap, and then the lane node sees the front camera. A name under /dev/v4l/ does not move:

  /dev/v4l/by-path/...-usb-0:1.2:1.0-video-index0   the USB port (use this when both cameras are the same model)
  /dev/v4l/by-id/usb-<maker>_<model>_<serial>-video-index0   the camera itself (different models)

`ros2 run autorace list_cameras` prints these names with a snapshot of each camera.
No ROS here (unit-testable).
'''
import glob
import os
import re

V4L_DIRS = ('/dev/v4l/by-path', '/dev/v4l/by-id')


def aliases(dev_dir='/dev'):
    '''{'/dev/videoN': [stable names pointing to it]}'''
    out = {}
    for d in V4L_DIRS:
        d = dev_dir.rstrip('/') + d[len('/dev'):]
        for link in sorted(glob.glob(os.path.join(d, '*'))):
            out.setdefault(os.path.realpath(link), []).append(link)
    return out


def resolve(spec, dev_dir='/dev'):
    '''spec -> (V4L2 index, text for the log).

    spec: '0' / '2' (index, the old behaviour, may swap), '/dev/video2', or a /dev/v4l/... name.
    Raises ValueError with the available stable names when the device is not there.'''
    spec = str(spec).strip()
    if re.fullmatch(r'\d+', spec):
        return int(spec), 'index %s (not stable: may swap after reboot / re-plug)' % spec
    if not spec.startswith('/'):
        raise ValueError('camera device %r: use a /dev/v4l/by-path or /dev/v4l/by-id name, '
                         '/dev/videoN or an index' % spec)
    if not os.path.exists(spec):
        names = sorted(n for v in aliases(dev_dir).values() for n in v)
        raise ValueError('camera device %s not found (unplugged, or another USB port?). Present: %s'
                         % (spec, ', '.join(names) or 'no /dev/v4l names at all'))
    real = os.path.realpath(spec)
    m = re.fullmatch(r'.*/video(\d+)', real)
    if not m:
        raise ValueError('camera device %s -> %s is not a /dev/videoN node' % (spec, real))
    return int(m.group(1)), '%s -> %s' % (spec, real)


def open_capture(spec, cv2):
    '''cv2.VideoCapture on the V4L2 backend for spec. Returns (cap, text for the log).'''
    index, text = resolve(spec)
    return cv2.VideoCapture(index, cv2.CAP_V4L2), text
