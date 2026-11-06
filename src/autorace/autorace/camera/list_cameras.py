'''
Which camera is which, on the robot (no ROS needed, run with robot.launch stopped):

    ros2 run autorace list_cameras            # snapshots in ~/autorace_cam_check/
    ros2 run autorace list_cameras --no-snap

For every /dev/videoN: its stable names (/dev/v4l/by-path, /dev/v4l/by-id), the camera name, whether
it gives frames (each USB camera also has a metadata node without frames) and the frame size at a
1280x720 MJPG request (the lane publisher requires 1280x720). One snapshot per camera is saved:
the lane camera looks down at the road just ahead, the front camera looks forward. Then put the
names into config/camera_params.yaml.
'''
import argparse
import os
import re
import sys

from autorace.camera.device import aliases


def video_nodes(dev_dir='/dev'):
    nodes = []
    for name in os.listdir(dev_dir):
        m = re.fullmatch(r'video(\d+)', name)
        if m:
            nodes.append((int(m.group(1)), os.path.join(dev_dir, name)))
    return [p for _, p in sorted(nodes)]


def sys_name(node):
    try:
        with open('/sys/class/video4linux/%s/name' % os.path.basename(node)) as f:
            return f.read().strip()
    except OSError:
        return '?'


def probe(node, cv2, snap_dir):
    '''-> (ok, text). Opens the node, asks for 1280x720 MJPG, grabs a few frames, saves one.'''
    index = int(re.search(r'(\d+)$', node).group(1))
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        return False, 'cannot open (metadata node, or busy: is robot.launch still running?)'
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        frame = None
        for _ in range(5):                      # first frames can be dark / empty
            ok, f = cap.read()
            if ok and f is not None:
                frame = f
        if frame is None:
            return False, 'opens but gives no frames (metadata node)'
        h, w = frame.shape[:2]
        text = 'frames %dx%d%s' % (w, h, '' if (w, h) == (1280, 720) else '  (lane publisher needs 1280x720)')
        if snap_dir:
            path = os.path.join(snap_dir, '%s.jpg' % os.path.basename(node))
            cv2.imwrite(path, frame)
            text += '  snapshot %s' % path
        return True, text
    finally:
        cap.release()


def main(argv=None):
    ap = argparse.ArgumentParser(description='List the robot cameras with stable device names.')
    ap.add_argument('--no-snap', action='store_true', help='do not save snapshots')
    ap.add_argument('--out', default=os.path.expanduser('~/autorace_cam_check'), help='snapshot folder')
    a = ap.parse_args(argv)
    import cv2

    snap_dir = None if a.no_snap else a.out
    if snap_dir:
        os.makedirs(snap_dir, exist_ok=True)
    names = aliases()
    nodes = video_nodes()
    if not nodes:
        print('no /dev/video* devices: cameras unplugged?')
        return 1
    capture = []
    for node in nodes:
        ok, text = probe(node, cv2, snap_dir)
        print('%s  [%s]  %s' % (node, sys_name(node), text))
        for n in names.get(node, []):
            print('    ' + n)
        if ok:
            capture.append(node)

    print()
    if not capture:
        print('no device gave frames.')
        return 1
    print('%d camera(s) give frames. Look at the snapshots: the lane camera sees the road right in front.'
          % len(capture))
    by_id = [[n for n in names.get(c, []) if '/by-id/' in n] for c in capture]
    flat = [n for ns in by_id for n in ns]
    same_model = len(flat) != len(set(flat)) or any(not ns for ns in by_id)
    print('Use %s names%s. config/camera_params.yaml:' % (
        'by-path' if same_model else 'by-id (or by-path)',
        ' (by-id missing or identical: same camera model, so the USB port has to tell them apart;'
        ' keep each camera in its port)' if same_model else ''))
    for c in capture:
        pick = [n for n in names.get(c, []) if ('/by-path/' if same_model else '/by-id/') in n] or [c]
        print("    device: '%s'      # %s, %s" % (pick[0], c, sys_name(c)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
