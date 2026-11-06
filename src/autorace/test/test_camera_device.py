'''Camera device names (no ROS, no camera): a fake /dev tree with /dev/v4l symlinks.'''
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from autorace.camera.device import aliases, resolve  # noqa: E402
from autorace.camera.list_cameras import video_nodes  # noqa: E402


@pytest.fixture
def dev(tmp_path):
    d = tmp_path / 'dev'
    (d / 'v4l' / 'by-path').mkdir(parents=True)
    (d / 'v4l' / 'by-id').mkdir(parents=True)
    for n in (0, 1, 2, 3, 10):
        (d / ('video%d' % n)).write_text('')
    # after this boot the lane camera (port 1.2) came up as video2, the front camera (port 1.3) as video0
    links = {
        'by-path/platform-xhci-hcd.0-usb-0:1.2:1.0-video-index0': 'video2',
        'by-path/platform-xhci-hcd.0-usb-0:1.2:1.0-video-index1': 'video3',
        'by-path/platform-xhci-hcd.0-usb-0:1.3:1.0-video-index0': 'video0',
        'by-id/usb-Cam_Wide_A1-video-index0': 'video2',
    }
    for name, target in links.items():
        os.symlink(os.path.join('..', '..', target), str(d / 'v4l' / name))
    return d


def test_stable_name_follows_the_camera_not_the_number(dev):
    lane = str(dev / 'v4l/by-path/platform-xhci-hcd.0-usb-0:1.2:1.0-video-index0')
    front = str(dev / 'v4l/by-path/platform-xhci-hcd.0-usb-0:1.3:1.0-video-index0')
    assert resolve(lane, str(dev))[0] == 2
    assert resolve(front, str(dev))[0] == 0
    assert resolve(str(dev / 'v4l/by-id/usb-Cam_Wide_A1-video-index0'), str(dev))[0] == 2
    assert resolve(str(dev / 'video10'), str(dev))[0] == 10


def test_index_still_accepted_but_flagged():
    i, text = resolve('2')
    assert i == 2 and 'not stable' in text


def test_missing_device_lists_what_is_there(dev):
    with pytest.raises(ValueError) as e:
        resolve(str(dev / 'v4l/by-path/platform-xhci-hcd.0-usb-0:1.4:1.0-video-index0'), str(dev))
    assert 'usb-0:1.2:1.0-video-index0' in str(e.value)
    with pytest.raises(ValueError):
        resolve('lane_cam')


def test_aliases_and_nodes(dev):
    a = aliases(str(dev))
    assert len(a[os.path.realpath(str(dev / 'video2'))]) == 2          # by-path and by-id
    assert [os.path.basename(p) for p in video_nodes(str(dev))] == ['video0', 'video1', 'video2', 'video3',
                                                                    'video10']
