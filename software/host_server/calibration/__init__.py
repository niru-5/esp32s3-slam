"""Camera calibration + ISP tuning tools (host side of firmware cam_calib.c).

Kept separate from the streaming server: this package needs numpy/OpenCV
(``software/requirements-calib.txt``), while ``host_server`` proper is
stdlib-only.
"""
