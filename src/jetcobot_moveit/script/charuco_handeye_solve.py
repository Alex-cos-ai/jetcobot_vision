#!/usr/bin/env python3
"""ChArUco-specific defaults for the shared hand-eye solver and validator."""

from handeye_solve import main


if __name__ == '__main__':
    main(default_input='/home/jetson/charuco_handeye_samples.json',
         default_result='/home/jetson/charuco_handeye_result.json',
         default_validation='/home/jetson/charuco_handeye_validation.json',
         required_target_type='charuco')
