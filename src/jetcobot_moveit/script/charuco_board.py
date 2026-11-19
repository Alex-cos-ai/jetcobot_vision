#!/usr/bin/env python3
"""ChArUco geometry and pose estimation, independent of ROS.

The supplied PDF uses DICT_5X5_100, IDs 0..16, 5x7 squares of 28 mm,
and 21 mm markers. Coordinates: outer top-left corner, x right, y down,
z into the printed board (OpenCV >= 4.8 convention). Lengths are metres.
"""

from dataclasses import asdict, dataclass
import hashlib
import json
import math

import cv2
import numpy as np


@dataclass(frozen=True)
class BoardSpec:
    squares_x: int = 5
    squares_y: int = 7
    square_length_m: float = 0.028
    marker_length_m: float = 0.021
    dictionary: str = 'DICT_5X5_100'

    def __post_init__(self):
        if self.squares_x < 3 or self.squares_y < 3:
            raise ValueError('标定板每个方向至少需要 3 格')
        if not (math.isfinite(self.square_length_m)
                and 0 < self.marker_length_m < self.square_length_m):
            raise ValueError('尺寸必须满足 0 < marker_length_m < square_length_m')
        if not self.dictionary.startswith('DICT_') or not hasattr(cv2.aruco, self.dictionary):
            raise ValueError(f'未知 ArUco 字典: {self.dictionary}')

    def metadata(self):
        return dict(asdict(self), coordinate_convention='top_left_x_right_y_down_z_into_board')

    def signature(self):
        return hashlib.sha256(json.dumps(
            self.metadata(), sort_keys=True).encode('utf-8')).hexdigest()

    def create_board(self):
        if not hasattr(cv2.aruco, 'CharucoDetector'):
            raise RuntimeError('需要包含 aruco.CharucoDetector 的 OpenCV contrib >= 4.8')
        board = cv2.aruco.CharucoBoard(
            (self.squares_x, self.squares_y), self.square_length_m,
            self.marker_length_m,
            cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary)))
        # Explicitly use the current pattern layout, including even-row boards.
        board.setLegacyPattern(False)
        return board


class CharucoPoseEstimator:
    def __init__(self, spec, min_markers=4, min_corners=8,
                 min_board_coverage=0.20, min_image_coverage=0.01,
                 max_reprojection_rms_px=0.8, ransac_threshold_px=1.5,
                 min_inlier_ratio=0.85):
        self.spec = spec
        self.board = spec.create_board()
        self.min_markers = min_markers
        self.min_corners = min_corners
        self.min_board_coverage = min_board_coverage
        self.min_image_coverage = min_image_coverage
        self.max_reprojection_rms_px = max_reprojection_rms_px
        self.ransac_threshold_px = ransac_threshold_px
        self.min_inlier_ratio = min_inlier_ratio
        if not (min_markers >= 2 and 4 <= min_corners <= len(self.board.getChessboardCorners())
                and 0 < min_board_coverage <= 1 and 0 < min_image_coverage <= 1
                and math.isfinite(max_reprojection_rms_px) and max_reprojection_rms_px > 0
                and math.isfinite(ransac_threshold_px) and ransac_threshold_px > 0
                and 0 < min_inlier_ratio <= 1):
            raise ValueError('无效的角点数量、覆盖率或重投影误差阈值')
        parameters = cv2.aruco.DetectorParameters()
        # ChArUco performs its own subpixel chessboard refinement. Refining the
        # marker edges close to black squares can bias their initial positions.
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self.detector = cv2.aruco.CharucoDetector(
            self.board, cv2.aruco.CharucoParameters(), parameters)

    def detect_corners(self, gray, camera_matrix, dist_coeffs):
        parameters = cv2.aruco.CharucoParameters()
        parameters.cameraMatrix = camera_matrix
        parameters.distCoeffs = dist_coeffs
        parameters.minMarkers = 2
        self.detector.setCharucoParameters(parameters)
        return self.detector.detectBoard(gray)

    def estimate(self, corners, ids, marker_ids, camera_matrix, dist_coeffs, image_size):
        if marker_ids is None or len(marker_ids) < self.min_markers:
            raise ValueError(f'整板 tag 数不足，需要 >= {self.min_markers}')
        if ids is None or len(ids) < self.min_corners:
            raise ValueError(f'ChArUco 角点不足，需要 >= {self.min_corners}')
        ids = np.asarray(ids).reshape(-1)
        all_objects = self.board.getChessboardCorners()
        if (len(np.unique(ids)) != len(ids) or ids.min() < 0 or ids.max() >= len(all_objects)):
            raise ValueError('角点 ID 无效或重复')
        objects = np.asarray(all_objects[ids], dtype=np.float64)
        pixels = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
        if len(pixels) != len(objects) or not np.isfinite(pixels).all():
            raise ValueError('角点像素无效')
        ok, _, _, inliers = cv2.solvePnPRansac(
            objects, pixels, camera_matrix, dist_coeffs,
            iterationsCount=100, reprojectionError=self.ransac_threshold_px,
            confidence=0.999, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok or inliers is None:
            raise ValueError('整板 PnP 求解失败')
        indices = inliers.reshape(-1)
        if (len(indices) < self.min_corners
                or len(indices) / len(ids) < self.min_inlier_ratio):
            raise ValueError('PnP 内点不足，角点异常比例过高')
        obj, img = objects[indices], pixels[indices]
        board_area = (self.spec.squares_x * self.spec.squares_y
                      * self.spec.square_length_m ** 2)
        board_coverage = cv2.contourArea(cv2.convexHull(
            obj[:, :2].astype(np.float32))) / board_area
        image_coverage = cv2.contourArea(cv2.convexHull(
            img.astype(np.float32))) / (image_size[0] * image_size[1])
        if board_coverage < self.min_board_coverage:
            raise ValueError(f'角点在板上过于集中，覆盖率 {board_coverage:.1%}')
        if image_coverage < self.min_image_coverage:
            raise ValueError(f'标定板在画面中过小，覆盖率 {image_coverage:.1%}')

        # A planar board has two possible IPPE poses. Refine and score both;
        # never use IPPE_SQUARE, which assumes four single-marker corners.
        solutions = cv2.solvePnPGeneric(
            obj, img, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE)
        candidates = []
        for rotation, translation in zip(solutions[1], solutions[2]):
            rotation, translation = cv2.solvePnPRefineLM(
                obj, img, camera_matrix, dist_coeffs, rotation.copy(), translation.copy())
            matrix = cv2.Rodrigues(rotation)[0]
            if np.any((obj @ matrix.T + translation.reshape(3))[:, 2] <= 0):
                continue
            projected = cv2.projectPoints(
                obj, rotation, translation, camera_matrix, dist_coeffs)[0].reshape(-1, 2)
            errors = np.linalg.norm(projected - img, axis=1)
            candidates.append((float(np.sqrt(np.mean(errors ** 2))),
                               rotation, translation, errors))
        if not candidates:
            raise ValueError('整板位姿位于相机后方或求解失败')
        rms, rvec, tvec, errors = min(candidates, key=lambda item: item[0])
        if rms > self.max_reprojection_rms_px:
            raise ValueError(f'重投影 RMS 过大: {rms:.3f} px')
        return {
            'rvec': rvec, 'tvec': tvec,
            'R_target2cam': cv2.Rodrigues(rvec)[0],
            'quality': {
                'marker_count': len(marker_ids), 'corner_count': len(ids),
                'inlier_count': len(indices), 'inlier_corner_ids': ids[indices].tolist(),
                'reprojection_rms_px': rms,
                'reprojection_max_px': float(np.max(errors)),
                'board_coverage': board_coverage, 'image_coverage': image_coverage,
            },
        }
