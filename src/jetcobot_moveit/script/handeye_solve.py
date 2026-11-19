#!/usr/bin/env python3
"""Solve eye-in-hand calibration without connecting to or moving the robot.

T_base_target = T_base_gripper @ T_gripper_camera @ T_camera_target.
The output T_gripper_camera maps camera optical coordinates into gripper
coordinates; it is the parent=gripper, child=camera ROS TF transform.
"""

import argparse
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np


METHODS = {
    name: getattr(cv2, 'CALIB_HAND_EYE_' + name)
    for name in ('TSAI', 'PARK', 'HORAUD', 'ANDREFF', 'DANIILIDIS')
}


def as_transform(rotation, translation):
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    if rotation.shape != (3, 3) or translation.size != 3:
        raise ValueError('旋转必须为 3x3，平移必须包含 3 个值')
    if not (np.isfinite(rotation).all() and np.isfinite(translation).all()):
        raise ValueError('样本包含 NaN 或无穷值')
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5)):
        raise ValueError('样本旋转矩阵不是有效的 SO(3) 旋转')
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation.reshape(3)
    return transform


def target_metadata(data):
    """Keep board identity separate from the single-marker compatibility path."""
    if data.get('target_type') == 'charuco':
        from charuco_board import BoardSpec
        board = data['board']
        spec = BoardSpec(**{key: board[key] for key in (
            'squares_x', 'squares_y', 'square_length_m', 'marker_length_m', 'dictionary')})
        if board != spec.metadata():
            raise ValueError('ChArUco 坐标约定或板参数不一致')
        if not data.get('camera_model'):
            raise ValueError('ChArUco 数据缺少相机内参')
        if any(s.get('target_signature') != spec.signature() for s in data.get('samples', [])):
            raise ValueError('样本的 ChArUco 板签名不一致')
        return {'target_type': 'charuco', 'board': board, 'camera_model': data['camera_model']}
    return {'target_type': 'single_marker', 'tag_id': data['samples'][0].get('tag_id'),
            'marker_length_m': data['marker_length_m']}


def load_samples(path, min_samples=6):
    data = json.loads(path.read_text(encoding='utf-8'))
    samples = data['samples']
    if len(samples) < min_samples:
        raise ValueError(f'至少需要 {min_samples} 组有效配对样本')
    target_metadata(data)
    gripper, target = [], []
    for index, sample in enumerate(samples, 1):
        try:
            for key in ('base_frame', 'gripper_frame', 'camera_frame', 'tag_id', 'target_signature'):
                if sample.get(key) != samples[0].get(key):
                    raise ValueError(f'样本的 {key} 不一致')
            gripper.append(as_transform(
                sample['R_gripper2base'], sample['t_gripper2base']))
            target.append(as_transform(
                sample['R_target2cam'], sample['t_target2cam']))
        except (KeyError, ValueError) as exc:
            raise ValueError(f'第 {index} 组样本无效: {exc}') from exc
    return data, np.asarray(gripper), np.asarray(target)


def solve(gripper, target, method):
    rotation, translation = cv2.calibrateHandEye(
        list(gripper[:, :3, :3]), list(gripper[:, :3, 3]),
        list(target[:, :3, :3]), list(target[:, :3, 3]),
        method=METHODS[method],
    )
    return as_transform(rotation, translation)


def average_pose(poses):
    u, _, vt = np.linalg.svd(np.mean(poses[:, :3, :3], axis=0))
    rotation = u @ np.diag([1, 1, np.linalg.det(u @ vt)]) @ vt
    return as_transform(rotation, poses[:, :3, 3].mean(axis=0))


def evaluate(gripper, target, handeye, reference=None):
    base_targets = gripper @ handeye @ target
    if reference is None:
        reference = average_pose(base_targets)
    translation_mm = np.linalg.norm(
        base_targets[:, :3, 3] - reference[:3, 3], axis=1) * 1000.0
    relative_rotation = reference[:3, :3].T @ base_targets[:, :3, :3]
    cosine = (np.trace(relative_rotation, axis1=1, axis2=2) - 1.0) / 2.0
    rotation_deg = np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0)))
    return {
        'translation_rms_mm': float(np.sqrt(np.mean(translation_mm ** 2))),
        'translation_max_mm': float(np.max(translation_mm)),
        'rotation_rms_deg': float(np.sqrt(np.mean(rotation_deg ** 2))),
        'rotation_max_deg': float(np.max(rotation_deg)),
        'translation_error_mm': translation_mm.tolist(),
        'rotation_error_deg': rotation_deg.tolist(),
        'reference_T_base_target': reference.tolist(),
    }


def quaternion_xyzw(rotation):
    vector = cv2.Rodrigues(rotation)[0].reshape(3)
    angle = float(np.linalg.norm(vector))
    xyz = np.zeros(3) if angle < 1e-12 else vector * np.sin(angle / 2) / angle
    return [*xyz.tolist(), float(np.cos(angle / 2))]


def sample_digest(gripper, target):
    return hashlib.sha256(gripper.tobytes() + target.tobytes()).hexdigest()


def validate(data, gripper, target, calibration, reference_report=None):
    """Check fresh poses against the stored calibration without fitting."""
    first = data['samples'][0]
    for key, result_key in (
            ('base_frame', 'base_frame'), ('gripper_frame', 'parent_frame'),
            ('camera_frame', 'child_frame')):
        if first.get(key) != calibration[result_key]:
            raise ValueError(f'验证样本的 {key} 与标定结果不一致')
    metadata = target_metadata(data)
    if metadata['target_type'] != calibration.get('target_type', 'single_marker'):
        raise ValueError('验证样本与标定结果的目标类型不一致')
    if metadata['target_type'] == 'charuco':
        for key in ('board', 'camera_model'):
            if metadata[key] != calibration.get(key):
                raise ValueError(f'验证样本的 {key} 与标定结果不一致')
    else:
        if first.get('tag_id') != calibration['tag_id']:
            raise ValueError('验证样本的 tag_id 与标定结果不一致')
        if not np.isclose(data['marker_length_m'], calibration['marker_length_m']):
            raise ValueError('验证样本的标志尺寸与标定数据不一致')
    if sample_digest(gripper, target) == calibration['sample_digest_sha256']:
        raise ValueError('验证数据与标定数据相同，请采集新的姿态')
    original_stamps = {json.dumps(s, sort_keys=True)
                       for s in calibration['sample_stamps'] if s is not None}
    if any(json.dumps(s.get('stamp'), sort_keys=True) in original_stamps
           for s in data['samples']):
        raise ValueError('验证数据包含已用于拟合的时间戳，请只使用新采集样本')
    matrix = np.asarray(calibration['T_gripper_camera'], dtype=float)
    handeye = as_transform(matrix[:3, :3], matrix[:3, 3])
    reference = np.asarray(calibration['methods'][calibration['method']][
        'all_sample_scatter']['reference_T_base_target'], dtype=float)
    reference_source = 'original_calibration_batch'
    if reference_report is not None:
        if reference_report.get('target_type', 'single_marker') != metadata['target_type']:
            raise ValueError('参考报告的目标类型不一致')
        if metadata['target_type'] == 'charuco' and any(
                reference_report.get(key) != metadata[key] for key in ('board', 'camera_model')):
            raise ValueError('参考报告的标定板或内参不一致')
        if reference_report['method'] != calibration['method']:
            raise ValueError('参考报告使用的标定方法不一致')
        if 'T_gripper_camera' in reference_report:
            if not np.allclose(reference_report['T_gripper_camera'], matrix,
                               atol=1e-10, rtol=0):
                raise ValueError('参考报告使用的手眼变换不一致')
        # Earlier reports lack their sample stamps. New reports preserve them
        # so reference samples cannot accidentally be reused as validation.
        reference_stamps = {json.dumps(s, sort_keys=True)
                            for s in reference_report.get('sample_stamps', [])
                            if s is not None}
        if any(json.dumps(s.get('stamp'), sort_keys=True) in reference_stamps
               for s in data['samples']):
            raise ValueError('新样本与参考报告样本时间戳重叠')
        reference = np.asarray(reference_report['batch_scatter'][
            'reference_T_base_target'], dtype=float)
        reference_source = 'previous_validation_batch'
    reference = as_transform(reference[:3, :3], reference[:3, 3])
    # Recenter the fixed-target reference only for a separate diagnostic.
    # Keep the original-reference error intact; neither calculation refits X.
    batch_reference = average_pose(gripper @ handeye @ target)
    shift_mm = (batch_reference[:3, 3] - reference[:3, 3]) * 1000.0
    rotation_shift_deg = np.degrees(np.linalg.norm(cv2.Rodrigues(
        reference[:3, :3].T @ batch_reference[:3, :3])[0]))
    return {
        **metadata,
        'mode': 'independent_validation_without_refitting',
        'sample_count': len(gripper),
        'method': calibration['method'],
        'T_gripper_camera': handeye.tolist(),
        'sample_stamps': [s.get('stamp') for s in data['samples']],
        'sample_digest_sha256': sample_digest(gripper, target),
        'reference_source': reference_source,
        'metrics': evaluate(gripper, target, handeye, reference),
        'batch_scatter': evaluate(gripper, target, handeye, batch_reference),
        'reference_shift': {
            'translation_xyz_mm': shift_mm.tolist(),
            'translation_norm_mm': float(np.linalg.norm(shift_mm)),
            'rotation_deg': float(rotation_shift_deg),
        },
        'validation_note': (
            '使用已保存手眼变换和指定参考报告或原标定的 Tag 基座位姿，没有重新拟合。'
            '必须保持同一 Tag 固定不动、相机安装和内参不变；'
            '相对原参考的误差要求 Tag 相对基座位置不变。'
            '批内离散仅以新样本均值作参考，未重新拟合手眼变换，不能消除或验证整体偏移。'
            '这些数值不能替代外部测量得到的绝对定位精度。'
        ),
    }


def calibrate(data, gripper, target, selected_method):
    # Hold out every fifth sample. The fitted transform and the fixed-target
    # reference for this check both use training poses only.
    heldout = np.arange(len(gripper)) % 5 == 4
    training = ~heldout
    methods = {}
    transforms = {}
    for method in METHODS:
        try:
            handeye = solve(gripper, target, method)
            train_handeye = solve(gripper[training], target[training], method)
            reference = average_pose(
                gripper[training] @ train_handeye @ target[training])
            methods[method] = {
                'T_gripper_camera': handeye.tolist(),
                'all_sample_scatter': evaluate(gripper, target, handeye),
                'heldout': evaluate(
                    gripper[heldout], target[heldout], train_handeye, reference),
            }
            transforms[method] = handeye
        except (cv2.error, ValueError, np.linalg.LinAlgError) as exc:
            methods[method] = {'error': str(exc)}
    if selected_method not in transforms:
        raise ValueError(f'{selected_method} 求解失败: {methods[selected_method]}')

    relative_rotations = gripper[0, :3, :3].T @ gripper[1:, :3, :3]
    vectors = np.array([cv2.Rodrigues(r)[0].reshape(3) for r in relative_rotations])
    singular_values = np.linalg.svd(vectors, compute_uv=False)
    if singular_values[1] < 1e-3:
        raise ValueError('旋转变化退化，需绕至少两个不同方向旋转末端后重新采集')

    handeye = transforms[selected_method]
    first = data['samples'][0]
    metadata = target_metadata(data)
    result = {
        'sample_count': len(gripper),
        'sample_digest_sha256': sample_digest(gripper, target),
        'sample_stamps': [s.get('stamp') for s in data['samples']],
        **metadata,
        'method': selected_method,
        'parent_frame': first.get('gripper_frame', data.get('gripper_frame')),
        'child_frame': first.get('camera_frame', 'camera_optical_frame'),
        'base_frame': first.get('base_frame', data.get('base_frame')),
        'convention': 'p_gripper = T_gripper_camera @ p_camera_optical',
        'T_gripper_camera': handeye.tolist(),
        'translation_m': handeye[:3, 3].tolist(),
        'quaternion_xyzw': quaternion_xyzw(handeye[:3, :3]),
        'rotation_excitation_singular_values_rad': singular_values.tolist(),
        'heldout_sample_numbers': (np.flatnonzero(heldout) + 1).tolist(),
        'methods': methods,
        'validation_note': (
            '固定 Tag 的基座位姿离散程度；假设 Tag 在采集期间固定不动。'
            '留出验证来自同一批采集，不能替代新姿态验证或实际定位精度测试。'
            '全部输入样本参与最终拟合，未自动剔除样本；未发布 TF 或执行运动。'
        ),
    }
    if metadata['target_type'] == 'charuco':
        result['T_base_board'] = methods[selected_method][
            'all_sample_scatter']['reference_T_base_target']
    return result


def compare_calibration(data, gripper, target, calibration):
    """Evaluate an existing X on this batch, without fitting X or sharing origins.

    The target may differ from the baseline's calibration target. Scatter uses
    this batch's mean board pose, so an old single-tag X can be compared too.
    """
    first = data['samples'][0]
    for key, result_key in (('base_frame', 'base_frame'), ('gripper_frame', 'parent_frame'),
                            ('camera_frame', 'child_frame')):
        if first.get(key) != calibration[result_key]:
            raise ValueError(f'对照手眼结果的 {result_key} 不一致')
    matrix = np.asarray(calibration['T_gripper_camera'], dtype=float)
    handeye = as_transform(matrix[:3, :3], matrix[:3, 3])
    return {
        'T_gripper_camera': handeye.tolist(),
        'batch_scatter': evaluate(gripper, target, handeye),
        'note': '仅比较同批新观测下的基座棋盘位姿离散；不重新拟合手眼变换。'
                '各自以本批均值为参考，不能验证整体偏移或绝对精度。',
    }


def main(default_input='/home/jetson/handeye_samples.json',
         default_result='/home/jetson/handeye_result.json',
         default_validation='/home/jetson/handeye_validation.json',
         required_target_type=None):
    parser = argparse.ArgumentParser(description='手眼标定离线求解与固定 Tag 一致性检查')
    parser.add_argument('--input', type=Path, default=Path(default_input))
    parser.add_argument('--output', type=Path)
    parser.add_argument('--calibration', type=Path,
                        help='验证模式：读取此标定结果，检查 --input 中的新样本，不重新拟合')
    parser.add_argument('--reference-report', type=Path,
                        help='底座转动后，使用之前一批验证报告中的 Tag 均值作参考')
    parser.add_argument('--method', choices=METHODS, default='PARK')
    parser.add_argument('--compare-calibration', type=Path,
                        help='在同批观测上额外评估旧手眼变换的棋盘位姿离散，允许旧单 tag 标定')
    args = parser.parse_args()
    if args.reference_report and not args.calibration:
        parser.error('--reference-report 需要同时指定 --calibration')
    try:
        data, gripper, target = load_samples(
            args.input, min_samples=1 if args.calibration else 6)
        if required_target_type and data.get('target_type') != required_target_type:
            raise ValueError(f'此入口需要 {required_target_type} 样本')
        if args.calibration:
            calibration = json.loads(args.calibration.read_text(encoding='utf-8'))
            reference_report = None
            if args.reference_report:
                reference_report = json.loads(
                    args.reference_report.read_text(encoding='utf-8'))
                if Path(reference_report['calibration_file']).resolve() != args.calibration.resolve():
                    raise ValueError('参考报告的标定文件路径不一致')
            result = validate(data, gripper, target, calibration, reference_report)
            result['calibration_file'] = str(args.calibration.resolve())
            if args.reference_report:
                result['reference_report_file'] = str(args.reference_report.resolve())
        else:
            result = calibrate(data, gripper, target, args.method)
        if args.compare_calibration:
            baseline = json.loads(args.compare_calibration.read_text(encoding='utf-8'))
            result['comparison'] = compare_calibration(data, gripper, target, baseline)
            result['comparison']['calibration_file'] = str(args.compare_calibration.resolve())
        if args.output is None:
            args.output = Path(default_validation if args.calibration else default_result)
        protected = [args.input.resolve()]
        if args.calibration:
            protected.append(args.calibration.resolve())
        if args.reference_report:
            protected.append(args.reference_report.resolve())
        if args.compare_calibration:
            protected.append(args.compare_calibration.resolve())
        if args.output.resolve() in protected:
            raise ValueError('输出路径不能覆盖输入样本或已有标定文件')
        result['input_file'] = str(args.input.resolve())
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n',
            encoding='utf-8',
        )
    except (OSError, ValueError, KeyError, cv2.error, np.linalg.LinAlgError) as exc:
        parser.exit(1, f'求解失败: {exc}\n')

    if 'comparison' in result:
        metrics = result['comparison']['batch_scatter']
        print(f'对照旧手眼（同批观测）位置 RMS={metrics["translation_rms_mm"]:.2f} mm，'
              f'最大={metrics["translation_max_mm"]:.2f} mm，'
              f'旋转 RMS={metrics["rotation_rms_deg"]:.2f} deg')
        print(result['comparison']['note'])

    if args.calibration:
        metrics = result['metrics']
        print(f'新样本数: {result["sample_count"]}，未重新拟合')
        print(f'相对参考 Tag 基座位姿：位置 RMS={metrics["translation_rms_mm"]:.2f} mm，'
              f'最大={metrics["translation_max_mm"]:.2f} mm')
        print(f'旋转 RMS={metrics["rotation_rms_deg"]:.2f} deg')
        shift = result['reference_shift']
        scatter = result['batch_scatter']
        print(f'新旧 Tag 均值偏移：{shift["translation_norm_mm"]:.2f} mm，'
              f'{shift["rotation_deg"]:.2f} deg')
        print(f'新样本批内离散：位置 RMS={scatter["translation_rms_mm"]:.2f} mm，'
              f'最大={scatter["translation_max_mm"]:.2f} mm，'
              f'旋转 RMS={scatter["rotation_rms_deg"]:.2f} deg')
        print(result['validation_note'])
        print(f'验证结果已保存: {args.output}')
        return

    print(f'样本数: {result["sample_count"]}（全部保留）')
    print('方法          全样本位置RMS(mm)  留出位置RMS(mm)  全样本旋转RMS(deg)')
    for name, item in result['methods'].items():
        if 'error' in item:
            print(f'{name:12} 失败: {item["error"]}')
            continue
        full, hold = item['all_sample_scatter'], item['heldout']
        print(f'{name:12} {full["translation_rms_mm"]:17.3f} '
              f'{hold["translation_rms_mm"]:17.3f} {full["rotation_rms_deg"]:20.3f}')
    print(f'\n选择方法: {args.method}')
    print(f'T_gripper_camera（{result["parent_frame"]} <- {result["child_frame"]}）:')
    print(np.array2string(np.array(result['T_gripper_camera']), precision=8))
    print('平移(m):', result['translation_m'])
    print('四元数(x,y,z,w):', result['quaternion_xyzw'])
    if 'T_base_board' in result:
        print(f'棋盘基座位姿均值 T_base_board（{result["base_frame"]} <- 棋盘左上角）:')
        print(np.array2string(np.asarray(result['T_base_board']), precision=8))
    metrics = result['methods'][args.method]['all_sample_scatter']
    print(f'固定 Tag 位置离散: RMS={metrics["translation_rms_mm"]:.2f} mm，'
          f'最大={metrics["translation_max_mm"]:.2f} mm')
    print(result['validation_note'])
    print(f'完整结果已保存: {args.output}')


if __name__ == '__main__':
    main()
