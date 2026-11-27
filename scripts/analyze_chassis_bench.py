#!/usr/bin/env python3
"""Summarize recorded chassis measurements; do not infer a hardware pass without limits."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation


def stamp(message):
    value = message['header']['stamp']
    return value['sec'] + value['nanosec'] * 1e-9


def vector(value):
    return [value[key] for key in ('x', 'y', 'z')]


def statistics(values):
    a = np.asarray(values)
    return {'count': len(a), 'rms': float(np.sqrt(np.mean(a*a))),
            'max_abs': float(np.max(np.abs(a))), 'std': float(np.std(a))} if len(a) else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('record', type=Path)
    args = parser.parse_args()
    manifest = json.loads((args.record / 'manifest.json').read_text())
    odom, timing = [], []
    for line in (args.record / 'samples.jsonl').read_text().splitlines():
        row = json.loads(line)
        msg = row['message']
        if row['topic'] == '/chassis/odom':
            p, q, twist = msg['pose']['pose']['position'], msg['pose']['pose']['orientation'], msg['twist']['twist']
            rotation = Rotation.from_quat([q[k] for k in ('x','y','z','w')])
            odom.append((stamp(msg), np.array(vector(p)), rotation,
                         rotation.apply(vector(twist['linear'])), rotation.apply(vector(twist['angular']))))
        elif row['topic'] == '/sentry/yaw_joint_state':
            timing.append(row['receive_ns'] * 1e-9 - stamp(msg))
    odom.sort(key=lambda row: row[0])
    velocity_residuals, angular_residuals = [], []
    for a, b in zip(odom, odom[1:]):
        dt = b[0] - a[0]
        if 0 < dt < 1.:
            velocity_residuals.extend((b[1] - a[1]) / dt - .5 * (a[3] + b[3]))
            angular_residuals.extend((b[2] * a[2].inv()).as_rotvec() / dt - .5 * (a[4] + b[4]))
    report = {'case': manifest['case'], 'status': 'requires_operator_review',
              'samples': len(odom), 'duplicate_tf_children': manifest['duplicate_tf_children'],
              'joint_receive_minus_measurement_seconds': statistics(timing),
              'timing_note': 'Includes transport and mapping error; not a measurement of clock offset truth.',
              'linear_difference_residual_m_s': statistics(velocity_residuals),
              'angular_difference_residual_rad_s': statistics(angular_residuals)}
    if odom and manifest['case'].startswith('stationary'):
        report.update(position_drift_m=statistics([v for row in odom for v in row[1]-odom[0][1]]),
                      orientation_drift_rad=statistics([v for row in odom for v in (row[2]*odom[0][2].inv()).as_rotvec()]),
                      world_linear_m_s=statistics([v for row in odom for v in row[3]]),
                      world_angular_rad_s=statistics([v for row in odom for v in row[4]]))
    path = args.record / 'summary.json'
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2))
    print(path)


if __name__ == '__main__':
    main()
