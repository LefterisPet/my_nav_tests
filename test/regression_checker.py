#!/usr/bin/env python3

"""Regression checker module to validate navigation metrics against defined thresholds."""

import json
import os
import sys


def check_regression():
    target_dir = os.path.dirname(os.path.abspath(__file__))
    report_path = os.path.join(target_dir, 'navigation_regression_report.json')
    thresholds_path = os.path.join(target_dir, 'test_thresholds.json')

    print('[CHECKER] Loading navigation report and threshold configuration...')

    if not os.path.exists(report_path):
        print(f'[ERROR] Navigation report not found at: {report_path}')
        sys.exit(1)

    if not os.path.exists(thresholds_path):
        print(f'[ERROR] Thresholds configuration not found at: {thresholds_path}')
        sys.exit(1)

    with open(report_path, 'r', encoding='utf-8') as f:
        metrics = json.load(f)

    with open(thresholds_path, 'r', encoding='utf-8') as f:
        config = json.load(f)

    thresholds = config.get('thresholds', {})
    passed = True

    print('\n' + '='*50)
    print('[CHECKER] Validating metrics against thresholds...')
    print('='*50)

    # 1. Check success flag
    if 'success' in thresholds:
        expected_success = thresholds['success']
        actual_success = metrics.get('success', False)
        if actual_success != expected_success:
            print(f'[FAIL] Success check failed! Expected: {expected_success}, Got: {actual_success}')
            passed = False
        else:
            print(f'[PASS] Success check passed: {actual_success}')

    # 2. Check total time
    if 'total_time_sec_max' in thresholds:
        max_time = thresholds['total_time_sec_max']
        actual_time = metrics.get('total_time_sec', 0.0)
        if actual_time > max_time:
            print(f'[FAIL] Total time exceeded! Limit: {max_time}s, Got: {actual_time}s')
            passed = False
        else:
            print(f'[PASS] Total time within limit: {actual_time}s <= {max_time}s')

    # 3. Check path efficiency
    if 'path_efficiency_min' in thresholds:
        min_eff = thresholds['path_efficiency_min']
        actual_eff = metrics.get('path_efficiency', 0.0)
        if actual_eff < min_eff:
            print(f'[FAIL] Path efficiency too low! Min: {min_eff}, Got: {actual_eff}')
            passed = False
        else:
            print(f'[PASS] Path efficiency acceptable: {actual_eff} >= {min_eff}')

    # 4. Check final positional error
    if 'final_positional_error_max' in thresholds:
        max_err = thresholds['final_positional_error_max']
        actual_err = metrics.get('final_positional_error_m', 0.0)
        if actual_err > max_err:
            print(f'[FAIL] Positional error too high! Max: {max_err}m, Got: {actual_err}m')
            passed = False
        else:
            print(f'[PASS] Positional error within limit: {actual_err}m <= {max_err}m')

    # 5. Check obstacle clearance
    if 'min_obstacle_clearance_min' in thresholds:
        min_clear = thresholds['min_obstacle_clearance_min']
        actual_clear = metrics.get('min_obstacle_clearance_m', 0.0)
        if actual_clear < min_clear:
            print(f'[FAIL] Obstacle clearance too close! Min: {min_clear}m, Got: {actual_clear}m')
            passed = False
        else:
            print(f'[PASS] Obstacle clearance safe: {actual_clear}m >= {min_clear}m')

    print('='*50)
    if passed:
        print('[SUCCESS] 🎉 All regression thresholds passed successfully!')
        sys.exit(0)
    else:
        print('[ERROR] ❌ Regression test failed due to threshold violations!')
        sys.exit(1)


if __name__ == '__main__':
    check_regression()
