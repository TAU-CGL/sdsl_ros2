#!/usr/bin/env python3

import numpy as np
import time
import itertools
import signal
from typing import List, Tuple, Dict, Optional
import yaml
from PIL import Image
import os

import sdsl


class TimeoutError(Exception):
    pass


def timeout_handler(signum, frame):
    raise TimeoutError("Operation timed out")


def load_map_files(yaml_path: str) -> np.ndarray:
    """Load PGM and YAML map files into a numpy array."""
    with open(yaml_path, 'r') as f:
        map_info = yaml.safe_load(f)
    
    pgm_path = os.path.join(os.path.dirname(yaml_path), map_info['image'])
    map_array = np.array(Image.open(pgm_path))
    
    points = []
    height, width = map_array.shape
    
    for y in range(height):
        for x in range(width):
            cell = map_array[y, x]
            if cell < 0 or cell > 20:
                continue
            wx = map_info['origin'][0] + (x + 0.5) * map_info['resolution']
            wy = map_info['origin'][1] - (y + 0.5) * map_info['resolution']
            points.append([wx, wy, 0.0])
    
    return np.array(points)


def parse_sds_txt(sds_path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Parse sds.txt file and return gs and ds arrays."""
    with open(sds_path, 'r') as f:
        data = yaml.safe_load(f)
    
    angle_min = data['angle_min']
    angle_increment = data['angle_increment']
    range_min = data['range_min']
    range_max = data['range_max']
    ranges = data['ranges']
    
    gs = []
    ds = []
    
    for i, range_val in enumerate(ranges):
        if range_val < range_min or range_val > range_max or np.isnan(range_val):
            continue
        
        angle = angle_min + i * angle_increment
        direction = [np.cos(angle), np.sin(angle), 0.0]
        origin = [0.0, 0.0, 0.0]
        
        gs.append(origin + direction)
        ds.append(range_val)
    
    return np.array(gs), np.array(ds)


def evaluate_schedule(env: sdsl.Env_R3_PCD, 
                     odometry: List[sdsl.R3xS2], 
                     measurements: List[float],
                     schedule: List[List[int]],
                     sigma: float = 0.05,
                     min_depth: int = 7,
                     max_depth: int = 13,
                     target_time_sec: float = 1.0,
                     timeout_sec: float = 4.0) -> Dict:
    """
    Evaluate a subdivision schedule with timeout protection.
    
    Returns:
        dict: Contains 'time', 'num_results', 'avg_volume', 'valid' (meets time constraint)
    """
    start_time = time.time()
    
    # Set up timeout signal
    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(int(timeout_sec))
    
    try:
        localization = sdsl.localize_R3_pcd_dynamic_scheduled(
            env, odometry, measurements, sigma, min_depth, max_depth, schedule
        )
        signal.alarm(0)  # Cancel timeout
        end_time = time.time()
        
        query_time = end_time - start_time
        num_results = len(localization)
        
        # Calculate average volume of resulting boxes
        total_volume = 0.0
        for v in localization:
            bl = v.bottom_left()
            tr = v.top_right()
            dx = tr.x() - bl.x()
            dy = tr.y() - bl.y()
            dz = tr.z() - bl.z()
            total_volume += dx * dy * dz
        
        avg_volume = total_volume / num_results if num_results > 0 else float('inf')
        
        return {
            'time': query_time,
            'num_results': num_results,
            'avg_volume': avg_volume,
            'valid': query_time <= target_time_sec,
            'schedule': schedule.copy()
        }
    
    except TimeoutError:
        signal.alarm(0)
        return {
            'time': timeout_sec,
            'num_results': 0,
            'avg_volume': float('inf'),
            'valid': False,
            'schedule': schedule.copy(),
            'error': f'Timeout after {timeout_sec}s'
        }
    except Exception as e:
        signal.alarm(0)
        return {
            'time': float('inf'),
            'num_results': 0,
            'avg_volume': float('inf'),
            'valid': False,
            'schedule': schedule.copy(),
            'error': str(e)
        }
    finally:
        # Restore original signal handler
        signal.signal(signal.SIGALRM, old_handler)


def generate_schedule_candidates(num_levels: int = 9, 
                               param_ranges: Dict[str, List[int]] = None) -> List[List[List[int]]]:
    """
    Generate candidate subdivision schedules to test.
    
    Args:
        num_levels: Number of subdivision levels
        param_ranges: Dict with ranges for each parameter
    
    Returns:
        List of schedule candidates
    """
    if param_ranges is None:
        param_ranges = {
            'param1': [1, 2, 3, 4],  # First parameter range
            'param2': [2, 3, 4, 5],  # Second parameter range  
            'param3': [1],           # Third parameter (usually 1)
            'param4': [1, 2, 3, 4]   # Fourth parameter range
        }
    
    schedules = []
    
    # Generate some basic patterns
    patterns = [
        # Progressive refinement
        lambda i: [min(4, 2 + i//3), min(5, 2 + i//3), 1, min(4, 2 + i//3)],
        # Conservative start, aggressive later
        lambda i: [2 if i < 3 else 3, 3 if i < 3 else 4, 1, 2 if i < 3 else 3],
        # Aggressive start, conservative later  
        lambda i: [4 if i < 3 else 2, 4 if i < 3 else 3, 1, 4 if i < 3 else 2],
        # Uniform patterns
        lambda i: [2, 2, 1, 2],
        lambda i: [3, 3, 1, 3],
        lambda i: [4, 4, 1, 4],
    ]
    
    for pattern in patterns:
        schedule = [pattern(i) for i in range(num_levels)]
        schedules.append(schedule)
    
    # Add some random variations
    np.random.seed(42)  # For reproducibility
    for _ in range(20):
        schedule = []
        for i in range(num_levels):
            level = [
                np.random.choice(param_ranges['param1']),
                np.random.choice(param_ranges['param2']),
                np.random.choice(param_ranges['param3']),
                np.random.choice(param_ranges['param4'])
            ]
            schedule.append(level)
        schedules.append(schedule)
    
    return schedules


def optimize_schedule(map_path: str, 
                     sds_path: str,
                     target_time_sec: float = 1.0,
                     num_candidates: int = 50,
                     verbose: bool = True) -> Dict:
    """
    Find the best subdivision schedule for the given map and sensor data.
    
    Args:
        map_path: Path to map YAML file (without extension)
        sds_path: Path to sensor data file
        target_time_sec: Maximum allowed query time
        num_candidates: Number of schedule candidates to test
        verbose: Print progress information
    
    Returns:
        Dict with best schedule and performance metrics
    """
    if verbose:
        print("Loading map and sensor data...")
    
    # Load data
    points = load_map_files(map_path + ".yaml")
    gs, ds = parse_sds_txt(sds_path)
    
    # Apply any modifications from original example
    ds[3] *= 0.5
    ds[5] *= 0.5
    ds[11] *= 0.5
    
    # Setup SDSL environment
    env = sdsl.Env_R3_PCD(points)
    odometry = [sdsl.R3xS2(*g) for g in gs]
    measurements = ds.tolist()
    
    if verbose:
        print(f"Map points: {len(points)}")
        print(f"Sensor rays: {len(odometry)}")
        print(f"Target query time: {target_time_sec}s")
        print()
    
    # Generate schedule candidates
    if verbose:
        print("Generating schedule candidates...")
    
    candidates = generate_schedule_candidates()[:num_candidates]
    
    if verbose:
        print(f"Testing {len(candidates)} schedule candidates...")
        print()
    
    # Evaluate all candidates
    results = []
    valid_results = []
    
    for i, schedule in enumerate(candidates):
        if verbose and (i + 1) % 10 == 0:
            print(f"Progress: {i + 1}/{len(candidates)}")
        
        result = evaluate_schedule(env, odometry, measurements, schedule, target_time_sec=target_time_sec)
        results.append(result)
        
        if result['valid'] and 'error' not in result:
            valid_results.append(result)
    
    if verbose:
        print(f"\nCompleted testing. Found {len(valid_results)} valid schedules (within {target_time_sec}s)")
    
    if not valid_results:
        print("WARNING: No schedules met the time constraint!")
        # Return best overall result
        best = min(results, key=lambda x: x['time'])
        return {
            'best_schedule': best['schedule'],
            'best_time': best['time'],
            'best_volume': best['avg_volume'],
            'num_results': best['num_results'],
            'all_valid': False,
            'num_valid': 0,
            'total_tested': len(results)
        }
    
    # Find best valid schedule (minimize average volume, as smaller = more precise)
    best = min(valid_results, key=lambda x: x['avg_volume'])
    
    if verbose:
        print(f"\nBest schedule found:")
        print(f"  Query time: {best['time']:.3f}s")
        print(f"  Num results: {best['num_results']}")
        print(f"  Avg volume: {best['avg_volume']:.6f}")
        print(f"  Schedule:")
        for i, level in enumerate(best['schedule']):
            print(f"    Level {i}: {level}")
    
    return {
        'best_schedule': best['schedule'],
        'best_time': best['time'],
        'best_volume': best['avg_volume'],
        'num_results': best['num_results'],
        'all_valid': True,
        'num_valid': len(valid_results),
        'total_tested': len(results),
        'all_results': sorted(valid_results, key=lambda x: x['avg_volume'])
    }


def save_schedule(schedule: List[List[int]], filename: str):
    """Save a schedule to a file."""
    with open(filename, 'w') as f:
        f.write("# Optimized subdivision schedule\n")
        f.write("# Format: [param1, param2, param3, param4] per level\n")
        f.write("schedule = [\n")
        for level in schedule:
            f.write(f"    {level},\n")
        f.write("]\n")


if __name__ == "__main__":
    # Configuration
    MAP_NAME = "example/data/maps/fl4_lowres/my_map"
    SDS_PATH = "example/data/sds.txt"
    TARGET_TIME = 1.0  # seconds
    
    print("=== SDSL Schedule Optimizer ===")
    print()
    
    # Run optimization
    result = optimize_schedule(
        map_path=MAP_NAME,
        sds_path=SDS_PATH,
        target_time_sec=TARGET_TIME,
        num_candidates=50,
        verbose=True
    )
    
    if result['all_valid']:
        print("\n=== OPTIMIZATION COMPLETE ===")
        print(f"Found {result['num_valid']} valid schedules out of {result['total_tested']} tested")
        
        # Save best schedule
        schedule_file = "optimized_schedule.py"
        save_schedule(result['best_schedule'], schedule_file)
        print(f"Best schedule saved to: {schedule_file}")
        
        # Show top 5 results
        print(f"\nTop 5 schedules by precision (avg volume):")
        for i, res in enumerate(result['all_results'][:5]):
            print(f"{i+1}. Time: {res['time']:.3f}s, Volume: {res['avg_volume']:.6f}, Results: {res['num_results']}")
    else:
        print("\n=== NO VALID SCHEDULES FOUND ===")
        print(f"All {result['total_tested']} schedules exceeded the {TARGET_TIME}s time limit")
        print(f"Best time achieved: {result['best_time']:.3f}s")
        print("Consider:")
        print("- Increasing the target time limit")
        print("- Using a lower resolution map")  
        print("- Reducing the number of sensor measurements")