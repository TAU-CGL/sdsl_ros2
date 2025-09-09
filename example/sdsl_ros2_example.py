import numpy as np
import yaml
from PIL import Image
import os
import time

import sdsl


def load_map_files(yaml_path):
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

def parse_sds_txt(sds_path):
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
        # Direction vector in XY plane (cos, sin, 0)
        direction = [np.cos(angle), np.sin(angle), 0.0]
        # Origin at sensor position (0, 0, 0)
        origin = [0.0, 0.0, 0.0]
        
        gs.append(origin + direction)  # [x, y, z, dx, dy, dz]
        ds.append(range_val)
    
    return np.array(gs), np.array(ds)

def plot_map_points(points, midpoints=None):
    """Plot map points using matplotlib."""
    import matplotlib.pyplot as plt
    if len(points) > 0:
        plt.figure(figsize=(10, 8))
        plt.scatter(points[:, 0], points[:, 1], s=1, label='Map points')
        if midpoints is not None and len(midpoints) > 0:
            midpoints = np.array(midpoints)
            plt.scatter(midpoints[:, 0], midpoints[:, 1], s=10, c='cyan', label='Localization')
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Map Point Cloud')
        plt.axis('equal')
        plt.legend()
        plt.show()



if __name__ == "__main__":
    # MAP_NAME = "example/data/maps/lab446_20250827_1700/my_map"
    # MAP_NAME = "example/data/maps/fl4_20250813_1725/my_map"
    MAP_NAME = "example/data/maps/fl4_lowres/my_map"
    yaml_path = MAP_NAME + ".yaml"
    sds_path = "example/data/sds.txt"
    
    points = load_map_files(yaml_path)    
    gs, ds = parse_sds_txt(sds_path)

    for i in range(gs.shape[0]):
        print(f"odometry.push_back(R3xS2<FT>({','.join([str(x) for x in gs[i]])}));")
    for d in ds:
        print(f"measurements.push_back({d});")

    ds[3] *= 0.5
    ds[5] *= 0.5
    ds[11] *= 0.5

    num_points = points.shape[0]
    with open("example/tmp/points.txt", "w") as fp:
        for i in range(num_points):
            fp.write(f"{points[i][0]},{points[i][1]},{points[i][2]}\n")

    env = sdsl.Env_R3_PCD(points)
    odometry = []
    for g in gs:
        odometry.append(sdsl.R3xS2(*g))
    measurements = ds.tolist()

    schedule = [
        [4, 8, 1, 4],
        [2, 2, 1, 2],
        [2, 2, 1, 2], 
        [2, 2, 1, 2],
        [2, 2, 1, 2],
        [2, 2, 1, 2],
        [2, 2, 1, 2],
        [2, 2, 1, 2],
        [2, 2, 1, 2],
    ]

    start = time.time()
    localization = sdsl.localize_R3_pcd_dynamic_scheduled(env, odometry, measurements, 0.05, 6, 13, schedule)
    end = time.time()
    print(f"Took {end-start:.3f}[sec]")

    midpoints = []
    for v in localization:
        bl = v.bottom_left()
        tr = v.top_right()
        midpoints.append(((bl.x() + tr.x()) / 2, (bl.y() + tr.y()) / 2))

    plot_map_points(points, midpoints)