import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.interpolate import griddata
import sys

def load_json_data(json_file):
    with open(json_file, 'r') as f:
        return json.load(f)

def visualize_interpolated_surface_keep_scale(json_file, min_distance=280, max_distance=292):
    data = load_json_data(json_file)
    
    xs, ys, zs = [], [], []
    
    for p in data:
        dist = p['laser_distance_mm']
        if dist <= 0:
            continue
        
        x = p['robot_x_mm']
        y = p['robot_y_mm']
        robot_z = p['robot_z_mm']
        surface_z = robot_z - dist

        if min_distance <= dist <= max_distance:
            xs.append(x)
            ys.append(y)
            zs.append(surface_z)
    
    if not xs:
        print(f"Нет точек в диапазоне {min_distance}–{max_distance} мм!")
        return
    
    print(f"Найдено {len(xs)} точек детали")
    
    resolution = 2
    
    xi = np.arange(min(xs), max(xs) + resolution, resolution)
    yi = np.arange(min(ys), max(ys) + resolution, resolution)
    Xi, Yi = np.meshgrid(xi, yi)
    
    Zi = griddata(
        (xs, ys), zs,
        (Xi, Yi),
        method='linear'
    )
    
    Zi_masked = np.ma.masked_invalid(Zi) 
    
    fig = plt.figure(figsize=(12, 6))
    
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax1.scatter(xs, ys, zs, c='red', s=15, alpha=0.6)
    ax1.set_title('Точки детали')
    ax1.set_xlabel('X (mm)')
    ax1.set_ylabel('Y (mm)')
    ax1.set_zlabel('Z (mm)')
    
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')
    surf = ax2.plot_surface(Xi, Yi, Zi_masked, cmap='viridis', edgecolor='none', alpha=0.9)
    ax2.set_title('Интерполированная поверхность')
    ax2.set_xlabel('X (mm)')
    ax2.set_ylabel('Y (mm)')
    ax2.set_zlabel('Z (mm)')
    plt.colorbar(surf, ax=ax2, shrink=0.5, aspect=10)
    
    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)
    z_min, z_max = min(zs), max(zs)

    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min)
    ax1.set_xlim((x_min + x_max - max_range)/2, (x_min + x_max + max_range)/2)
    ax1.set_ylim((y_min + y_max - max_range)/2, (y_min + y_max + max_range)/2)
    ax1.set_zlim((z_min + z_max - max_range)/2, (z_min + z_max + max_range)/2)
    
    ax2.set_xlim((x_min + x_max - max_range)/2, (x_min + x_max + max_range)/2)
    ax2.set_ylim((y_min + y_max - max_range)/2, (y_min + y_max + max_range)/2)
    ax2.set_zlim((z_min + z_max - max_range)/2, (z_min + z_max + max_range)/2)
    
    plt.tight_layout()
    output = json_file.replace('.json', '_interpolated_surface.png')
    plt.savefig(output, dpi=150)
    print(f"Поверхность сохранена: {output}")
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python visualize_interpolated_surface_keep_scale.py <json_file>")
        sys.exit(1)
    
    MIN_DISTANCE = 280
    MAX_DISTANCE = 292
    
    visualize_interpolated_surface_keep_scale(
        sys.argv[1],
        min_distance=MIN_DISTANCE,
        max_distance=MAX_DISTANCE
    )