import json
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

def generate_ideal_trajectory(x_min=950, x_max=1280, y_min=-150, y_max=100, step_x=10, step_y=10, z_scan=725):
    points = []
    y_current = y_min
    row = 0
    
    while y_current <= y_max:
        row += 1
        if row % 2 == 1:
            x_current = x_min
            while x_current <= x_max:
                points.append([x_current, y_current, z_scan])
                x_current += step_x
        else:
            x_current = x_max
            while x_current >= x_min:
                points.append([x_current, y_current, z_scan])
                x_current -= step_x
        y_current += step_y
    
    return np.array(points, dtype=float)

def load_robot_trajectory(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)
    
    robot_trajectory = []
    for entry in data:
        robot_trajectory.append([
            entry['robot_x_mm'],
            entry['robot_y_mm'],
            entry['robot_z_mm']
        ])
    
    return np.array(robot_trajectory, dtype=float)

def visualize_trajectories(robot_file, x_min=950, x_max=1280, y_min=-150, y_max=100, step_x=10, step_y=10, z_scan=725):
    ideal_traj = generate_ideal_trajectory(x_min, x_max, y_min, y_max, step_x, step_y, z_scan)
    robot_traj = load_robot_trajectory(robot_file)
    
    print(f"Идеальных точек: {len(ideal_traj)}")
    print(f"Реальных точек: {len(robot_traj)}")
    
    fig = plt.figure(figsize=(14, 6))
    
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.plot(ideal_traj[:, 0], ideal_traj[:, 1], ideal_traj[:, 2], 'b-', label='Идеальная', linewidth=1.2)
    ax1.plot(robot_traj[:, 0], robot_traj[:, 1], robot_traj[:, 2], 'r-', label='Реальная', linewidth=1.2)
    ax1.set_xlabel('X (мм)')
    ax1.set_ylabel('Y (мм)')
    ax1.set_zlabel('Z (мм)')
    ax1.set_title('Траектории (3D)')
    ax1.legend()
    
    ax2 = fig.add_subplot(122)
    ax2.plot(ideal_traj[:, 0], ideal_traj[:, 1], 'b-', label='Идеальная', linewidth=1.2)
    ax2.plot(robot_traj[:, 0], robot_traj[:, 1], 'r-', label='Реальная', linewidth=1.2)
    ax2.set_xlabel('X (мм)')
    ax2.set_ylabel('Y (мм)')
    ax2.set_title('Траектории (XY)')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Использование: python script.py <путь_к_json>")
        sys.exit(1)
    
    json_file = sys.argv[1]
    if not os.path.exists(json_file):
        print(f"Ошибка: файл не найден — {json_file}")
        sys.exit(1)
    
    visualize_trajectories(
        json_file,
        x_min=950,
        x_max=1280,
        y_min=-150,
        y_max=100,
        step_x=10,
        step_y=10,
        z_scan=725  # 600 + 125
    )