import json
import matplotlib.pyplot as plt
from datetime import datetime
import sys

def load_json_data(filepath):
    with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
    if isinstance(data, dict):
        data = [data]
    return data

def plot_dual_fixed_and_zoom(data):
    times = [datetime.fromtimestamp(entry["timestamp_ms"] / 1000.0) for entry in data]
    mm_vals = [entry["mm"] for entry in data]

    y_min, y_max = min(mm_vals), max(mm_vals)
    margin = (y_max - y_min) * 0.05 or 1.0
    zoom_y_min = y_min - margin
    zoom_y_max = y_max + margin

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 9), sharex=False)

    ax1.scatter(times, mm_vals, s=30, color='blue', alpha=0.8, edgecolors='black', linewidth=0.3, zorder=5)
    ax1.plot(times, mm_vals, color='blue', alpha=0.35, linewidth=1.0, zorder=4)
    ax1.set_ylim(125, 500)
    ax1.set_ylabel('Distance (mm)\n[Fixed: 125–500]', fontsize=11)
    ax1.set_title('Laser Distance — Full Sensor Range', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.5)

    ax2.scatter(times, mm_vals, s=30, color='green', alpha=0.8, edgecolors='black', linewidth=0.3, zorder=5)
    ax2.plot(times, mm_vals, color='green', alpha=0.35, linewidth=1.0, zorder=4)
    ax2.set_ylim(zoom_y_min, zoom_y_max)
    ax2.set_ylabel('Distance (mm)\n[Zoomed to Data]', fontsize=11)
    ax2.set_xlabel('Time (with millisecond precision)')
    ax2.set_title('Laser Distance — Zoomed View', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.5)

    for ax in (ax1, ax2):
        ax.tick_params(axis='x', rotation=45)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python dual_plot_fixed_and_zoom.py <json_file>")
        sys.exit(1)
    data = load_json_data(sys.argv[1])
    plot_dual_fixed_and_zoom(data)