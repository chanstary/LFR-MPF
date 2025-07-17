# LFR-MPF-Simulation/visualization.py

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Wedge
from matplotlib.transforms import Affine2D
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from IPython.display import display, clear_output
import math

from .config import *

def _get_plot_position(angle, radius):
    return radius * np.cos(angle), radius * np.sin(angle)

def _visualize_lanes(ax):
    """Draws the roundabout boundaries and entry/exit lanes."""
    roundabout_region = Wedge((0, 0), OUTER_RADIUS + 2, 0, 360,
                              width=OUTER_RADIUS - INNER_RADIUS + 4, color='lightgray', alpha=0.5)
    ax.add_patch(roundabout_region)
    inner_circle = plt.Circle((0, 0), INNER_RADIUS, fill=False, color='black', ls='--', lw=2)
    outer_circle = plt.Circle((0, 0), OUTER_RADIUS, fill=False, color='black', ls='--', lw=2)
    ax.add_artist(inner_circle)
    ax.add_artist(outer_circle)
    for i in range(len(ENTRY_ANGLES)):
        ex, ey = _get_plot_position(ENTRY_ANGLES[i], OUTER_RADIUS)
        ax.plot([ex, ex + 30 * np.cos(ENTRY_ANGLES[i])],
                [ey, ey + 30 * np.sin(ENTRY_ANGLES[i])], 'g--', lw=1.5)
        ax.plot(ex, ey, 'gs', markersize=8, label=f"Entry" if i == 0 else "")
    for i in range(len(EXIT_ANGLES)):
        ex, ey = _get_plot_position(EXIT_ANGLES[i], OUTER_RADIUS)
        ax.plot([ex, ex + 30 * np.cos(EXIT_ANGLES[i])],
                [ey, ey + 30 * np.sin(EXIT_ANGLES[i])], 'r--', lw=1.5)
        ax.plot(ex, ey, 'rs', markersize=8, label=f"Exit" if i == 0 else "")
    ax.legend()


def animate_simulation(vehicle_positions):
    """
    Animates the simulation results.
    Note: This function is designed to be run in a Jupyter environment
    due to its use of IPython.display.
    """
    fig, ax = plt.subplots(figsize=(12, 12))
    plt.rcParams.update({'font.size': 14, 'font.family': 'serif', 'font.serif': 'Times New Roman'})

    norm = Normalize(vmin=0, vmax=DESIRED_SPEED + 5)
    cmap = plt.get_cmap('viridis')
    sm = ScalarMappable(cmap=cmap, norm=norm)
    
    # 1. Find the maximum length among all vehicle records
    max_len = 0
    for i in range(NUM_VEHICLES):
        length = len(vehicle_positions[f"Veh[{i}]"]["position_x"])
        if length > max_len:
            max_len = length

    # 2. Pad shorter records with a sentinel value
    for i in range(NUM_VEHICLES):
        data = vehicle_positions[f"Veh[{i}]"]
        current_len = len(data["position_x"])
        
        if current_len < max_len:
            padding_size = max_len - current_len
            data["position_x"].extend([999] * padding_size)
            data["position_y"].extend([999] * padding_size)
            data["tangential_speeds"].extend([0] * padding_size)
            data["radial_speeds"].extend([0] * padding_size)
            
            last_entry_idx = data["entry_idx"][-1] if data["entry_idx"] else -1
            last_exit_idx = data["exit_idx"][-1] if data["exit_idx"] else -1
            data["entry_idx"].extend([last_entry_idx] * padding_size)
            data["exit_idx"].extend([last_exit_idx] * padding_size)

    print(f"Data padded. All vehicle records now have a consistent length of {max_len} steps.")

    num_recorded_steps = max_len 
    FRAME_SKIP = 2
    
    for t in range(0, num_recorded_steps, FRAME_SKIP):
        ax.clear()
        ax.set_xlim(-OUTER_RADIUS - 40, OUTER_RADIUS + 40)
        ax.set_ylim(-OUTER_RADIUS - 40, OUTER_RADIUS + 40)
        ax.set_aspect('equal')
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title(f'Roundabout Simulation: Time = {t * DT:.1f}s')
        _visualize_lanes(ax)
        
        for i in range(NUM_VEHICLES):
            vehicle_data = vehicle_positions[f"Veh[{i}]"]
            x = vehicle_data["position_x"][t]
            y = vehicle_data["position_y"][t]
            
            if x > 900: # Sentinel value check
                continue
                
            vx = vehicle_data["tangential_speeds"][t]
            vy = vehicle_data["radial_speeds"][t]
            speed = math.sqrt(vx**2 + vy**2)
            color = sm.to_rgba(speed)
            pos_angle = np.arctan2(y, x)
            dist_from_center = np.sqrt(x**2 + y**2)

            if dist_from_center > OUTER_RADIUS:
                 orientation_angle = pos_angle
            else:
                if vx <= 1e-6:
                    orientation_angle = pos_angle + np.pi / 2
                else:
                    movement_angle = np.arctan(vy / (vx + 1e-6))
                    orientation_angle = pos_angle + movement_angle
            
            rect = Rectangle(
                (x - VEHICLE_LENGTH / 2, y - VEHICLE_WIDTH / 2),
                VEHICLE_LENGTH, VEHICLE_WIDTH,
                color=color, alpha=0.9
            )
            transform = Affine2D().rotate_around(x, y, orientation_angle)
            rect.set_transform(transform + ax.transData)
            ax.add_patch(rect)
            
            entry_idx = vehicle_data["entry_idx"][t]
            exit_idx = vehicle_data["exit_idx"][t]
            ax.text(x, y, f"{entry_idx+1},{exit_idx+1}", color='white',
                    ha='center', va='center', fontsize=8, weight='bold')

        display(fig)
        plt.pause(1e-4)
        clear_output(wait=True)
    
    plt.show()