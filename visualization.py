import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from config import *

def plot_roundabout_geometry(ax):
    """Plots the inner and outer boundaries of the roundabout."""
    outer_circle = Circle((0, 0), OUTER_RADIUS, color='gray', fill=False, lw=2, linestyle='--')
    inner_circle = Circle((0, 0), INNER_RADIUS, color='gray', fill=False, lw=2, linestyle='--')
    ax.add_patch(outer_circle)
    ax.add_patch(inner_circle)
    
    # Plot entry/exit nodes
    node_angles = np.linspace(0, 2 * np.pi, NUM_NODES, endpoint=False)
    for angle in node_angles:
        x_outer = (OUTER_RADIUS + 5) * np.cos(angle)
        y_outer = (OUTER_RADIUS + 5) * np.sin(angle)
        x_inner = (OUTER_RADIUS - 5) * np.cos(angle)
        y_inner = (OUTER_RADIUS - 5) * np.sin(angle)
        ax.plot([x_inner, x_outer], [y_inner, y_outer], 'k--', lw=0.5)

def plot_trajectories(vehicles):
    """Plots the full trajectory of each vehicle."""
    fig, ax = plt.subplots(figsize=(10, 10))
    plot_roundabout_geometry(ax)
    
    for v in vehicles:
        if v.position_x:
            ax.plot(v.position_x, v.position_y, label=f'Vehicle {v.idx}')
            
    ax.set_title("Vehicle Trajectories in LFR-MPF Simulation")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)
    # plt.legend() # Legend can be cluttered, optional

def plot_final_positions(vehicles):
    """Plots the final position of each vehicle."""
    fig, ax = plt.subplots(figsize=(10, 10))
    plot_roundabout_geometry(ax)
    
    for v in vehicles:
        if v.position_x:
            x = v.position_x[-1]
            y = v.position_y[-1]
            ax.plot(x, y, 'o', label=f'Vehicle {v.idx}')
            ax.text(x, y, str(v.idx))
            
    ax.set_title("Final Vehicle Positions")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_aspect('equal', adjustable='box')
    ax.grid(True)

