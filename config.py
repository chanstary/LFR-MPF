# LFR-MPF-Simulation/config.py

import numpy as np

# --- Simulation Parameters ---
DT = 0.1  # Time step (s)
TOTAL_TIME = 100  # Total simulation time (s)
NUM_VEHICLES = 1000 # Total number of vehicles to simulate
FLOW_RATE = 1.8 # Seconds between vehicle spawns

# --- Roundabout Geometry ---
INNER_RADIUS = 46  # Inner radius of the roundabout (m)
OUTER_RADIUS = 84  # Outer radius of the roundabout (m)

# --- Vehicle Properties ---
VEHICLE_WIDTH = 1.8  # Vehicle width (m)
VEHICLE_LENGTH = 4.5  # Vehicle length (m)

# --- Intelligent Driver Model (IDM) Parameters ---
# These parameters control the car-following behavior (tangential acceleration).
DESIRED_SPEED = 15.0  # Desired speed (m/s)
MAX_ACCELERATION = 3.0  # Maximum acceleration (m/s^2)
COMFORTABLE_DECELERATION = 4.0  # Comfortable deceleration (m/s^2)
MIN_SAFE_DISTANCE = 4.0  # Minimum safe distance / jam distance (m)
TIME_HEADWAY = 1.0  # Desired time headway (s)

# --- Entry and Exit Angle Configuration ---
# Generate 12 entry and 12 exit points, sorted by angle.
num_base_points = 4
num_lanes_per_point = 3
lane_spacing_angle = np.pi / 6

base_entry_angles = [
    np.arctan2(np.sqrt(OUTER_RADIUS**2 - (-3/1)**2), -3/1),
    np.arctan2(-np.sqrt(OUTER_RADIUS**2 - (3/1)**2), 3/1),
    np.arctan2(-3/1, -np.sqrt(OUTER_RADIUS**2 - (-3/1)**2)),
    np.arctan2(3/1, np.sqrt(OUTER_RADIUS**2 - (3/1)**2))
]

base_exit_angles = [
    np.arctan2(np.sqrt(OUTER_RADIUS**2 - (3/1)**2), 3/1),
    np.arctan2(-np.sqrt(OUTER_RADIUS**2 - (-3/1)**2), -3/1),
    np.arctan2(-3/1, np.sqrt(OUTER_RADIUS**2 - (3/1)**2)),
    np.arctan2(3/1, -np.sqrt(OUTER_RADIUS**2 - (-3/1)**2))
]

all_entry_angles = []
for i in range(num_base_points):
    for j in range(num_lanes_per_point):
        all_entry_angles.append((base_entry_angles[i] + j * lane_spacing_angle) % (2 * np.pi))

all_exit_angles = []
for i in range(num_base_points):
    for j in range(num_lanes_per_point):
        all_exit_angles.append((base_exit_angles[i] + j * lane_spacing_angle) % (2 * np.pi))

ENTRY_ANGLES = sorted(all_entry_angles)
EXIT_ANGLES = sorted(all_exit_angles)