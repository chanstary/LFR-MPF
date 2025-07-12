import numpy as np

# --- Simulation Parameters ---
DT = 0.1  # Time step (s)
SIMULATION_TIME = 120  # Total simulation time (s)
NUM_VEHICLES = 20 # Number of vehicles in the simulation

# --- Roundabout Geometry ---
OUTER_RADIUS = 84.0  # meters
INNER_RADIUS = 46.0  # meters
NUM_NODES = 12 # Number of entry/exit nodes

# --- Vehicle Properties ---
VEHICLE_LENGTH = 4.5  # meters
VEHICLE_WIDTH = 1.8   # meters

# --- LFR-MPF Model Parameters (from paper calibration) ---
# Note: These are representative values. In a real simulation,
# these might be initialized per-vehicle with some variance.

# Tangential-IAM / IDM
MAX_ACCELERATION = 2.25       # a_max (m/s^2)
COMFORTABLE_DECELERATION = 3.51 # b_comp (m/s^2)
DESIRED_SPEED = 15.0          # v0 (m/s)
DESIRED_TIME_GAP = 1.16         # T (s)
JAM_DISTANCE = 3.47           # s0 (m)
RADIAL_ATTENUATION_SCALE = 0.43 # S_theta0 (m)

# Radial-IAM
RADIAL_RELAXATION_TIME = 0.53 # tau (s)
RADIAL_SENSITIVITY = 0.39     # sigma (s)
ATTENUATION_WIDTH = 0.62      # S_r0 (m)
MAX_BOUNDARY_ACC = 3.80       # g_B_hat (m/s^2)
BOUNDARY_INTERACTION_SCALE = 0.17 # S_rB_hat (m)

# Target Guidance Model
MAX_GUIDING_FORCE = 1.58 # g_max (m/s^2)
PATH_ADJUSTMENT_SENSITIVITY = 1.41 # omega

