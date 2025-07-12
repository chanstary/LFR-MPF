import numpy as np
from config import *
from models import idm_acceleration, idm_acceleration1, IDM, IAM
from utils import calculate_angle_gap

class Vehicle:
    """
    Represents a single vehicle in the simulation.
    Contains its state, properties, and the core update logic based on the LFR-MPF.
    """
    def __init__(self, idx, angle, radius, entry_angle, exit_angle):
        self.idx = idx
        self.angle = angle % (2 * np.pi)
        self.radius = radius
        self.entry_angle = entry_angle % (2 * np.pi)
        self.exit_angle = exit_angle % (2 * np.pi)
        
        self.tangential_speed = 0.0
        self.radial_speed = -10.0  # Initial speed to enter the roundabout
        self.tangential_acc = 0.0
        self.radial_acc = 0.0
        
        self.width = VEHICLE_WIDTH
        self.length = VEHICLE_LENGTH
        
        # For data logging
        self.position_x = []
        self.position_y = []
        self.tangential_speeds = []
        self.radial_speeds = []
        
        self.in_roundabout = False
        self.out = False # Has exited
        
        # Decision making state
        self.decide = 100 # Placeholder for decision state
        self.entry_idx = None
        self.exit_idx = None
        
        # Parameters that can be heterogeneous
        self.gammar = PATH_ADJUSTMENT_SENSITIVITY
        self.max_angle = np.deg2rad(60)
        self.desired_speed = DESIRED_SPEED
        self.T = DESIRED_TIME_GAP

    def update(self, front_vehicle, follower_vehicle, dt):
        """
        Updates the vehicle's state for one time step.
        This method implements the core logic of the LFR-MPF.
        """
        # Note: The original code has complex, state-based logic. This is a simplified
        # representation focusing on the main interaction cases.
        
        # 1. Determine effective road boundaries
        effective_inner_radius, effective_outer_radius = self.calculate_effective_radius()

        # 2. Calculate Tangential Acceleration (Tangential-IAM)
        if self.radius < OUTER_RADIUS:
            if front_vehicle == 1: # Exiting logic
                angle_diff_exit = calculate_angle_gap(self.angle, self.exit_angle)
                gap = self.radius * angle_diff_exit
                sy = OUTER_RADIUS - self.radius
                self.tangential_acc = idm_acceleration1(self.tangential_speed, 0, gap, sy)
            elif front_vehicle is None: # Free driving
                self.tangential_acc = MAX_ACCELERATION * (1 - (self.tangential_speed / self.desired_speed) ** 4)
            else: # Following a vehicle
                angle_diff = calculate_angle_gap(self.angle, front_vehicle.angle)
                gap = min(self.radius * angle_diff, front_vehicle.radius * angle_diff) - self.length
                sy = self.radius - front_vehicle.radius
                self.tangential_acc = idm_acceleration(self.tangential_speed, front_vehicle.tangential_speed, gap, sy)
            
            # Influence from follower vehicle
            if follower_vehicle:
                # This part of the logic seems to reduce acceleration if a follower is close,
                # which is a cooperative behavior.
                pass # Simplified for clarity

        else: # Outside roundabout (on ramp)
            # Logic for approaching vehicles
            gap_to_entry = self.radius - OUTER_RADIUS
            self.tangential_acc = 0
            self.radial_acc = -IDM(abs(self.radial_speed), 0, gap_to_entry)

        # 3. Calculate Radial Acceleration (Radial-IAM + Target Guidance + Boundary)
        if self.radius < OUTER_RADIUS:
            # Radial-IAM component
            acc_rad_iam = 0
            if front_vehicle and front_vehicle != 1:
                iam_ego_params = {'width': self.width, 'speed_y': self.radial_speed, 'position_y': self.radius, 'speed_x': self.tangential_speed}
                iam_front_params = {'width': front_vehicle.width, 'speed_y': front_vehicle.radial_speed, 'position_y': front_vehicle.radius}
                acc_rad_iam = IAM(self.tangential_acc, 1, 0.6, 0.7, 0.5, iam_ego_params, iam_front_params)

            # Target Guidance Force
            acc_target_guidance = self.targetF()

            # Boundary Force
            acc_boundary = self.calcAccB(effective_outer_radius, effective_inner_radius)
            
            self.radial_acc = acc_rad_iam + acc_target_guidance + acc_boundary
        
        # 4. Update State (Kinematics)
        self.tangential_speed = max(0, self.tangential_speed + self.tangential_acc * dt)
        self.radial_speed += self.radial_acc * dt

        # Apply kinematic constraints (e.g., max steering angle)
        self.apply_kinematic_constraints()

        # Update position
        self.angle = (self.angle + self.tangential_speed / self.radius * dt) % (2 * np.pi)
        self.radius += self.radial_speed * dt
        self.radius = max(INNER_RADIUS + self.width / 2, min(self.radius, OUTER_RADIUS - self.width / 2))
        
        # Exit condition
        angle_to_exit = calculate_angle_gap(self.angle, self.exit_angle)
        if angle_to_exit < np.arcsin(3 / OUTER_RADIUS) and self.radius > OUTER_RADIUS - 10:
            self.out = True # Mark as exited
            self.radius = 999 # Move vehicle out of simulation area

        # Record data
        self.log_state()

    def targetF(self):
        # Implementation of the Target Guidance Model force
        # This is a simplified representation of the logic in the paper.
        
        # Determine target radius based on long/short path strategy
        is_long_path = calculate_angle_gap(self.entry_angle, self.exit_angle) >= np.pi / 2
        
        target_radius = 0
        if is_long_path:
            # Phase 1: Move inward
            if calculate_angle_gap(self.entry_angle, self.angle) < calculate_angle_gap(self.entry_angle, self.exit_angle) / 2:
                target_radius = INNER_RADIUS + self.width / 2 + 3 # Target inner area
            # Phase 2: Move outward for exit
            else:
                target_radius = OUTER_RADIUS - self.width / 2
        else: # Short path
            target_radius = OUTER_RADIUS - self.width / 2

        # Calculate force based on deviation from target radius
        radial_deviation = target_radius - self.radius
        force = np.sign(radial_deviation) * MAX_GUIDING_FORCE * np.exp(-self.gammar * abs(radial_deviation))
        
        return max(-2, min(2, force))

    def calcAccB(self, widthLeft, widthRight):
        # Implementation of the Boundary Force
        syRight = self.radius - INNER_RADIUS
        syLeft = OUTER_RADIUS - self.radius
        
        def alphaLatBfun(sy, B):
            return np.exp(-sy / B) if sy > 0 else 1 - sy / B
            
        alphaLatLeft = alphaLatBfun(syLeft, BOUNDARY_INTERACTION_SCALE)
        alphaLatRight = alphaLatBfun(syRight, BOUNDARY_INTERACTION_SCALE)
        
        accLatB0 = MAX_BOUNDARY_ACC * (alphaLatRight - alphaLatLeft)
        accLatB = accLatB0 * (0.2 + 0.8 * self.tangential_speed / DESIRED_SPEED)
        
        return max(-6, min(6, accLatB))

    def apply_kinematic_constraints(self):
        # Constrains radial speed based on max steering angle
        if self.tangential_speed > 1e-6:
            current_steering_angle = np.arctan(self.radial_speed / self.tangential_speed)
            if abs(current_steering_angle) > self.max_angle:
                self.radial_speed = np.sign(self.radial_speed) * self.tangential_speed * np.tan(self.max_angle)

    def calculate_effective_radius(self):
        # Simplified: in the real model, this could change based on proximity to entry/exit
        return INNER_RADIUS, OUTER_RADIUS

    def log_state(self):
        self.position_x.append(self.radius * np.cos(self.angle))
        self.position_y.append(self.radius * np.sin(self.angle))
        self.tangential_speeds.append(self.tangential_speed)
        self.radial_speeds.append(self.radial_speed)

    def __str__(self):
        return f"Vehicle {self.idx}: Angle={np.degrees(self.angle):.1f}°, R={self.radius:.1f}m, v_t={self.tangential_speed:.2f}m/s, v_r={self.radial_speed:.2f}m/s"

