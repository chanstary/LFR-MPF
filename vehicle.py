# LFR-MPF-Simulation/vehicle.py

import numpy as np
from .config import *
from .models import *
from .utils import calculate_angle_gap

class Vehicle:
    """Represents a single vehicle in the simulation."""
    YIELD_FLAG = 1

    def __init__(self, idx, entry_angle, exit_angle, entry_idx, exit_idx):
        self.idx = idx
        self.angle = 0
        self.radius = 999
        self.entry_angle = entry_angle
        self.exit_angle = exit_angle
        self.entry_idx = entry_idx
        self.exit_idx = exit_idx
        self.tangential_speed = 0.0
        self.radial_speed = 0.0
        self.tangential_acc = 0.0
        self.radial_acc = 0.0
        self.width = VEHICLE_WIDTH
        self.length = VEHICLE_LENGTH
        self.gammar = 3.0
        self.max_angle = np.deg2rad(60)
        self.T = TIME_HEADWAY
        self.desired_speed = DESIRED_SPEED
        self.paused = True
        self.out = False
        self.decide = 100
        self.position_x = [999]
        self.position_y = [999]
        self.tangential_speeds = [0]
        self.radial_speeds = [0]
        self.radius_history = [999]
        self.angle_history = [0]

    def update(self, front_vehicle, follower_vehicle):
        """Updates the vehicle's state for one time step."""
        if self.radius >= OUTER_RADIUS:
            self._handle_approaching(front_vehicle)
        elif self.out and self.decide < 0:
            self._handle_exiting()
        else:
            self._handle_in_roundabout(front_vehicle, follower_vehicle)
        self._record_state()

    def update_decision(self):
        """Updates the vehicle's decision state (enter or exit phase)."""
        if self.decide < 0 or self.radius > OUTER_RADIUS:
            return
        self.angle %= (2 * np.pi)
        entry_range_start = self.entry_angle
        gap_to_exit = calculate_angle_gap(self.entry_angle, self.exit_angle)
        split_angle = gap_to_exit / 2 if gap_to_exit > np.pi / 2 else np.pi / 2
        entry_range_end = (self.entry_angle + split_angle) % (2 * np.pi)
        is_in_entry_range = False
        if entry_range_start <= entry_range_end:
            if entry_range_start <= self.angle < entry_range_end:
                is_in_entry_range = True
        else:
            if self.angle >= entry_range_start or self.angle < entry_range_end:
                is_in_entry_range = True
        self.decide = (self.entry_idx + 1) if is_in_entry_range else -(self.exit_idx + 1)

    def _handle_approaching(self, front_vehicle):
        """Vehicle is outside the roundabout, approaching an entry."""
        if self.decide > 0:
            if front_vehicle:
                gap = self.radius - front_vehicle.radius - self.length
                radial_acc = idm_entry_acceleration(abs(self.radial_speed), abs(front_vehicle.radial_speed), gap)
            else:
                radial_acc = 5 * (1 - (abs(self.radial_speed) / 20) ** 4)
            self.radial_acc = -radial_acc
            self.tangential_acc = 0
            self.tangential_speed = 0
            self.radial_speed = min(0, self.radial_speed + self.radial_acc * DT)
            self.radius += self.radial_speed * DT
        else:
            self.radius = OUTER_RADIUS - self.width / 2
            self.radial_speed = 0

    def _handle_exiting(self):
        """Vehicle has passed the exit point and is moving away."""
        self.angle = self.exit_angle
        self.radial_acc = 4/3
        self.radial_speed += self.radial_acc * DT
        self.radius += self.radial_speed * DT
        if self.radius > OUTER_RADIUS + 32:
             self._set_paused()

    def _handle_in_roundabout(self, front_vehicle, follower_vehicle):
        """Vehicle is inside the main roundabout area."""
        proximity_to_inner = (OUTER_RADIUS - self.radius) / (OUTER_RADIUS - INNER_RADIUS)
        local_desired_speed = self.desired_speed + 5 * proximity_to_inner
        self.T = TIME_HEADWAY - 0.2 * proximity_to_inner
        effective_inner, effective_outer = self._calculate_effective_radius()
        if front_vehicle == self.YIELD_FLAG:
            self._calculate_exit_yield_acceleration(effective_inner, effective_outer)
        elif front_vehicle is None:
            self._calculate_free_road_acceleration(follower_vehicle, local_desired_speed, effective_inner, effective_outer)
        else:
            self._calculate_following_acceleration(front_vehicle, follower_vehicle, local_desired_speed, effective_inner, effective_outer)
        self._update_kinematics()
        self._constrain_movement_angle()
        self._check_for_exit()

    def _calculate_exit_yield_acceleration(self, effective_inner, effective_outer):
        """Calculates acceleration when yielding near an exit."""
        angle_diff_exit = calculate_angle_gap(self.angle, (self.exit_angle + np.pi/36) % (2 * np.pi))
        gap_to_exit = self.radius * abs(angle_diff_exit)
        sy1 = OUTER_RADIUS - self.radius
        self.tangential_acc = idm_exit_approach(self.tangential_speed, 0, gap_to_exit, sy1)
        angle_diff11 = calculate_angle_gap(self.angle, (self.exit_angle - np.pi/36) % (2 * np.pi))
        gap1 = self.radius * abs(angle_diff11)
        new_radial_speed = self.tangential_speed / gap1 * sy1
        self.radial_acc = (new_radial_speed - self.radial_speed) / DT + self._target_force() + self._boundary_force(effective_outer, effective_inner)

    def _calculate_free_road_acceleration(self, follower, local_ds, eff_in, eff_out):
        """Calculates acceleration with no vehicle ahead."""
        tangential_acc = MAX_ACCELERATION * (1 - (self.tangential_speed / local_ds) ** 4)
        if follower:
            angle_diff = calculate_angle_gap(follower.angle, self.angle)
            gap = min(self.radius, follower.radius) * abs(angle_diff) - self.length
            sy = follower.radius - self.radius
            follower_influence = 0.6 * idm_acceleration(follower.tangential_speed, self.tangential_speed, gap, sy)
            tangential_acc -= max(-2, follower_influence)
        self.tangential_acc = tangential_acc
        self.radial_acc = self._target_force() * (1 if self.tangential_acc >= 0 else 0) + self._boundary_force(eff_out, eff_in)

    def _calculate_following_acceleration(self, leader, follower, local_ds, eff_in, eff_out):
        """Calculates acceleration when following another vehicle."""
        angle_diff = calculate_angle_gap(self.angle, leader.angle)
        gap = min(self.radius, leader.radius) * abs(angle_diff) - self.length
        sy = self.radius - leader.radius
        tangential_acc = idm_acceleration(self.tangential_speed, leader.tangential_speed, gap, sy)
        if follower:
            angle_diff_f = calculate_angle_gap(follower.angle, self.angle)
            gap_f = min(self.radius, follower.radius) * abs(angle_diff_f) - self.length
            sy_f = follower.radius - self.radius
            follower_influence = 0.6 * idm_acceleration(follower.tangential_speed, self.tangential_speed, gap_f, sy_f)
            tangential_acc -= max(-2, follower_influence)
        self.tangential_acc = tangential_acc
        self.radial_acc = iam_radial_acceleration(
            self.tangential_acc, 1, 0.6, 0.7, 0.5,
            {'width': self.length, 'speed_y': self.radial_speed, 'position_y': self.radius, 'speed_x': self.tangential_speed},
            {'width': leader.length, 'speed_y': leader.radial_speed, 'position_y': leader.radius}
        ) + self._target_force() * (1 if self.tangential_acc >= 0 else 0) + self._boundary_force(eff_out, eff_in)

    def _update_kinematics(self):
        """
        Updates position and velocity based on calculated accelerations, including a reaction delay.
        """
        # --- ADDED CODE: Define the reaction delay ---
        td = 0.2  # Driver reaction delay of 0.2 seconds

        # Calculate how much time is left in the current time step (DT) to apply the new acceleration
        # after the delay has passed. If the delay is longer than the time step,
        # the effective time for acceleration in this step will be 0.
        effective_accel_time = max(0, DT - td)

        # --- Update kinematic equations to incorporate the delay ---

        # 1. Calculate displacement
        # The vehicle moves at its current velocity for the entire duration of the time step (DT).
        # However, the newly calculated acceleration only contributes to displacement during the effective_accel_time.
        radial_displacement = self.radial_speed * DT + 0.5 * self.radial_acc * (effective_accel_time ** 2)
        tangential_displacement_on_arc = self.tangential_speed * DT + 0.5 * self.tangential_acc * (effective_accel_time ** 2)

        # 2. Update position (angle and radius)
        # Use an effective radius for angle calculation to prevent numerical instability.
        next_radius = self.radius + radial_displacement
        effective_radius = max(self.radius, next_radius) if next_radius > 0 else self.radius

        # Update angle
        self.angle = (self.angle + (tangential_displacement_on_arc / effective_radius)) % (2 * np.pi)

        # Update radius and constrain it within the lane boundaries
        self.radius += radial_displacement
        self.radius = max(INNER_RADIUS + self.width / 2, min(self.radius, OUTER_RADIUS - self.width / 2))

        # 3. Update speed
        # The change in velocity only occurs during the effective_accel_time.
        self.tangential_speed = max(0, self.tangential_speed + self.tangential_acc * effective_accel_time)
        self.radial_speed += self.radial_acc * effective_accel_time

    def _constrain_movement_angle(self):
        """Constrains the vehicle's movement angle to prevent unrealistic turns."""
        if self.tangential_speed > 1e-6:
            angle_between = np.arctan(self.radial_speed / self.tangential_speed)
        else:
            angle_between = np.pi / 2 if self.radial_speed > 0 else -np.pi / 2
        max_allowed_angle = np.deg2rad(75) if calculate_angle_gap(self.angle, self.exit_angle) < np.math.asin(20 / OUTER_RADIUS) else np.deg2rad(50)
        if abs(angle_between) >= max_allowed_angle:
            new_radial_speed = self.tangential_speed * np.tan(max_allowed_angle)
            self.radial_speed = np.sign(self.radial_speed) * new_radial_speed

    def _check_for_exit(self):
        """Checks if the vehicle is in a position to exit the roundabout."""
        angle_to_exit = calculate_angle_gap(self.angle, self.exit_angle)
        if (angle_to_exit < np.math.asin(5 / OUTER_RADIUS)) and self.decide < 0 and self.radius > OUTER_RADIUS - 20:
            self.angle = self.exit_angle
            self.radial_speed = 10
            self.radius = OUTER_RADIUS + 0.1
            self.tangential_speed = 0
            self.tangential_acc = 0
            self.radial_acc = 4/3
            self.out = True

    def _target_force(self):
        """Calculates a force directing the vehicle towards its goal (inner or outer lane)."""
        position_x_local = self.radius * calculate_angle_gap(self.entry_angle, self.angle)
        position_y_local = self.radius
        H1 = lambda x: np.where(x <= np.pi, 1, 0)
        if self.decide > 0:
            target_y = INNER_RADIUS + self.length + 3
            gap_to_exit = calculate_angle_gap(self.entry_angle, self.exit_angle)
            Cita = gap_to_exit / 2 if gap_to_exit > np.pi/2 else np.pi/2
            d_cita = calculate_angle_gap(self.entry_angle, (self.entry_angle + Cita) % (2*np.pi))
            target_x = min(INNER_RADIUS + self.length, self.radius) * d_cita
            x_weight = np.exp(-(self.gammar - 1) * (abs(position_x_local - target_x) / (target_x + 1e-6)))
        else:
            target_y = OUTER_RADIUS - self.width / 2
            d_cita = calculate_angle_gap(self.entry_angle, self.exit_angle)
            target_x = OUTER_RADIUS * d_cita
            x_weight = np.exp(-self.gammar * (abs(position_x_local - target_x) / (target_x + 1e-6)))
        y_weight = 1 - np.exp(-abs(target_y - position_y_local))
        result = y_weight * np.sign(target_y - position_y_local) * x_weight * H1(d_cita) * 4
        return max(-4, min(4, result))

    def _boundary_force(self, widthRight, widthLeft):
        """Calculates a repulsive force from the road boundaries."""
        Wroad1 = widthRight + 3
        Wroad2 = widthLeft
        syRight = self.radius - 5 - Wroad1
        syLeft = Wroad2 - self.radius - 5
        alphaLatLeft = np.exp(-syLeft / 0.2) if syLeft > 0 else 1 - syLeft / 0.2
        alphaLatRight = np.exp(-syRight / 0.2) if syRight > 0 else 1 - syRight / 0.2
        accLatB0 = 6 * (min(alphaLatRight, 6) - min(alphaLatLeft, 6))
        accLatB = accLatB0 * (0.2 + 0.8 * self.tangential_speed / DESIRED_SPEED)
        return max(-6, min(6, accLatB))

    def _calculate_effective_radius(self):
        """Determines the effective road boundaries based on the vehicle's position."""
        if calculate_angle_gap(self.entry_angle, self.angle) < np.math.asin(10 / OUTER_RADIUS):
            effective_inner = OUTER_RADIUS - 15
            effective_outer = OUTER_RADIUS
        elif calculate_angle_gap(self.angle, self.exit_angle) < np.math.asin(30 / OUTER_RADIUS):
            effective_inner = OUTER_RADIUS - 10
            effective_outer = OUTER_RADIUS
        else:
            effective_inner = INNER_RADIUS
            effective_outer = OUTER_RADIUS
        return effective_inner, effective_outer

    def _record_state(self):
        """Appends the current state to the vehicle's history lists."""
        self.position_x.append(self.radius * np.cos(self.angle))
        self.position_y.append(self.radius * np.sin(self.angle))
        self.tangential_speeds.append(self.tangential_speed)
        self.radial_speeds.append(self.radial_speed)
        self.radius_history.append(self.radius)
        self.angle_history.append(self.angle)

    def _set_paused(self):
        """Resets the vehicle to a paused state off-screen."""
        self.paused = True
        self.out = False
        self.radius = 999
        self.angle = 0
        self.decide = 999
        self._record_state()

    def activate(self, entry_idx, exit_idx):
        """Activates a paused vehicle, setting its initial state."""
        self.paused = False
        self.entry_idx = entry_idx
        self.exit_idx = exit_idx
        self.entry_angle = ENTRY_ANGLES[entry_idx]
        self.exit_angle = EXIT_ANGLES[exit_idx]
        self.angle = self.entry_angle
        self.radius = OUTER_RADIUS + 30
        self.tangential_speed = 0
        self.radial_speed = -10
        self.decide = entry_idx + 1