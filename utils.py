# LFR-MPF-Simulation/utils.py

import numpy as np
from .config import *
from .models import idm_interaction_deceleration, idm_acceleration
# Forward declaration to avoid circular import
class Vehicle: pass


def calculate_angle_gap(start_angle, end_angle):
    """Calculates the shortest positive angle from start_angle to end_angle."""
    return (end_angle - start_angle + 2 * np.pi) % (2 * np.pi)

def find_approaching_leader(vehicle, vehicles):
    """Finds the closest vehicle directly ahead when entering the roundabout."""
    front_vehicles = []
    for other in vehicles:
        if other.idx != vehicle.idx and other.radius < vehicle.radius:
            is_in_lane = (vehicle.angle == other.angle and other.radius > OUTER_RADIUS)
            is_at_entry = (calculate_angle_gap(vehicle.angle, other.angle) < np.pi/18 and
                           OUTER_RADIUS - 5 < other.radius < OUTER_RADIUS)
            if is_in_lane or is_at_entry:
                front_vehicles.append(other)
    return min(front_vehicles, key=lambda v: vehicle.radius - v.radius) if front_vehicles else None

def find_leader_in_roundabout(vehicle, vehicles):
    """Finds the leading vehicle inside the roundabout."""
    front_vehicles = []
    angle_to_exit = calculate_angle_gap(vehicle.angle, vehicle.exit_angle)
    for other in vehicles:
        if other.idx == vehicle.idx or other.radius > OUTER_RADIUS:
            continue
        angle_diff = calculate_angle_gap(vehicle.angle, other.angle)
        arc_length = min(vehicle.radius, other.radius) * angle_diff
        radius_diff = abs(vehicle.radius - other.radius)
        if 0 < angle_diff < angle_to_exit and 0 < arc_length <= 100 and radius_diff < 5:
            front_vehicles.append(other)
    if not front_vehicles:
        return None
    best_vehicle = min(front_vehicles, key=lambda v:
        idm_interaction_deceleration(vehicle.tangential_speed, v.tangential_speed,
                                     min(vehicle.radius, v.radius) * calculate_angle_gap(vehicle.angle, v.angle) - vehicle.length,
                                     vehicle.radius - v.radius)
    )
    min_decel = idm_interaction_deceleration(vehicle.tangential_speed, best_vehicle.tangential_speed,
                                            min(vehicle.radius, best_vehicle.radius) * calculate_angle_gap(vehicle.angle, best_vehicle.angle) - vehicle.length,
                                            vehicle.radius - best_vehicle.radius)
    if min_decel > -2 and angle_to_exit < np.math.asin(30 / OUTER_RADIUS):
        return Vehicle.YIELD_FLAG
    return best_vehicle

def find_follower_in_roundabout(vehicle, vehicles):
    """Finds the following vehicle inside the roundabout."""
    followers = []
    for other in vehicles:
        if other.idx == vehicle.idx:
            continue
        angle_diff = calculate_angle_gap(other.angle, vehicle.angle)
        if not (0 < angle_diff < np.pi):
            continue
        arc_length = min(vehicle.radius, other.radius) * angle_diff
        radius_diff = abs(vehicle.radius - other.radius)
        if 0 < arc_length <= 100 and radius_diff < vehicle.length:
            followers.append(other)
    if not followers:
        return None
    return min(followers, key=lambda v:
        idm_interaction_deceleration(v.tangential_speed, vehicle.tangential_speed,
                                     min(vehicle.radius, v.radius) * calculate_angle_gap(v.angle, vehicle.angle) - vehicle.length,
                                     v.radius - vehicle.radius)
    )