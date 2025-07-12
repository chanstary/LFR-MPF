import numpy as np
from config import *
from models import idm_int

def calculate_angle_gap(current_angle, front_angle):
    """
    Calculates the shortest positive angle difference from current_angle to front_angle.
    Args:
        current_angle (float): Current vehicle's angle in radians [0, 2π).
        front_angle (float): Front target's angle in radians [0, 2π).
    Returns:
        float: The angle gap in radians [0, 2π).
    """
    # Normalize angles to be within [0, 2π)
    current_angle = current_angle % (2 * np.pi)
    front_angle = front_angle % (2 * np.pi)

    gap_angle = front_angle - current_angle
    
    # Adjust the angle to be positive
    if gap_angle < 0:
        gap_angle += 2 * np.pi

    return gap_angle

def find_front_vehicle(vehicle, vehicles):
    """
    Finds the most influential preceding vehicle for the ego vehicle.
    The "most influential" is the one causing the strongest deceleration.
    """
    front_vehicles = []
    
    for other in vehicles:
        if other.idx == vehicle.idx or other.radius > OUTER_RADIUS:
            continue

        angle_diff = calculate_angle_gap(vehicle.angle, other.angle)
        
        # Consider only vehicles within a forward perception cone (e.g., 180 degrees)
        if angle_diff > np.pi:
            continue

        arc_length = min(vehicle.radius * angle_diff, other.radius * angle_diff)
        radius_diff = abs(vehicle.radius - other.radius)
        
        # Perception range check
        if 0 < arc_length <= 100 and radius_diff < 5:
             front_vehicles.append(other)

    if front_vehicles:
        best_vehicle = None
        min_tangential_acc = float('inf')

        for other in front_vehicles:
            angle_diff = calculate_angle_gap(vehicle.angle, other.angle)
            gap = min(vehicle.radius * angle_diff, other.radius * angle_diff) - vehicle.length
            sy = vehicle.radius - other.radius
            tangential_acc_influence = idm_int(vehicle.tangential_speed, other.tangential_speed, gap, sy)

            if tangential_acc_influence < min_tangential_acc:
                min_tangential_acc = tangential_acc_influence
                best_vehicle = other
        
        # Special condition for exiting (less yielding)
        angle_to_exit = calculate_angle_gap(vehicle.angle, vehicle.exit_angle)
        if min_tangential_acc > -2 and angle_to_exit < np.arcsin(30 / OUTER_RADIUS):
            return 1 # A flag indicating clear to exit
        else:
            return best_vehicle
    else:
        # Special condition for exiting when no vehicles are ahead
        angle_to_exit = calculate_angle_gap(vehicle.angle, vehicle.exit_angle)
        if angle_to_exit < np.arcsin(30 / OUTER_RADIUS):
            return 1 # A flag indicating clear to exit
        return None

def find_follower(vehicle, vehicles):
    """
    Finds the most influential following vehicle for the ego vehicle.
    """
    find_followers = []

    for other in vehicles:
        if other.idx == vehicle.idx:
            continue

        angle_diff = calculate_angle_gap(other.angle, vehicle.angle)

        if angle_diff <= 0 or angle_diff > np.pi:
            continue

        arc_length = min(vehicle.radius * angle_diff, other.radius * angle_diff)
        radius_diff = abs(vehicle.radius - other.radius)
        
        if 0 < arc_length <= 100 and radius_diff < vehicle.width:
            find_followers.append(other)

    if find_followers:
        best_vehicle = None
        min_tangential_acc = float('inf')

        for other in find_followers:
            angle_diff = calculate_angle_gap(other.angle, vehicle.angle)
            gap = min(vehicle.radius * angle_diff, other.radius * angle_diff) - other.length
            sy = other.radius - vehicle.radius
            tangential_acc_influence = idm_int(other.tangential_speed, vehicle.tangential_speed, gap, sy)

            if tangential_acc_influence < min_tangential_acc:
                min_tangential_acc = tangential_acc_influence
                best_vehicle = other
        return best_vehicle
    else:
        return None
