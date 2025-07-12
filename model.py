import numpy as np
from config import *

# This file contains the core mathematical models for vehicle interactions,
# as described in the LFR-MPF paper.

def idm_acceleration(v, v_lead, gap, sy):
    """
    Calculates the Intelligent Driver Model (IDM) acceleration.
    This version includes a radial decay factor for lane-free interactions.
    """
    def alphalongfun(sy):
        # Radial decay factor based on lateral separation
        return min(1, np.exp(-(abs(sy) - 4.846) / RADIAL_ATTENUATION_SCALE))

    delta_v = v - v_lead
    s_star = JAM_DISTANCE + max(0, DESIRED_TIME_GAP * v + v * delta_v / (2 * np.sqrt(MAX_ACCELERATION * COMFORTABLE_DECELERATION)))
    
    interaction_term = np.maximum(-12, -alphalongfun(sy) * MAX_ACCELERATION * (s_star / max(gap, 1e-6)) ** 2)
    free_road_term = MAX_ACCELERATION * (1 - (v / DESIRED_SPEED) ** 4)
    
    acc = free_road_term + interaction_term
    return acc

def idm_acceleration1(v, v_lead, gap, sy):
    """ A variant of IDM, potentially for specific scenarios like exiting. """
    def alphalongfun(sy):
        return min(1, np.exp(-(abs(sy) - 2) / 0.6))

    delta_v = v - v_lead
    s_star = max(0, DESIRED_TIME_GAP * v + v * delta_v / (2 * np.sqrt(MAX_ACCELERATION * COMFORTABLE_DECELERATION)))
    
    interaction_term = np.maximum(-4, -alphalongfun(sy) * MAX_ACCELERATION * (s_star / max(gap, 1e-6)) ** 2)
    free_road_term = MAX_ACCELERATION * (1 - (v / DESIRED_SPEED) ** 4)
    
    acc = free_road_term + interaction_term
    if v < 1:
        acc = 0.1
    return acc

def IDM(v, v_lead, gap):
    """ A simplified IDM for entry/exit ramp behavior. """
    delta_v = v - v_lead
    s_star = 1 + max(0, 0.5 * v + v * delta_v / (4 * np.sqrt(3 * COMFORTABLE_DECELERATION)))
    
    interaction_term = np.maximum(-18, -3 * (s_star / max(gap, 1e-6)) ** 2)
    free_road_term = 5 * (1 - (v / 15) ** 4)
    
    acc = free_road_term + interaction_term
    return acc

def idm_int(v, v_lead, gap, sy):
    """ Calculates only the interaction component of the IDM. """
    def alphalongfun(sy):
        return min(1, np.exp(-(abs(sy) - 4.846) / 0.6))

    delta_v = v - v_lead
    s_star = JAM_DISTANCE + max(0, DESIRED_TIME_GAP * v + v * delta_v / (2 * np.sqrt(MAX_ACCELERATION * COMFORTABLE_DECELERATION)))
    
    acc = -(MAX_ACCELERATION * (s_star / max(gap, 1e-6)) ** 2) * alphalongfun(sy)
    return acc

def IAM(a, A, B, C, D, ego_vehicle, front_vehicle):
    """
    Calculates the Intelligent Agent Model (IAM) acceleration for the radial direction.
    """
    Wveh, Wl = ego_vehicle['width'], front_vehicle['width']
    vy = ego_vehicle['speed_y']  # Ego's radial velocity
    vy1 = front_vehicle['speed_y']  # Front vehicle's radial velocity
    dy = front_vehicle['position_y'] - ego_vehicle['position_y']  # Radial difference
    
    sign_dy = -1 if dy < 0 else 1
    Wavg = ego_vehicle['width']
    overlap = abs(dy) < Wavg
    
    alpha = sign_dy * (abs(dy) / Wavg if overlap else np.exp(-(abs(dy) - Wavg) / B))
    
    v0LatInt = A * alpha * (a - MAX_ACCELERATION * (1 - (ego_vehicle['speed_x'] / DESIRED_SPEED) ** 4))
    mult_dv_factor = 1 if overlap else max(0.0, 1.0 - C * sign_dy * (vy1 - vy))
    
    accLatInt = (v0LatInt) / D * mult_dv_factor
    
    return max(-2, min(2, accLatInt - vy / D))

