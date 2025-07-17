# LFR-MPF-Simulation/models.py

import numpy as np
from .config import *

def idm_acceleration(v, v_lead, gap, sy):
    """IDM calculation for tangential acceleration with lateral influence."""
    alphalongfun = min(1, np.exp(-(abs(sy) - 4.846) / 0.6))
    delta_v = v - v_lead
    s_star = MIN_SAFE_DISTANCE + max(0, TIME_HEADWAY * v + v * delta_v / (2 * np.sqrt(MAX_ACCELERATION * COMFORTABLE_DECELERATION)))
    interaction_term = -alphalongfun * MAX_ACCELERATION * (s_star / gap) ** 2
    acc = MAX_ACCELERATION * (1 - (v / DESIRED_SPEED) ** 4) + np.maximum(-12, interaction_term)
    return acc

def idm_exit_approach(v, v_lead, gap, sy):
    """Variant of IDM for a vehicle approaching its exit."""
    alphalongfun = min(1, np.exp(-(abs(sy) - 2) / 0.6))
    delta_v = v - v_lead
    s_star = max(0, TIME_HEADWAY * v + v * delta_v / (2 * np.sqrt(MAX_ACCELERATION * COMFORTABLE_DECELERATION)))
    interaction_term = -alphalongfun * MAX_ACCELERATION * (s_star / gap) ** 2
    acc = MAX_ACCELERATION * (1 - (v / DESIRED_SPEED) ** 4) + np.maximum(-4, interaction_term)
    if v < 1:
        acc = 0.1
    return acc

def idm_entry_acceleration(v, v_lead, gap):
    """A simplified IDM for vehicles entering the roundabout."""
    delta_v = v - v_lead
    s_star = 1 + max(0, 0.5 * v + v * delta_v / (4 * np.sqrt(3 * COMFORTABLE_DECELERATION)))
    interaction_term = -3 * (s_star / gap) ** 2
    acc = 5 * (1 - (v / 15) ** 4) + np.maximum(-18, interaction_term)
    return acc

def idm_interaction_deceleration(v, v_lead, gap, sy):
    """Calculates only the deceleration interaction term of the IDM."""
    alphalongfun = min(1, np.exp(-(abs(sy) - 4.846) / 0.6))
    delta_v = v - v_lead
    s_star = MIN_SAFE_DISTANCE + max(0, TIME_HEADWAY * v + v * delta_v / (2 * np.sqrt(MAX_ACCELERATION * COMFORTABLE_DECELERATION)))
    acc = -(MAX_ACCELERATION * (s_star / gap) ** 2) * alphalongfun
    return acc

def iam_radial_acceleration(a, A, B, C, D, ego_vehicle, front_vehicle):
    """IAM calculation for radial acceleration."""
    Wveh, Wl = ego_vehicle['width'], front_vehicle['width']
    vy, vy1 = ego_vehicle['speed_y'], front_vehicle['speed_y']
    dy = front_vehicle['position_y'] - ego_vehicle['position_y']
    sign_dy = np.sign(dy)
    Wavg = ego_vehicle['width']
    overlap = abs(dy) < Wavg

    alpha = sign_dy * (abs(dy) / Wavg if overlap else np.exp(-(abs(dy) - Wavg) / B))
    v0LatInt = A * alpha * (a - MAX_ACCELERATION * (1 - (ego_vehicle['speed_x'] / DESIRED_SPEED) ** 4))
    mult_dv_factor = 1 if overlap else max(0.0, 1.0 - C * sign_dy * (vy1 - vy))
    accLatInt = (v0LatInt) / D * mult_dv_factor
    return max(-2, min(2, accLatInt - vy / D))