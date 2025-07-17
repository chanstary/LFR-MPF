# LFR-MPF-Simulation/main.py

import numpy as np
import time

# Import from project modules
from config import *
from vehicle import Vehicle
from utils import find_approaching_leader, find_leader_in_roundabout, find_follower_in_roundabout
from visualization import animate_simulation

def run_simulation():
    """
    Initializes and runs the main simulation loop.
    """
    # --- Initialization ---
    vehicles = [
        Vehicle(
            idx=i, 
            entry_angle=0, 
            exit_angle=0, 
            entry_idx=-1, 
            exit_idx=-1
        ) for i in range(NUM_VEHICLES)
    ]
    
    # Store history for visualization
    vehicle_positions = {f"Veh[{v.idx}]": {
        "position_x": v.position_x,
        "position_y": v.position_y,
        "tangential_speeds": v.tangential_speeds,
        "radial_speeds": v.radial_speeds,
        "exit_idx": [v.exit_idx],
        "entry_idx": [v.entry_idx],
    } for v in vehicles}

    num_steps = int(TOTAL_TIME / DT)
    last_spawn_time = -FLOW_RATE
    spawned_count = 0

    # --- Main Simulation Loop ---
    for t_step in range(num_steps):
        current_time = t_step * DT
        print(f"Simulating time: {current_time:.1f}s / {TOTAL_TIME}s")

        # --- Spawn New Vehicles ---
        if current_time - last_spawn_time >= FLOW_RATE and spawned_count < NUM_VEHICLES:
            paused_vehicles = [v for v in vehicles if v.paused]
            if paused_vehicles:
                vehicle_to_spawn = paused_vehicles[0]
                entry_idx = np.random.randint(0, len(ENTRY_ANGLES))
                exit_idx = np.random.randint(0, len(EXIT_ANGLES))
                while entry_idx == exit_idx: # Ensure entry and exit are different
                    exit_idx = np.random.randint(0, len(EXIT_ANGLES))
                
                vehicle_to_spawn.activate(entry_idx, exit_idx)
                spawned_count += 1
                last_spawn_time = current_time

        # --- Update Vehicles ---
        active_vehicles = [v for v in vehicles if not v.paused]
        
        for vehicle in active_vehicles:
            vehicle.update_decision()
            
            if vehicle.radius > OUTER_RADIUS:
                leader = find_approaching_leader(vehicle, active_vehicles)
                follower = None
            else:
                leader = find_leader_in_roundabout(vehicle, active_vehicles)
                follower = find_follower_in_roundabout(vehicle, active_vehicles)
            
            vehicle.update(leader, follower)
        
        # --- Update Paused Vehicles (to keep history lists consistent) ---
        for vehicle in vehicles:
            if vehicle.paused:
                if len(vehicle.position_x) < t_step + 2:
                    vehicle._set_paused()

    print("Simulation finished.")

    # --- Prepare Data for Visualization ---
    final_vehicle_positions = {}
    for v in vehicles:
        final_vehicle_positions[f"Veh[{v.idx}]"] = {
            "position_x": v.position_x,
            "position_y": v.position_y,
            "tangential_speeds": v.tangential_speeds,
            "radial_speeds": v.radial_speeds,
            "exit_idx": [v.exit_idx] * len(v.position_x),
            "entry_idx": [v.entry_idx] * len(v.position_x),
        }

    return final_vehicle_positions


if __name__ == '__main__':
    # Run the simulation
    simulation_data = run_simulation()
    
    # Animate the results
    # Note: This requires a graphical backend and is best run in environments like Jupyter
    try:
        animate_simulation(simulation_data)
    except Exception as e:
        print(f"Could not run animation, likely due to missing graphical backend.")
        print(f"Error: {e}")
        print("Simulation data generation is complete.")