import numpy as np
import matplotlib.pyplot as plt
from config import *
from vehicle import Vehicle
from utils import find_front_vehicle, find_follower
from visualization import plot_trajectories, plot_final_positions

def setup_simulation():
    """Initializes vehicles for the simulation."""
    vehicles = []
    
    # Define entry and exit nodes around the roundabout
    node_angles = np.linspace(0, 2 * np.pi, NUM_NODES, endpoint=False)
    
    for i in range(NUM_VEHICLES):
        # Assign random entry and exit nodes
        entry_idx = np.random.randint(0, NUM_NODES)
        exit_idx = (entry_idx + np.random.randint(1, NUM_NODES)) % NUM_NODES
        
        entry_angle = node_angles[entry_idx]
        exit_angle = node_angles[exit_idx]
        
        # Place vehicles on the entry ramp
        initial_radius = OUTER_RADIUS + 10 + i * 15 # Stagger vehicles on the ramp
        initial_angle = entry_angle
        
        v = Vehicle(i, initial_angle, initial_radius, entry_angle, exit_angle)
        v.entry_idx = entry_idx
        v.exit_idx = exit_idx
        vehicles.append(v)
        
    return vehicles

def run_simulation():
    """Main simulation loop."""
    vehicles = setup_simulation()
    
    num_steps = int(SIMULATION_TIME / DT)
    
    for step in range(num_steps):
        if step % 100 == 0:
            print(f"--- Step {step}/{num_steps} ---")

        # Update each vehicle
        for v in vehicles:
            if v.out: # Skip vehicles that have exited
                continue
            
            # Find interacting vehicles
            front_vehicle = find_front_vehicle(v, vehicles)
            follower_vehicle = find_follower(v, vehicles)
            
            # Update vehicle state
            v.update(front_vehicle, follower_vehicle, DT)

        # Optional: Print status of a vehicle
        if step % 100 == 0:
            print(vehicles[0])
            
    print("--- Simulation Finished ---")
    return vehicles

if __name__ == "__main__":
    # Run the simulation
    final_vehicles = run_simulation()
    
    # Visualize the results
    plot_trajectories(final_vehicles)
    plot_final_positions(final_vehicles)
    plt.show()

