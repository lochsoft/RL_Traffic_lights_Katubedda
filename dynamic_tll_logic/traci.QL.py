""" # Step 1: Add modules to provide access to specific libraries and functions
import os  # Module provides functions to handle file paths, directories, environment variables
import sys  # Module provides access to Python-specific system parameters and functions
import random
import numpy as np
import matplotlib.pyplot as plt  # Visualization

# Step 2: Establish path to SUMO (SUMO_HOME)
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")
2
# Step 3: Add Traci module to provide access to specific libraries and functions
import traci  # Static network information (such as reading and analyzing network files)

# Step 4: Define Sumo configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'simulation_katubedda_junction_dynamic.sumocfg',
    '--step-length', '0.10',
    '--delay', '200',
    '--lateral-resolution', '0'
]

# Step 5: Open connection between SUMO and Traci
traci.start(Sumo_config)
traci.gui.setSchema("View #0", "real world")

# -------------------------
# Step 6: Define Variables
# -------------------------

# Variables for RL State (queue lengths from detectors and current phase)
q_EB_0 = 0
q_EB_1 = 0
q_EB_2 = 0
q_EB_3 = 0
q_SB_0 = 0
q_SB_1 = 0
q_SB_2 = 0
q_SB_3 = 0
q_WB_0 = 0
q_WB_1 = 0
current_phase = 0

# ---- Reinforcement Learning Hyperparameters ----
TOTAL_STEPS = 10000    # The total number of simulation steps for continuous (online) training.

ALPHA = 0.1            # Learning rate (α) between[0, 1]    #If α = 1, you fully replace the old Q-value with the newly computed estimate.
                                                            #If α = 0, you ignore the new estimate and never update the Q-value.
GAMMA = 0.9            # Discount factor (γ) between[0, 1]  #If γ = 0, the agent only cares about the reward at the current step (no future rewards).
                                                            #If γ = 1, the agent cares equally about current and future rewards, looking at long-term gains.
EPSILON = 0.1          # Exploration rate (ε) between[0, 1] #If ε = 0 means very greedy, if=1 means very random

ACTIONS = [0, 1]       # The discrete action space (0 = keep phase, 1 = switch phase)

# Q-table dictionary: key = state tuple, value = numpy array of Q-values for each action
Q_table = {}

# ---- Additional Stability Parameters ----
MIN_GREEN_STEPS = 100
last_switch_step = -MIN_GREEN_STEPS

# -------------------------
# Step 7: Define Functions
# -------------------------

def get_max_Q_value_of_state(s): #1. Objective Function 1
    if s not in Q_table:
        Q_table[s] = np.zeros(len(ACTIONS))
    return np.max(Q_table[s])

def get_reward(state):  #2. Constraint 2 

    total_queue = sum(state[:-1])  # Exclude the current_phase element
    reward = -float(total_queue)
    return reward

def get_state():  #3.& 4. Constraint 3 & 4
    global q_EB_0, q_EB_1, q_EB_2, q_EB_3, q_SB_0, q_SB_1, q_SB_2, q_SB_3, q_WB_0, q_WB_1, current_phase
    
    # Detector IDs for CMB_to_KBJ_001
    detector_CMB_to_KBJ_001_1 = "CMB_to_KBJ_001_1"
    detector_CMB_to_KBJ_001_2 = "CMB_to_KBJ_001_2"
    detector_CMB_to_KBJ_001_3 = "CMB_to_KBJ_001_3"
    detector_CMB_to_KBJ_001_4 = "CMB_to_KBJ_001_4"
    
    # Detector IDs for MRT_to_KB_001
    detector_MRT_to_KB_001_2 = "MRT_to_KB_001.37_2"
    detector_MRT_to_KB_001_3 = "MRT_to_KB_001.37_3"
    detector_MRT_to_KB_001_4 = "MRT_to_KB_001.37_4"
    detector_MRT_to_KB_001_5 = "MRT_to_KB_001.37_5"
    
    detector_P_to_KBJ_1 = "P_to_KBJ_2"
    detector_P_to_KBJ_2 = "P_to_KBJ_1"

    # Traffic light ID
    traffic_light_id = "KB_Junction"
    
    # Get queue lengths from each detector
    q_EB_0 = get_queue_length(detector_CMB_to_KBJ_001_1)
    q_EB_1 = get_queue_length(detector_CMB_to_KBJ_001_2)
    q_EB_2 = get_queue_length(detector_CMB_to_KBJ_001_3)
    q_EB_3 = get_queue_length(detector_CMB_to_KBJ_001_4)
    
    q_SB_0 = get_queue_length(detector_MRT_to_KB_001_2)
    q_SB_1 = get_queue_length(detector_MRT_to_KB_001_3)
    q_SB_2 = get_queue_length(detector_MRT_to_KB_001_4)
    q_SB_3 = get_queue_length(detector_MRT_to_KB_001_5)

    q_WB_0 = get_queue_length(detector_P_to_KBJ_1)
    q_WB_1 = get_queue_length(detector_P_to_KBJ_2)
    
    # Get current phase index
    current_phase = get_current_phase(traffic_light_id)
    
    return (q_EB_0, q_EB_1, q_EB_2, q_EB_3, q_SB_0, q_SB_1, q_SB_2, q_SB_3, q_WB_0, q_WB_1, current_phase)

def apply_action(action, tls_id="KB_Junction"): #5. Constraint 5

    global last_switch_step
    
    if action == 0:
        # Do nothing (keep current phase)
        return
    
    elif action == 1:
        # Check if minimum green time has passed before switching
        if current_simulation_step - last_switch_step >= MIN_GREEN_STEPS:
            program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
            num_phases = len(program.phases)
            next_phase = (get_current_phase(tls_id) + 1) % num_phases
            traci.trafficlight.setPhase(tls_id, next_phase)
            last_switch_step = current_simulation_step

def update_Q_table(old_state, action, reward, new_state): #6. Constraint 6
    if old_state not in Q_table:
        Q_table[old_state] = np.zeros(len(ACTIONS))
    
    # 1) Predict current Q-values from old_state (current state)
    old_q = Q_table[old_state][action]
    # 2) Predict Q-values for new_state to get max future Q (new state)
    best_future_q = get_max_Q_value_of_state(new_state)
    # 3) Incorporate ALPHA to partially update the Q-value and update Q table
    Q_table[old_state][action] = old_q + ALPHA * (reward + GAMMA * best_future_q - old_q)

def get_action_from_policy(state): #7. Constraint 7
    if random.random() < EPSILON:
        return random.choice(ACTIONS)
    else:
        if state not in Q_table:
            Q_table[state] = np.zeros(len(ACTIONS))
        return int(np.argmax(Q_table[state]))

def get_queue_length(detector_id): #8.Constraint 8
    return traci.lanearea.getLastStepVehicleNumber(detector_id)

def get_current_phase(tls_id): #8.Constraint 8
    return traci.trafficlight.getPhase(tls_id)

# -------------------------
# Step 8: Fully Online Continuous Learning Loop
# -------------------------

# Lists to record data for plotting
step_history = []
reward_history = []
queue_history = []

cumulative_reward = 0.0

print("\n=== Starting Fully Online Continuous Learning ===")
for step in range(TOTAL_STEPS):
    current_simulation_step = step
    
    state = get_state()
    action = get_action_from_policy(state)
    apply_action(action)
    
    traci.simulationStep()  # Advance simulation by one step
    
    new_state = get_state()
    reward = get_reward(new_state)
    cumulative_reward += reward
    
    update_Q_table(state, action, reward, new_state)
    
    # Print Q-values for the old_state right after update
    updated_q_vals = Q_table[state]

    # Record data every 100 steps
    if step % 1 == 0:
        print(f"Step {step}, Current_State: {state}, Action: {action}, New_State: {new_state}, Reward: {reward:.2f}, Cumulative Reward: {cumulative_reward:.2f}, Q-values(current_state): {updated_q_vals}")
        step_history.append(step)
        reward_history.append(cumulative_reward)
        queue_history.append(sum(new_state[:-1]))  # sum of queue lengths
        print("Current Q-table:")
        for st, qvals in Q_table.items():
            print(f"  {st} -> {qvals}")
     
# -------------------------
# Step 9: Close connection between SUMO and Traci
# -------------------------
traci.close()

# Print final Q-table info
print("\nOnline Training completed. Final Q-table size:", len(Q_table))
for st, actions in Q_table.items():
    print("State:", st, "-> Q-values:", actions)

# -------------------------
# Visualization of Results
# -------------------------

# Plot Cumulative Reward over Simulation Steps
plt.figure(figsize=(10, 6))
plt.plot(step_history, reward_history, marker='o', linestyle='-', label="Cumulative Reward")
plt.xlabel("Simulation Step")
plt.ylabel("Cumulative Reward")
plt.title("RL Training: Cumulative Reward over Steps")
plt.legend()
plt.grid(True) 
plt.show()

# Plot Total Queue Length over Simulation Steps
plt.figure(figsize=(10, 6))
plt.plot(step_history, queue_history, marker='o', linestyle='-', label="Total Queue Length")
plt.xlabel("Simulation Step")
plt.ylabel("Total Queue Length")
plt.title("RL Training: Queue Length over Steps")
plt.legend()
plt.grid(True)
plt.show() """

import os
import pickle
import sys
import random
import numpy as np
import matplotlib.pyplot as plt

# Set up SUMO_HOME
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# SUMO configuration
Sumo_config = [
    'sumo-gui',
    '-c', 'simulation_katubedda_junction_dynamic.sumocfg',
    '--step-length', '1.0',  # Increased to 1s for efficiency
    '--delay', '200',
    '--lateral-resolution', '0'
]

# Variables for RL State
q_EB_0 = q_EB_1 = q_EB_2 = q_EB_3 = 0
q_SB_0 = q_SB_1 = q_SB_2 = q_SB_3 = 0
q_WB_0 = q_WB_1 = 0
current_phase = 0

# RL Hyperparameters
START = 900
TOTAL_STEPS = 4500  # 1.15 hour (5.45-7 AM)
ALPHA = 0.1
GAMMA = 0.9
EPSILON = 0.1
ACTIONS = [0, 1]  # 0: keep phase, 1: switch phase
Q_table = {}
MIN_GREEN_STEPS = 100
last_switch_step = -MIN_GREEN_STEPS

# Detector configuration for flexibility
DETECTORS = {
    "EB": ["CMB_to_KBJ_001_1", "CMB_to_KBJ_001_2", "CMB_to_KBJ_001_3", "CMB_to_KBJ_001_4"],
    "SB": ["MRT_to_KB_001.37_2", "MRT_to_KB_001.37_3", "MRT_to_KB_001.37_4", "MRT_to_KB_001.37_5"],
    "WB": ["P_to_KBJ_2", "P_to_KBJ_1"]
}

# Function Definitions
def get_queue_length(detector_id):
    try:
        return traci.lanearea.getLastStepVehicleNumber(detector_id)
    except traci.TraCIException:
        print(f"Warning: Detector {detector_id} not found.")
        return 0

def get_current_phase(tls_id):
    return traci.trafficlight.getPhase(tls_id)

def discretize_queue(q):
    return 0 if q <= 2 else 1 if q <= 5 else 2

def get_max_Q_value_of_state(s):
    if s not in Q_table:
        Q_table[s] = np.zeros(len(ACTIONS))
    return np.max(Q_table[s])

def get_reward(state):
    total_queue = sum(state[:-1])
    return -float(total_queue)

def get_state():
    global q_EB_0, q_EB_1, q_EB_2, q_EB_3, q_SB_0, q_SB_1, q_SB_2, q_SB_3, q_WB_0, q_WB_1, current_phase
    state = []
    for direction in DETECTORS.values():
        for det in direction:
            state.append(discretize_queue(get_queue_length(det)))
    state.append(get_current_phase("KB_Junction"))
    return tuple(state)

def apply_action(action, tls_id="KB_Junction"):
    global last_switch_step
    if action == 0:
        return
    elif action == 1:
        if current_simulation_step - last_switch_step >= MIN_GREEN_STEPS:
            program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
            num_phases = len(program.phases)
            next_phase = (get_current_phase(tls_id) + 1) % num_phases
            traci.trafficlight.setPhase(tls_id, next_phase)
            last_switch_step = current_simulation_step

def update_Q_table(old_state, action, reward, new_state):
    if old_state not in Q_table:
        Q_table[old_state] = np.zeros(len(ACTIONS))
    old_q = Q_table[old_state][action]
    best_future_q = get_max_Q_value_of_state(new_state)
    Q_table[old_state][action] = old_q + ALPHA * (reward + GAMMA * best_future_q - old_q)

def get_action_from_policy(state):
    if random.random() < EPSILON:
        return random.choice(ACTIONS)
    else:
        if state not in Q_table:
            Q_table[state] = np.zeros(len(ACTIONS))
        return int(np.argmax(Q_table[state]))

# Main RL Loop
step_history = []
reward_history = []
queue_history = []
cumulative_reward = 0.0

try:
    traci.start(Sumo_config)
    traci.gui.setSchema("View #0", "real world")
    print("\n=== Starting RL Training ===")
    for step in range(START, TOTAL_STEPS + 1):
        if traci.simulation.getMinExpectedNumber() == 0:
            print("No vehicles left. Ending simulation.")
            break
        current_simulation_step = step
        state = get_state()
        action = get_action_from_policy(state)
        apply_action(action)
        traci.simulationStep()
        new_state = get_state()
        reward = get_reward(new_state)
        cumulative_reward += reward
        update_Q_table(state, action, reward, new_state)
        if step % 100 == 0:
            print(f"Step {step}, State: {state}, Action: {action}, Reward: {reward:.2f}, Cumulative Reward: {cumulative_reward:.2f}, Q-values: {Q_table[state]}")
            step_history.append(step)
            reward_history.append(cumulative_reward)
            queue_history.append(sum(new_state[:-1]))
finally:
    traci.close()
    os.system("pkill sumo")  # Force-kill SUMO if needed

# Save Q-table
with open("q_table.pkl", "wb") as f:
    pickle.dump(Q_table, f)

# Print final Q-table
print("\nTraining completed. Q-table size:", len(Q_table))
for st, actions in Q_table.items():
    print(f"State: {st} -> Q-values: {actions}")

# Visualization
plt.figure(figsize=(10, 6))
plt.plot(step_history, reward_history, marker='o', linestyle='-', label="Cumulative Reward")
plt.xlabel("Simulation Step")
plt.ylabel("Cumulative Reward")
plt.title("RL Training: Cumulative Reward over Steps")
plt.legend()
plt.grid(True)
plt.show()

plt.figure(figsize=(10, 6))
plt.plot(step_history, queue_history, marker='o', linestyle='-', label="Total Queue Length")
plt.xlabel("Simulation Step")
plt.ylabel("Total Queue Length")
plt.title("RL Training: Queue Length over Steps")
plt.legend()
plt.grid(True)
plt.show()