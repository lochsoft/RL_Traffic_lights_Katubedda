# dqn_katubedda.py
import os, sys, argparse, random, math, collections, time
import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=224178)
parser.add_argument("--episodes", type=int, default=1)        # 1 long episode is fine; increase if you want resets
parser.add_argument("--max_steps", type=int, default=6000)    # safety cap
parser.add_argument("--min_green", type=int, default=30)
parser.add_argument("--switch_penalty", type=float, default=2.0)
parser.add_argument("--lr", type=float, default=1e-3)
parser.add_argument("--gamma", type=float, default=0.99)
parser.add_argument("--batch_size", type=int, default=128)
parser.add_argument("--buffer_size", type=int, default=50000)
parser.add_argument("--start_epsilon", type=float, default=0.3)
parser.add_argument("--end_epsilon", type=float, default=0.05)
parser.add_argument("--epsilon_decay_steps", type=int, default=3000)
parser.add_argument("--target_update", type=int, default=500) # steps between target sync
args = parser.parse_args()

# ----- Reproducibility -----
random.seed(args.seed); np.random.seed(args.seed)

# ----- SUMO tools path -----
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")
import traci

# ----- PyTorch imports -----
import torch
import torch.nn as nn
import torch.optim as optim

ACTIONS = [0, 1]  # 0 = keep phase, 1 = switch phase

class ReplayBuffer:
    def __init__(self, capacity):
        self.buf = collections.deque(maxlen=capacity)
    def push(self, s, a, r, ns, d):
        self.buf.append((s, a, r, ns, d))
    def sample(self, bs):
        batch = random.sample(self.buf, bs)
        s, a, r, ns, d = map(np.array, zip(*batch))
        return (torch.as_tensor(s, dtype=torch.float32),
                torch.as_tensor(a, dtype=torch.int64),
                torch.as_tensor(r, dtype=torch.float32),
                torch.as_tensor(ns, dtype=torch.float32),
                torch.as_tensor(d, dtype=torch.float32))
    def __len__(self): return len(self.buf)

class QNet(nn.Module):
    def __init__(self, in_dim, n_actions):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(in_dim, 128), nn.ReLU(),
            nn.Linear(128, 128), nn.ReLU(),
            nn.Linear(128, n_actions)
        )
    def forward(self, x): return self.net(x)

def make_output_dir():
    out_dir = f"outputs/dynamic_DQL_vehicle_data"
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(f"{out_dir}/plots", exist_ok=True)
    return out_dir

def start_sumo(seed, out_dir):
    queue_xml = f"{out_dir}/{args.seed}.dynamic_DQL_vehicle_data.xml"
    trip_xml  = f"{out_dir}/{args.seed}.tripinfo_dynamic_DQL_vehicle_data.xml"
    cfg = [
        "sumo",
        "-c", "simulation_katubedda_junction_dynamic.sumocfg",
        "--queue-output", queue_xml,
        "--queue-output.period", "300",
        "--tripinfo-output", trip_xml,
        "--seed", str(seed)
    ]
    traci.start(cfg)

def get_phase_count(tls_id="KB_Junction"):
    program = traci.trafficlight.getAllProgramLogics(tls_id)[0]
    return len(program.phases)

def get_state_vec(det_ids, phases, last_switch_step, step):
    # queues
    q = np.array([traci.lanearea.getLastStepVehicleNumber(d) for d in det_ids], dtype=np.float32)
    # normalize queues by a rough cap to stabilize (use 20 as typical lane queue cap; tune if needed)
    q_norm = np.clip(q / 20.0, 0.0, 1.5)

    current_phase = traci.trafficlight.getPhase("KB_Junction")
    phase_one_hot = np.zeros(phases, dtype=np.float32)
    phase_one_hot[current_phase] = 1.0

    time_since_switch = max(0, step - last_switch_step)
    t_norm = np.array([min(time_since_switch / 60.0, 5.0)], dtype=np.float32)  # 0..~5 (≈ up to 300s)

    state = np.concatenate([q_norm, phase_one_hot, t_norm], axis=0)
    return state

def main():
    out_dir = make_output_dir()
    start_sumo(args.seed, out_dir)

    det_ids = [
        "CMB_to_KBJ_001_1","CMB_to_KBJ_001_2","CMB_to_KBJ_001_3","CMB_to_KBJ_001_4",
        "MRT_to_KB_001.37_2","MRT_to_KB_001.37_3","MRT_to_KB_001.37_4","MRT_to_KB_001.37_5",
        "P_to_KBJ_1","P_to_KBJ_2"
    ]
    phases = get_phase_count("KB_Junction")

    in_dim = len(det_ids) + phases + 1
    n_actions = len(ACTIONS)
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    online = QNet(in_dim, n_actions).to(device)
    target = QNet(in_dim, n_actions).to(device)
    target.load_state_dict(online.state_dict())
    optimizer = optim.Adam(online.parameters(), lr=args.lr)
    buffer = ReplayBuffer(args.buffer_size)

    epsilon = args.start_epsilon
    eps_decay = (args.start_epsilon - args.end_epsilon) / max(1, args.epsilon_decay_steps)

    min_green = args.min_green
    last_switch_step = -min_green

    step_global = 0
    reward_hist, queue_hist, step_hist = [], [], []

    print("\n=== Starting Double DQN ===")
    for ep in range(args.episodes):
        step = 0
        # episode runs until no vehicles are left or cap reached
        while traci.simulation.getMinExpectedNumber() > 0:
            state = get_state_vec(det_ids, phases, last_switch_step, step)
            # ε-greedy
            if random.random() < epsilon:
                action = random.choice(ACTIONS)
            else:
                with torch.no_grad():
                    a = online(torch.as_tensor(state, dtype=torch.float32, device=device).unsqueeze(0)).argmax(dim=1).item()
                action = int(a)

            switched = False
            if action == 1 and (step - last_switch_step) >= min_green:
                tls_id = "KB_Junction"
                prog = traci.trafficlight.getAllProgramLogics(tls_id)[0]
                num_phases = len(prog.phases)
                next_phase = (traci.trafficlight.getPhase(tls_id) + 1) % num_phases
                traci.trafficlight.setPhase(tls_id, next_phase)
                last_switch_step = step
                switched = True

            # advance simulation
            traci.simulationStep()

            next_state = get_state_vec(det_ids, phases, last_switch_step, step+1)
            total_queue = sum([traci.lanearea.getLastStepVehicleNumber(d) for d in det_ids])
            reward = -( total_queue + (args.switch_penalty if switched else 0.0) )
            done = (traci.simulation.getMinExpectedNumber() == 0) or (step+1 >= args.max_steps)

            buffer.push(state, action, reward, next_state, float(done))

            # train if we have enough samples
            if len(buffer) >= args.batch_size:
                s, a, r, ns, d = buffer.sample(args.batch_size)
                s = s.to(device); ns = ns.to(device)
                a = a.to(device); r = r.to(device); d = d.to(device)

                # Q(s,a)
                q_sa = online(s).gather(1, a.view(-1,1)).squeeze(1)

                # Double DQN target:
                with torch.no_grad():
                    next_actions = online(ns).argmax(dim=1, keepdim=True)
                    q_ns_target = target(ns).gather(1, next_actions).squeeze(1)
                    target_q = r + args.gamma * (1.0 - d) * q_ns_target

                loss = nn.functional.smooth_l1_loss(q_sa, target_q)
                optimizer.zero_grad(); loss.backward(); optimizer.step()

                # target sync
                if step_global % args.target_update == 0:
                    target.load_state_dict(online.state_dict())

            # ε-decay
            if epsilon > args.end_epsilon:
                epsilon -= eps_decay

            # lightweight logging
            if step_global % 50 == 0:
                reward_hist.append(reward if len(reward_hist)==0 else reward_hist[-1] + reward)
                queue_hist.append(total_queue)
                step_hist.append(step_global)
                print(f"step={step_global:5d}  eps={epsilon:.2f}  queue={total_queue}  rew={reward:.1f}")

            step += 1
            step_global += 1

    traci.close()
    # plots
    plt.figure(figsize=(10,4))
    plt.plot(step_hist, reward_hist, label="Cumulative Reward")
    plt.grid(True); plt.legend(); plt.title("Double DQN Training"); plt.xlabel("Step"); plt.ylabel("Cum. Reward")
    plt.savefig(f"{out_dir}/plots/{args.seed}.dqn_cum_reward.png")

    plt.figure(figsize=(10,4))
    plt.plot(step_hist, queue_hist, label="Total Queue Length")
    plt.grid(True); plt.legend(); plt.title("Queue Length over Time"); plt.xlabel("Step"); plt.ylabel("Vehicles")
    plt.savefig(f"{out_dir}/plots/{args.seed}.dqn_queue.png")

    # save model
    torch.save(online.state_dict(), f"{out_dir}/{args.seed}.double_dqn.pt")
    print("\nTraining finished. Model saved.")

if __name__ == "__main__":
    main()


'''# Step 1: Add modules to provide access to specific libraries and functions
import os  # Module provides functions to handle file paths, directories, environment variables
import sys  # Module provides access to Python-specific system parameters and functions
import random
import numpy as np
import matplotlib.pyplot as plt  # Visualization
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=None, help="Random seed for reproducibility")
args = parser.parse_args()

# Set random seeds (for Python and NumPy)
if args.seed is not None:
    print(f"Using random seed: {args.seed}")
    random.seed(args.seed)
    np.random.seed(args.seed)

def run_traciDQL_simulation():  # <--- wrap entire logic here

    # Step 2: Establish path to SUMO (SUMO_HOME)
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("Please declare environment variable 'SUMO_HOME'")

    # Step 3: Add Traci module to provide access to specific libraries and functions
    import traci  # Static network information (such as reading and analyzing network files)

    output_file = f"outputs/dynamic_DQL_vehicle_data/{args.seed}.dynamic_DQL_vehicle_data.xml"
    output_tripinfo = f"outputs/dynamic_DQL_vehicle_data/{args.seed}.tripinfo_dynamic_DQL_vehicle_data.xml"
    # Step 4: Define Sumo configuration
    Sumo_config = [
        'sumo',
        '-c', 'simulation_katubedda_junction_dynamic.sumocfg',
        '--queue-output', output_file,
        '--queue-output.period', '300',
        '--tripinfo-output', output_tripinfo,
        '--random', 'true',
        '--seed', '224178'
    ]

    # Step 5: Open connection between SUMO and Traci
    traci.start(Sumo_config)

    # -------------------------
    # Step 6: Define Variables
    # -------------------------

    # Variables for RL State (queue lengths from detectors and current phase)
    global q_EB_0, q_EB_1, q_EB_2, q_EB_3, q_SB_0, q_SB_1, q_SB_2, q_SB_3, q_WB_0, q_WB_1, current_phase, current_simulation_step
    q_EB_0 = q_EB_1 = q_EB_2 = q_EB_3 = 0
    q_SB_0 = q_SB_1 = q_SB_2 = q_SB_3 = 0
    q_WB_0 = q_WB_1 = 0
    current_phase = 0

    # ---- Reinforcement Learning Hyperparameters ----
    TOTAL_STEPS = 5500    # The total number of simulation steps for continuous (online) training.

    ALPHA = 0.1            # Learning rate (α) between[0, 1]    #If α = 1, you fully replace the old Q-value with the newly computed estimate.
                                                                #If α = 0, you ignore the new estimate and never update the Q-value.
    GAMMA = 0.9            # Discount factor (γ) between[0, 1]  #If γ = 0, the agent only cares about the reward at the current step (no future rewards).
                                                                #If γ = 1, the agent cares equally about current and future rewards, looking at long-term gains.
    EPSILON = 0.1          # Exploration rate (ε) between[0, 1] #If ε = 0 means very greedy, if=1 means very random

    ACTIONS = [0, 1]       # The discrete action space (0 = keep phase, 1 = switch phase)

    # Q-table dictionary: key = state tuple, value = numpy array of Q-values for each action
    global Q_table
    Q_table = {}

    # ---- Additional Stability Parameters ----
    MIN_GREEN_STEPS = 100
    global last_switch_step
    last_switch_step = -MIN_GREEN_STEPS

    # -------------------------
    # Step 7: Define Functions
    # -------------------------


    def get_max_Q_value_of_state(s): #1. Objective Function 1
        if s not in Q_table:
            Q_table[s] = np.zeros(len(ACTIONS))
        return np.max(Q_table[s])

    def get_reward(state):  #2. Constraint 2 
        """
        Simple reward function:
        Negative of total queue length to encourage shorter queues.
        """
        total_queue = sum(state[:-1])  # Exclude the current_phase element
        reward = -float(total_queue)
        return reward

    def get_state():  #3.& 4. Constraint 3 & 4
        global q_EB_0, q_EB_1, q_EB_2, q_EB_3, q_SB_0, q_SB_1, q_SB_2, q_SB_3, q_WB_0, q_WB_1, current_phase
        
        detector_CMB_to_KBJ_001_1 = "CMB_to_KBJ_001_1"
        detector_CMB_to_KBJ_001_2 = "CMB_to_KBJ_001_2"
        detector_CMB_to_KBJ_001_3 = "CMB_to_KBJ_001_3"
        detector_CMB_to_KBJ_001_4 = "CMB_to_KBJ_001_4"
        
        detector_MRT_to_KB_001_2 = "MRT_to_KB_001.37_2"
        detector_MRT_to_KB_001_3 = "MRT_to_KB_001.37_3"
        detector_MRT_to_KB_001_4 = "MRT_to_KB_001.37_4"
        detector_MRT_to_KB_001_5 = "MRT_to_KB_001.37_5"
        
        detector_P_to_KBJ_1 = "P_to_KBJ_1"
        detector_P_to_KBJ_2 = "P_to_KBJ_2"

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
        """
        Executes the chosen action on the traffic light, combining:
        - Min Green Time check
        - Switching to the next phase if allowed
        Constraint #5: Ensure at least MIN_GREEN_STEPS pass before switching again.
        """
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
    plt.xticks(np.arange(900, 4501, 300))
    plt.grid(True) 
    plt.savefig(f"outputs/dynamic_DQL_vehicle_data/plots/{args.seed}.Cumulative Reward over Steps DQL.png")

    # Plot Total Queue Length over Simulation Steps
    plt.figure(figsize=(10, 6))
    plt.plot(step_history, queue_history, marker='o', linestyle='-', label="Total Queue Length")
    plt.xlabel("Simulation Step")
    plt.ylabel("Total Queue Length")
    plt.title("RL Training: Queue Length over Steps")
    plt.legend()
    plt.grid(True)
    plt.savefig(f"outputs/dynamic_DQL_vehicle_data/plots/{args.seed}.Queue Length over Steps.png")

if __name__ == '__main__':
    run_traciDQL_simulation()'''