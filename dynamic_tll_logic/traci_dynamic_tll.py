import traci
import traci.constants as tc
import sys
import os

# Set SUMO_HOME environment variable
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

# --- Your Traffic Light Logic Configuration ---
# Define phases for J1 (assuming 15 signals, 0-14, with index 7 as '0')
# The 'detectors' list for each green phase now explicitly uses your provided detector IDs.
PHASES = {
    "J1": [
        # --- Phase 1: A-B Green (Signals 8,9,10) ---
        # DETECTORS: MRT_to_KB_001.37_2, MRT_to_KB_001.37_3, MRT_to_KB_001.37_4
        {"state": "rrrrrrOGGGrrrrr", "duration": 103, "detectors": [
            "MRT_to_KB_001.37_2",
            "MRT_to_KB_001.37_3",
            "MRT_to_KB_001.37_4"
        ]},
        {"state": "rrrrrrOYYYrrrrr", "duration": 4, "detectors": []}, # Yellow for A-B

        # --- Phase 2: B-A Green (Signals 3,4,5,6) ---
        # DETECTORS: CMB_to_KBJ_001_2, CMB_to_KBJ_001_3, CMB_to_KBJ_001_4
        {"state": "rrrGGGGOrrrrrrr", "duration": 44, "detectors": [
            "CMB_to_KBJ_001_2",
            "CMB_to_KBJ_001_3",
            "CMB_to_KBJ_001_4"
        ]},
        {"state": "rrrYYYYOrrrrrrr", "duration": 4, "detectors": []}, # Yellow for B-A

        # --- Phase 3: B-C Green (Signal 2) ---
        # DETECTORS: CMB_to_KBJ_001_1
        {"state": "rrGrrrOrrrrrrrr", "duration": 29, "detectors": [
            "CMB_to_KBJ_001_1"
        ]},
        {"state": "rrYrrrOrrrrrrrr", "duration": 4, "detectors": []}, # Yellow for B-C

        # --- Phase 4: A-C Green (Signals 11,12) ---
        # DETECTORS: MRT_to_KB_001.37_5
        {"state": "rrrrrrOrrrGGGrr", "duration": 59, "detectors": [
            "MRT_to_KB_001.37_5"
        ]},
        {"state": "rrrrrrOrrrYYYrr", "duration": 4, "detectors": []}, # Yellow for A-C

        # --- Phase 5: C-A Green (Signal 0) ---
        # DETECTORS: P_to_KBJ_1
        {"state": "GrrrrrOrrrrrrrr", "duration": 59, "detectors": [
            "P_to_KBJ_1"
        ]},
        {"state": "YrrrrrOrrrrrrrr", "duration": 4, "detectors": []}, # Yellow for C-A

        # --- Phase 6: C-B Green (Signal 1) ---
        # DETECTORS: P_to_KBJ_2
        {"state": "rGrrrrOrrrrrrrr", "duration": 57, "detectors": [
            "P_to_KBJ_2"
        ]},
        {"state": "rYrrrrOrrrrrrrr", "duration": 4, "detectors": []}, # Yellow for C-B

        # --- Repeating Phases from Signal Light 1 Observation Set 2 (if it's a continuation of the same cycle) ---
        # Phase 7: A-B Green (8,9,10) - repeat (duration 103)
        {"state": "rrrrrrOGGGrrrrr", "duration": 103, "detectors": [
            "MRT_to_KB_001.37_2",
            "MRT_to_KB_001.37_3",
            "MRT_to_KB_001.37_4"
        ]},
        {"state": "rrrrrrOYYYrrrrr", "duration": 4, "detectors": []}, # Yellow for A-B

        # Phase 8: B-A Green (3,4,5,6) - repeat (duration 52)
        {"state": "rrrGGGGOrrrrrrr", "duration": 52, "detectors": [
            "CMB_to_KBJ_001_2",
            "CMB_to_KBJ_001_3",
            "CMB_to_KBJ_001_4"
        ]},
        {"state": "rrrYYYYOrrrrrrr", "duration": 4, "detectors": []}, # Yellow for B-A

        # Phase 9: B-C Green (2) - repeat (duration 24)
        {"state": "rrGrrrOrrrrrrrr", "duration": 24, "detectors": [
            "CMB_to_KBJ_001_1"
        ]},
        {"state": "rrYrrrOrrrrrrrr", "duration": 4, "detectors": []}, # Yellow for B-C

        # Phase 10: A-C Green (11,12) - repeat (duration 51)
        {"state": "rrrrrrOrrrGGGrr", "duration": 51, "detectors": [
            "MRT_to_KB_001.37_5"
        ]},
        {"state": "rrrrrrOrrrYYYrr", "duration": 4, "detectors": []}, # Yellow for A-C
    ]
}

MAX_GREEN_EXT = 20
MIN_GREEN_DUR = 15
YELLOW_DUR = 4
PAD_LENGTH = 15 # Assuming 15 signals (0-14)

def run_simulation():
    traci.start(["sumo-gui", "-c", "simulation_katubedda_junction_dynamic.sumocfg"])

    step = 0
    current_phase_idx = 0
    current_phase_start_time = 0
    current_tl_id = "J1"

    # Initialize the traffic light state at step 0
    # Set the initial phase duration from the first phase defined in PHASES
    initial_phase_info = PHASES[current_tl_id][0]
    traci.trafficlight.setRedYellowGreenState(current_tl_id, initial_phase_info["state"].ljust(PAD_LENGTH, 'r'))
    traci.trafficlight.setPhaseDuration(current_tl_id, initial_phase_info["duration"])

    try:
        while traci.simulation.getMinExpectedNumber() > 0 or step < traci.simulation.getEndTime():
            traci.simulationStep()

            current_phase_info = PHASES[current_tl_id][current_phase_idx]
            base_green_duration = current_phase_info["duration"]
            is_yellow_phase = 'Y' in current_phase_info["state"]

            if not is_yellow_phase:
                halting_vehicles_on_current_approach = 0
                for det_id in current_phase_info.get("detectors", []):
                    try:
                        halting_vehicles_on_current_approach += traci.lanearea.getLastStepHaltingNumber(det_id)
                    except traci.exceptions.TraCIException as e:
                        print(f"Warning: Could not get data from detector {det_id}. Check if ID is correct and detector is active. Error: {e}")
                        # You might want to handle this more robustly, e.g., assume 0 or stop simulation
                        
                time_in_current_phase = step - current_phase_start_time

                # Decision Logic:
                if halting_vehicles_on_current_approach > 0 and \
                   time_in_current_phase < (base_green_duration + MAX_GREEN_EXT):
                    # Keep current phase
                    pass
                elif time_in_current_phase >= MIN_GREEN_DUR:
                    # Time to switch phase
                    next_phase_idx = (current_phase_idx + 1) % len(PHASES[current_tl_id])
                    next_phase_info = PHASES[current_tl_id][next_phase_idx]

                    traci.trafficlight.setRedYellowGreenState(current_tl_id, next_phase_info["state"].ljust(PAD_LENGTH, 'r'))
                    traci.trafficlight.setPhaseDuration(current_tl_id, next_phase_info["duration"])
                    current_phase_idx = next_phase_idx
                    current_phase_start_time = step
            else: # Yellow phase logic
                if step - current_phase_start_time >= YELLOW_DUR:
                    next_phase_idx = (current_phase_idx + 1) % len(PHASES[current_tl_id])
                    next_phase_info = PHASES[current_tl_id][next_phase_idx]
                    
                    traci.trafficlight.setRedYellowGreenState(current_tl_id, next_phase_info["state"].ljust(PAD_LENGTH, 'r'))
                    traci.trafficlight.setPhaseDuration(current_tl_id, next_phase_info["duration"])
                    current_phase_idx = next_phase_idx
                    current_phase_start_time = step

            step += 1

    except traci.exceptions.TraCIException as e:
        print(f"TraCI error during simulation: {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        traci.close()
        sys.stdout.flush()

if __name__ == "__main__":
    run_simulation()