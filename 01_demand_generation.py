import os
import subprocess
import random
import pandas as pd
from traciQL import run_traciQL_simulation

# 0.set working directory
os.chdir("c:\\Users\\Lochana Minuwansala\\Downloads\\Simulation  model\\Katubedda Junction")

# 1. Random Trips Generation
route_whitelist_6am_7am = ("randomTrips.py --net-file network/katubedda_junction_network.net.xml "
                   "--route-file intermediate_outputs/whitelist_6am_7am.rou.xml "
                   "--fringe-factor 300 "
                   "--max-distance 5000 "
                   "--min-distance 200 "
                   "--speed-exponent 4.0 "
                   "--period 0.1 "
                   "--validate")

subprocess.run(route_whitelist_6am_7am, shell=True)

route_whitelist_warmup = ("randomTrips.py --net-file network/katubedda_junction_network.net.xml "
                   "--route-file intermediate_outputs/whitelist_warmup.rou.xml "
                   "--fringe-factor 300 "
                   "--max-distance 5000 "
                   "--min-distance 200 "
                   "--speed-exponent 4.0 "
                   "--period 0.1 "
                   "--validate")

subprocess.run(route_whitelist_warmup, shell=True)

# 2. Generate calibrated demand
route_sampler_6am_7am = ("routeSampler.py "
                 "--route-files intermediate_outputs/whitelist_6am_7am.rou.xml "
                 "--turn-files data/edge_relation_data_6am_7am.dat.xml "
                 "--output-file demand/calibrated_demand_6am_7am.rou.xml "
                 "--attributes=\"type=\"'v_type_dist_base'\" departLane=\"'free'\" \"departSpeed=\"'0'\" ")

subprocess.run(route_sampler_6am_7am, shell=True)

route_sampler_warmup = ("routeSampler.py "
                 "--route-files intermediate_outputs/whitelist_warmup.rou.xml "
                 "--turn-files data/edge_relation_data_warmup.dat.xml "
                 "--output-file demand/calibrated_demand_warmup.rou.xml "
                 "--attributes=\"type=\"'v_type_dist_base'\" departLane=\"'free'\" \"departSpeed=\"'0'\" ")

subprocess.run(route_sampler_warmup, shell=True)

# 3. Generate Bus flows
bus_flows = ("ptlines2flows.py "
             "--net-file network/katubedda_junction_network.net.xml "
             "--ptlines-file additionals/bus/bus_lines.xml "
             "--ptstops-file additionals/bus/bus_stops.add.xml "
             "--begin 0 "
             "--end 4500 "
             "--stop-duration 20 "
             "--output-file demand/bus_demand.rou.xml")

subprocess.run(bus_flows, shell=True)

# 4.Run Simulation - No GUI - Static
run_simulation_no_gui_static = ("sumo -c simulation_katubedda_junction_static.sumocfg")
subprocess.run(run_simulation_no_gui_static, shell=True)

# 4.Run Simulation - No GUI - Dynamic
run_traciQL_simulation()
print("dynamic TL logic running completed!")

# 5. Run multiple simulation runs by changing random seed
seed = 12345
random.seed(seed)
##random_numbers = [random.randint(10000, 99999) for i in range(3)]
random_numbers = [1,2,3]
comma_sep_string = ",".join(map(str, random_numbers))

# Static Traffic lights
multiple_runs_static = ("runSeeds.py --configuration simulation_katubedda_junction_static.sumocfg "
                 "--verbose "
                 f"--seeds {comma_sep_string}")
subprocess.run(multiple_runs_static, shell=True)

# 6. Output processing
scenarios = ["static_vehicle_data"]

# converting static xml data file to csv
for scene_folder in scenarios:
    all_pd = pd.DataFrame()
    for random_number in random_numbers:
        xml_2_csv_call = f"""python "%SUMO_HOME%\\tools\\xml\\xml2csv.py" outputs\\{scene_folder}\\{random_number}.{scene_folder}.xml -s ," """
        subprocess.run(xml_2_csv_call, shell=True)

        # read the output from xml2csv call to a pandas dataframe
        temp = pd.read_csv(f"outputs/{scene_folder}/{random_number}.{scene_folder}.csv")
        temp['random_seed'] = random_number
        temp['scenario'] = scene_folder

        all_pd = pd.concat([all_pd, temp], ignore_index=True)

    all_pd.to_csv(f'outputs/{scene_folder}/processed_{scene_folder}.csv', index = False)

# converting dynamic xml data file to csv
xml_2_csv_call = """python "%SUMO_HOME%\\tools\\xml\\xml2csv.py" outputs\\dynamic_vehicle_data\\dynamic_vehicle_data.xml -s ," """
subprocess.run(xml_2_csv_call, shell=True)

'''detectors = ["CMB_to_KBJ_001_1", "CMB_to_KBJ_001_2", "CMB_to_KBJ_001_3", "CMB_to_KBJ_001_4", 
             "MRT_to_KB_001.37_2", "MRT_to_KB_001.37_3", "MRT_to_KB_001.37_4", "MRT_to_KB_001.37_5", 
             "P_to_KBJ_1", "P_to_KBJ_2"]

#scenarios = ["static_vehicle_data", "dynamic_vehicle_data"]
scenarios = ["static_vehicle_data"]'''

'''for scene_folder in scenarios:
    all_pd = pd.DataFrame()
    for detector in detectors:
        detector_dt = pd.DataFrame()
        for random_number in random_numbers:
            xml_2_csv_call = f"""python "%SUMO_HOME%\\tools\\xml\\xml2csv.py" outputs\\{scene_folder}\\{random_number}.{detector}.xml -s ," """
            subprocess.run(xml_2_csv_call, shell=True)

            # read the output from xml2csv call to a pandas dataframe
            temp = pd.read_csv(f"outputs/{scene_folder}/{random_number}.{detector}.csv")
            temp['random_seed'] = random_number
            temp['scenario'] = scene_folder

            detector_dt = pd.concat([detector_dt, temp], ignore_index=True)

        all_pd = pd.concat([all_pd, detector_dt], ignore_index=True)

    all_pd.to_csv(f'outputs/{scene_folder}/processed_{scene_folder}_results/all_{scene_folder}.csv', index = False)'''

'''for detector in detectors:
        detector_dt = pd.DataFrame()
        for random_number in random_numbers:
            xml_2_csv_call = f"""python "%SUMO_HOME%\\tools\\xml\\xml2csv.py" outputs\\dynamic_vehicle_data\\{detector}.xml -s ," """
            subprocess.run(xml_2_csv_call, shell=True)

            # read the output from xml2csv call to a pandas dataframe
            temp = pd.read_csv(f"outputs/dynamic_vehicle_data/{detector}.csv")
            temp['random_seed'] = random_number
            temp['scenario'] = "dynamic"

            detector_dt = pd.concat([detector_dt, temp], ignore_index=True)

        all_pd = pd.concat([all_pd, detector_dt], ignore_index=True)

all_pd.to_csv(f'outputs/dynamic_vehicle_data/processed_dynamic_vehicle_data_results/all_dynamic_vehicle_data.csv', index = False)
'''