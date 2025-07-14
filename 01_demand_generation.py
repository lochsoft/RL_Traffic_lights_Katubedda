import os
import subprocess
import random

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
             "--ptlines-file additionals/bus_lines.xml "
             "--ptstops-file additionals/bus_stops.add.xml "
             "--begin 0 "
             "--end 4500 "
             "--stop-duration 20 "
             "--output-file demand/bus_demand.rou.xml")

subprocess.run(bus_flows, shell=True)

# 4.Run Simulation - No GUI
run_simulation_no_gui = ("sumo -c simulation_katubedda_junction_static.sumocfg")
subprocess.run(run_simulation_no_gui, shell=True)

# 5. Run multiple simulation runs by changing random seed
"""seed = 12345
random.seed(seed)
random_numbers = [random.randint(10000, 99999) for i in range(3)]

comma_sep_string = ",".join(map(str, random_numbers))

multiple_runs = ("runSeeds.py --configuration simulation_katubedda_junction_static.sumocfg "
                 "--verbose "
                 f"--seeds {comma_sep_string}")
subprocess.run(multiple_runs, shell=True)"""