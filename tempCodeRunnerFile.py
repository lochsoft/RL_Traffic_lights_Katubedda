random_numbers = [1,2,3]

comma_sep_string = ",".join(map(str, random_numbers))

# Static Traffic lights
multiple_runs_static = ("runSeeds.py --configuration simulation_katubedda_junction_static.sumocfg "
                 "--verbose "
                 f"--seeds {comma_sep_string}")
subprocess.run(multiple_runs_static, shell=True)

# Dynamic (RL) logic
multiple_runs_dynamic = ("python runSeeds.py -c simulation_katubedda_junction_dynamic.sumocfg "
                        "--application python dynamic_tll_logic/traci.QL.py "
                        "--config simulation_katubedda_junction_dynamic.sumocfg " 
                        f"--seeds {comma_sep_string} "
                        "--no-plot")
subprocess.run(multiple_runs_dynamic, shell=True)