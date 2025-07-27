seed = 12345
random.seed(seed)
random_numbers = [random.randint(10000, 99999) for i in range(3)]
comma_sep_string = ",".join(map(str, random_numbers))

# Static Traffic lights
multiple_runs_static = ("runSeeds.py --configuration simulation_katubedda_junction_static.sumocfg "
                 "--verbose "
                 f"--seeds {comma_sep_string}")
subprocess.run(multiple_runs_static, shell=True)

# Dynamic Traffic lights - QL
for seed in random_numbers:
    print(f"\nRunning simulation with seed {seed}")
    cmd = f"python traciQL.py --seed {seed}"
    subprocess.run(cmd, shell=True)