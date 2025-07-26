import subprocess
import sys
import os
os.chdir("c:\\Users\\Lochana Minuwansala\\Downloads\\Simulation  model\\Katubedda Junction")
random_numbers = [1,2,3]
comma_sep_string = ",".join(map(str, random_numbers))
custom_run_seeds_script = "runSeedsDynamic.py"
multiple_runs_dynamic = (
    f"{sys.executable} {custom_run_seeds_script} traciQL.py "
    f"--specific-seeds {comma_sep_string}"
)
subprocess.run(multiple_runs_dynamic)