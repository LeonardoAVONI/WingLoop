import cProfile
import subprocess
from wingloop_testrun import main
import datetime,time

import pathlib

path = pathlib.Path(__file__).parent

ts = time.time()
dts = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d_%H:%M:%S')

filename = path.joinpath("profile_"+str(dts)+".profile")
print(f"Saving profile at {filename}")
cProfile.run('main()',filename=filename)
subprocess.run(f"snakeviz {filename}")