import sys
sys.path.append('build')
import AvTrajectoryPlanner as av
import time


scenario_file = "benchmark/speed_test_scenario.txt"
with open(scenario_file) as file:
    envFileStr = file.read()

planner = av.Planner()

planner.loadFromJson(envFileStr)


start = time.time()
# run your code
for i in range(100):
    vehicleTrajectory = planner.solveTrajectory()

end = time.time()

elapsed = end - start
print("Benchmark results:")
print(100.0/elapsed)

