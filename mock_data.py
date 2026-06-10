import csv
import math
import random
import os

num_robots = 50
steps = 200

# Generate realistic-looking swarm aggregation paths
csv_file = open('experiment_results.csv', mode='w', newline='')
writer = csv.writer(csv_file)
writer.writerow(['timestamp', 'robot_id', 'x', 'y', 'z'])

for step in range(steps):
    t = step * 0.1
    for i in range(num_robots):
        # Start at grid
        start_x = (i % 10) * 1.0 - 5.0
        start_y = (i // 10) * 1.0 - 2.5
        
        # Converge to center with some noise (Aggregation task)
        progress = min(1.0, step / float(steps))
        current_x = start_x * (1.0 - progress) + random.uniform(-0.1, 0.1)
        current_y = start_y * (1.0 - progress) + random.uniform(-0.1, 0.1)
        
        writer.writerow([t, i, current_x, current_y, 0.0])

csv_file.close()
print("Generated mock experiment_results.csv")
