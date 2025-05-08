import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
import threading

distance_comm = 4.0  # Maximum distance between neighbors
filename = '40r_100s_0.csv'  # Input CSV file

# make an array of all the .csv files in the directory
# file ends with .csv and contains 'r_
filenames = []
for file in os.listdir('.'):
    if file.endswith('.csv') and 'r_' in file:
        filenames.append(file)

output_file = "out.csv"
def calc_distance_single_robot(robot_id: str, df: pd.DataFrame) -> pd.DataFrame:
    """Calculate the Euclidean distance from one robot to all the other neighbors."""
    # Get the coordinates of the robot
    robot_coords = df[df['robot'] == robot_id][['x', 'y']].values[0]
    
    # Calculate the distance to all other robots
    df['distance'] = np.sqrt((df['x'] - robot_coords[0]) ** 2 + (df['y'] - robot_coords[1]) ** 2)
    
    # Filter out distances greater than the communication range
    df = df[df['distance'] <= distance_comm]
    
    return df

def calc_distance(df: pd.DataFrame) -> pd.DataFrame:
    """Calculate the distance between all robots."""
    # Create a copy of the DataFrame to avoid modifying the original
    df_copy = df.copy()
    
    # Calculate distances for each robot
    for robot_id in df['robot'].unique():
        df_copy = calc_distance_single_robot(robot_id, df_copy)
    
    return df_copy
    
def calculate_neighbor_matrix(df_timestep, distance_comm=4.0):
    """
    Vectorized calculation of neighbor matrix for robots at a specific timestep.
    """
    # Get unique robots in this timestep
    robots = df_timestep['robot'].unique()
    num_robots = len(robots)
    
    # Create a positions array for all robots at once
    positions = np.zeros((num_robots, 2))
    for i, robot in enumerate(robots):
        positions[i] = df_timestep[df_timestep['robot'] == robot][['x', 'y']].values[0]
    
    # Calculate all pairwise distances at once using broadcasting
    # Reshape positions for broadcasting: (num_robots, 1, 2) - (1, num_robots, 2)
    diff = positions[:, np.newaxis, :] - positions[np.newaxis, :, :]
    distances = np.sqrt(np.sum(diff**2, axis=2))
    
    # Apply communication range constraint vectorized
    neighbor_matrix = np.where(distances <= distance_comm, distances, 0)
    
    # Set diagonal to 0 (self-connections)
    np.fill_diagonal(neighbor_matrix, 0)
    
    return neighbor_matrix, robots


def plot_neighbor_matrix(neighbor_matrix, robots):
    """
    Plot the neighbor matrix as a heatmap.
    """
    plt.figure(figsize=(10, 8))
    plt.imshow(neighbor_matrix, cmap='hot', interpolation='nearest')
    plt.colorbar(label='Distance')
    plt.xticks(range(len(robots)), robots, rotation=90)
    plt.yticks(range(len(robots)), robots)
    plt.title('Neighbor Matrix')
    plt.tight_layout()
    plt.show()
    
def plot_gauci_metric(ts, gauci_metric, total_food, name):
    """
    Plot the Gauci metric over time.
    """
   # plot just the gauci metric on a log scale
    # clear the figure
    plt.clf()
    plt.figure(figsize=(10, 6))
    plt.plot(ts, gauci_metric, label='Gauci Metric', color='blue')
    plt.xlabel('Timestep')
    plt.ylabel('Gauci Metric')
    plt.title('Gauci Metric Over Time for ' + name)
    plt.yscale('log')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    # save the figure
    plt.savefig(f"{name}_gauci_metric.png")
    
    # plot the total food collected over time
    plt.clf()
    plt.figure(figsize=(10, 6))
    plt.plot(ts, total_food, label='Total Food Collected', color='green')
    plt.xlabel('Timestep')
    plt.ylabel('Total Food Collected')
    plt.title('Total Food Collected Over Time for ' + name)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{name}_total_food.png")
    


def process_csv(input_file, output_file, local_array=None):
    """
    Process CSV file to calculate neighbor matrices at each timestep.
    Saves results in human-readable format.
    """
    # Use provided array or create a new one
    if local_array is None:
        local_array = [[],[],[]]
    
    # Read the CSV file into a DataFrame
    df = pd.read_csv(input_file)
    
    # Get unique timesteps
    timesteps = df['ts'].unique()
    total_timesteps = len(timesteps)
    ten_percent = max(1, total_timesteps // 10)  # Ensure we don't divide by zero
    
    print(f"Started processing {input_file} with {total_timesteps} timesteps")
    
    # Create output directory if it doesn't exist
    output_dir = os.path.dirname(output_file)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Process each timestep
    for i, ts in enumerate(timesteps):
        ts_data = df.loc[df['ts'] == ts]

        # Calculate neighbor matrix for this timestep
        neighbor_matrix, robots = calculate_neighbor_matrix(ts_data)

        # 1/4r^2 * sum(distance^2) (then /2 again bc array is symmetric and redundant)
        r = 0.17/2
        gauci_metric = 1/(8*r**2) * np.sum(neighbor_matrix**2)
        total_food = np.sum(ts_data['num_food_collected'])
        
        local_array[0].append(ts)
        local_array[1].append(gauci_metric)
        local_array[2].append(total_food)
        
        # Print progress at each 10%
        if (i + 1) % ten_percent == 0 or i + 1 == total_timesteps:
            percent_complete = min(100, ((i + 1) * 100) // total_timesteps)
            print(f"{input_file}: {percent_complete}% complete ({i + 1}/{total_timesteps} timesteps processed)")
    
    print(f"Finished processing {input_file}")
    return local_array

# Create a dictionary to store results for each file
results = {}

def process_single_file(filename, output_file, results_dict):
    """Process a single CSV file in a separate thread and store results"""
    local_giga_array = [[],[],[]]  # Local array for this thread
    local_giga_array = process_csv(filename, output_file, local_giga_array)
    # Store results instead of plotting immediately
    results_dict[filename] = {
        'ts': local_giga_array[0],
        'gauci_metric': local_giga_array[1],
        'total_food': local_giga_array[2]
    }
    print(f"Data processing complete for {filename}")

# Create and start a thread for each file
threads = []
for filename in filenames:
    thread = threading.Thread(target=process_single_file, args=(filename, output_file, results))
    threads.append(thread)
    thread.start()

# Wait for all threads to complete
for thread in threads:
    thread.join()

# Now plot all results sequentially after all processing is done
print("All processing complete. Generating plots...")
for filename, data in results.items():
    name = filename.split('.')[0]
    print(f"Plotting data for {name}")
    plot_gauci_metric(data['ts'], data['gauci_metric'], data['total_food'], name)

print("All plots generated successfully")

for filename in filenames:
    df = pd.read_csv(filename)
    ts_data = df.loc[df['ts'] == max(df['ts'])]
    total_food = np.sum(ts_data['num_food_collected'])
    print(filename[:2]+","+str(total_food))
    