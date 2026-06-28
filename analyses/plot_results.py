import pandas as pd
import matplotlib.pyplot as plt

def plot_performance(csv_file, algo_name):
    print(f"Reading data from {csv_file}...")
    
    # Load the CSV data
    df = pd.read_csv(csv_file)
    
    # FIX: Convert pandas columns to raw numpy arrays to prevent library conflicts
    t = df['timestamp'].to_numpy()
    l_v = df['leader_v'].to_numpy()
    r_v = df['robot_v'].to_numpy()
    l_w = df['leader_w'].to_numpy()
    r_w = df['robot_w'].to_numpy()
    l_x = df['leader_x'].to_numpy()
    l_y = df['leader_y'].to_numpy()
    r_x = df['robot_x'].to_numpy()
    r_y = df['robot_y'].to_numpy()
    
    # Create a figure with 3 subplots (stacked vertically)
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
    
    # --- 1. Linear Velocity Profile (v) ---
    ax1.plot(t, l_v, label='Leader (v)', linestyle='--', color='black')
    ax1.plot(t, r_v, label=f'{algo_name} Follower (v)', linewidth=2, color='blue')
    ax1.set_title(f'[{algo_name}] ')
    ax1.set_xlabel('Time (seconds)')
    ax1.set_ylabel('Velocity (m/s)')
    ax1.legend()
    ax1.grid(True)
    
    # --- 2. Angular Velocity Profile (w) ---
    ax2.plot(t, l_w, label='Leader (w)', linestyle='--', color='black')
    ax2.plot(t, r_w, label=f'{algo_name} Follower (w)', linewidth=2, color='red')
    ax2.set_title(f'[{algo_name}] ')
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity (rad/s)')
    ax2.legend()
    ax2.grid(True)
    
    # --- 3. Trajectory Plot (X vs Y) ---
    ax3.plot(l_x, l_y, label='Leader Path', linestyle='--', color='black')
    ax3.plot(r_x, r_y, label=f'{algo_name} Follower Path', linewidth=2, color='red')
    ax3.set_title(f'[{algo_name}] ')
    ax3.set_xlabel('X Position (m)')
    ax3.set_ylabel('Y Position (m)')
    ax3.legend()
    ax3.grid(True)
    ax3.axis('equal') # This makes sure the map isn't stretched out of proportion
    
    # Polish and save the image
    plt.tight_layout()
    output_filename = f'{algo_name}_performance_graphs.png'
    plt.savefig(output_filename, dpi=300) # dpi=300 makes it high quality
    print(f"Success! Graphs saved as {output_filename}")

# Run the function on your WOA data
plot_performance('woa_tracking_data.csv', 'WOA')
plot_performance('acc_tracking_data.csv', 'ACC')
plot_performance('pid_tracking_data.csv', 'PID')
