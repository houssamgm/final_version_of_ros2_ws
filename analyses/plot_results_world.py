import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import xml.etree.ElementTree as ET
import os

def parse_sdf_obstacles(sdf_file):
    """Parses an SDF file and returns a list of obstacles."""
    if not os.path.exists(sdf_file):
        print(f"File {sdf_file} not found!")
        return []
        
    tree = ET.parse(sdf_file)
    root = tree.getroot()
    obstacles = []
    
    for model in root.iter('model'):
        pose_elem = model.find('.//pose')
        if pose_elem is None: continue
            
        pose = [float(v) for v in pose_elem.text.split()]
        x, y, yaw = pose[0], pose[1], pose[5]
        
        box = model.find('.//box/size')
        if box is not None:
            size = [float(v) for v in box.text.split()]
            w, h = size[0], size[1]
            if abs(yaw) > 1.5: w, h = h, w
            obstacles.append({'type': 'box', 'x': x, 'y': y, 'w': w, 'h': h})
            
        cyl = model.find('.//cylinder/radius')
        if cyl is not None:
            r = float(cyl.text)
            obstacles.append({'type': 'cylinder', 'x': x, 'y': y, 'r': r})
    return obstacles

def plot_performance(csv_file, algo_name, sdf_file='world.sdf'):
    print(f"Processing {algo_name}...")
    if not os.path.exists(csv_file):
        print(f"CSV {csv_file} missing.")
        return

    df = pd.read_csv(csv_file).dropna()
    if df.empty:
        print(f"Data in {csv_file} is empty.")
        return

    t, l_v, r_v = df['timestamp'].to_numpy(), df['leader_v'].to_numpy(), df['robot_v'].to_numpy()
    l_w, r_w = df['leader_w'].to_numpy(), df['robot_w'].to_numpy()
    l_x, l_y = df['leader_x'].to_numpy(), df['leader_y'].to_numpy()
    r_x, r_y = df['robot_x'].to_numpy(), df['robot_y'].to_numpy()
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 15))
    
    # Velocity Plots
    ax1.plot(t, l_v, '--', color='black', label='Leader (v)')
    ax1.plot(t, r_v, color='blue', label=f'{algo_name} Follower (v)')
    ax1.legend(); ax1.grid(True); ax1.set_title('Linear Velocity')
    
    ax2.plot(t, l_w, '--', color='black', label='Leader (w)')
    ax2.plot(t, r_w, color='red', label=f'{algo_name} Follower (w)')
    ax2.legend(); ax2.grid(True); ax2.set_title('Angular Velocity')
    
    # Trajectory Plot
    ax3.plot(l_x, l_y, '--', color='black', label='Leader Path', zorder=2)
    ax3.plot(r_x, r_y, color='blue', label='Follower Path', zorder=2)
    
    # Obstacles & Start Points
    for obs in parse_sdf_obstacles(sdf_file):
        if obs['type'] == 'box':
            ax3.add_patch(patches.Rectangle((obs['x']-obs['w']/2, obs['y']-obs['h']/2), obs['w'], obs['h'], color='gray', alpha=0.5))
        else:
            ax3.add_patch(patches.Circle((obs['x'], obs['y']), obs['r'], color='red', alpha=0.6))
            
    # Start points
    ax3.plot(r_x[0], r_y[0], 's', color='blue', markersize=10, label='Robot Start')
    ax3.plot(l_x[0], l_y[0], '*', color='gold', markersize=16, label='Leader')
    
    ax3.legend(loc='lower left', fontsize='small', framealpha=0.8)
    
    ax3.grid(True)
    ax3.axis('equal') 
    ax3.set_title(f'[{algo_name}] Trajectory Map with Obstacles & Start Points')
    plt.tight_layout()
    plt.savefig(f'{algo_name}_performance.png', dpi=300)
    plt.close(fig)
    print(f"Saved {algo_name}_performance.png")

# Execution
if __name__ == "__main__":
    plot_performance('woa_tracking_data.csv', 'WOA')
    plot_performance('acc_tracking_data.csv', 'ACC')
    plot_performance('pid_tracking_data.csv', 'PID')
