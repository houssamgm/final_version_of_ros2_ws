import re

# Initialize lists to hold the data
errors = []
heading_errors = []
times = []
osc_counts = []
max_oscillations = []
effort_v = []
effort_w = []

# --- Read log file ---
# Assumes you are reading from your log file
log_filename = "woa_tracking_data.csv" # Or your log.txt if you prefer parsing stdout

try:
    with open("log.txt", "r") as f:
        for line in f:
            # Match standard metrics
            err_match = re.search(r"err=([0-9.]+)", line)
            time_match = re.search(r"t=([0-9.]+)", line)
            heading_match = re.search(r"head_err=([0-9.]+)", line)
            
            # Match oscillation metrics
            osc_count_match = re.search(r"osc=(\d+)", line)
            max_osc_match = re.search(r"max_osc=([0-9.]+)", line)

            # --- MATCHING NEW EFFORT METRICS ---
            eff_v_match = re.search(r"eff_v=([0-9.]+)", line)
            eff_w_match = re.search(r"eff_w=([0-9.]+)", line)

            if err_match:
                errors.append(float(err_match.group(1)))
            if heading_match:
                heading_errors.append(float(heading_match.group(1)))
            if time_match:
                times.append(float(time_match.group(1)))
            if osc_count_match:
                osc_counts.append(int(osc_count_match.group(1)))
            if max_osc_match:
                max_oscillations.append(float(max_osc_match.group(1)))
            if eff_v_match:
                effort_v.append(float(eff_v_match.group(1)))
            if eff_w_match:
                effort_w.append(float(eff_w_match.group(1)))

    # --- Basic stats ---
    if not errors:
        print("Error: No valid metrics found. Check log file format.")
        exit()

    avg_err = sum(errors) / len(errors)
    avg_heading_err = sum(heading_errors) / len(heading_errors) if heading_errors else 0.0
    avg_time = sum(times) / len(times) if times else 0.0
    
    # Get the final effort values (most recent logged)
    final_eff_v = effort_v[-1] if effort_v else 0.0
    final_eff_w = effort_w[-1] if effort_w else 0.0

    # Calculate Average per tick (The "Nervousness" index)
    avg_eff_v = final_eff_v / len(effort_v) if effort_v else 0.0
    avg_eff_w = final_eff_w / len(effort_w) if effort_w else 0.0

    # --- Output ---
    print(f"--- Analysis Summary ---")
    print(f"Samples processed     = {len(errors)}")
    print(f"Average linear error  = {avg_err:.4f}")
    print(f"Average heading err   = {avg_heading_err:.4f} rad")
    print(f"Average loop time     = {avg_time:.4f} ms")

    print("\n--- Oscillation Metrics ---")
    print(f"Oscillation count     = {osc_counts[-1] if osc_counts else 0}")
    print(f"Max osc. amplitude    = {max_oscillations[-1] if max_oscillations else 0.0:.4f}")

    print("\n--- Control Effort Metrics ---")
    print(f"Total Linear Effort   = {final_eff_v:.4f}")
    print(f"Avg Linear Effort/tick= {avg_eff_v:.6f}")
    print(f"Total Angular Effort  = {final_eff_w:.4f}")
    print(f"Avg Angular Effort/tick= {avg_eff_w:.6f}")

except FileNotFoundError:
    print("Log file not found. Ensure the filename matches.")
