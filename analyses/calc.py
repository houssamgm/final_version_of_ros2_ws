import re

errors = []
times = []

with open("log.txt", "r") as f:
    for line in f:
        err_match = re.search(r"err=([0-9.]+)", line)
        time_match = re.search(r"loop time = ([0-9.]+)", line)

        if err_match:
            errors.append(float(err_match.group(1)))

        if time_match:
            times.append(float(time_match.group(1)))

avg_err = sum(errors) / len(errors)
avg_time = sum(times) / len(times)

print(f"Average error = {avg_err:.4f}")
print(f"Average loop time = {avg_time:.4f} ms")
print(f"Samples = {len(errors)}")
# python3 calc.py
