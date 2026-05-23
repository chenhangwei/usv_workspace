import json, os
path = "/tmp/fresh375_step1260_guard1463_stride20/repeat1_seed1463/seed1463.json"
with open(path, "r") as f:
    data = json.load(f)

# The trace samples are per episode
episode = data["episode_metrics"][0]
trace = episode.get("trace_samples", [])
print(f"Status: {episode.get('status')}, Collision: {episode.get('collision')}, Trace Count: {len(trace)}")

indices = [(0, "Start"), (len(trace)//2, "Mid"), (-1, "End")]
for i, label in indices:
    if abs(i) < len(trace) or (i == -1 and len(trace) > 0):
        s = trace[i]
        real_idx = i if i >= 0 else len(trace) + i
        print(f"\n{label} (idx {real_idx}):")
        # In frame-by-frame trace, the structure might be different. 
        # Checking if it has 'usvs' or if it is a list of agent data
        usvs = s.get("usvs", [])
        for j, u in enumerate(usvs):
            d = u.get("distance_to_goal", 0)
            p = u.get("route_progress", 0)
            c = u.get("cross_track_error", 0)
            h = u.get("heading_error", 0)
            lx = u.get("final_linear_x", 0)
            az = u.get("final_angular_z", 0)
            nd = u.get("nearest_distance", 0)
            da = u.get("deconflict_active", False)
            print(f"  USV {j}: goal_dist: {d:.2f}, prog: {p:.2f}, cte: {c:.2f}, h_err: {h:.2f}, lx: {lx:.2f}, az: {az:.2f}, near: {nd:.2f}, deconf: {da}")
