import json
import sys
import numpy as np

def analyze(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    metrics = data['episode_metrics'][0]
    trace_samples = metrics['trace_samples']
    
    print(f"Total samples: {len(trace_samples)}")
    print(f"Collision: {metrics.get('collision', 'N/A')}")
    print(f"Success: {metrics.get('success', 'N/A')}")
    print(f"Timeout: {metrics.get('timeout', 'N/A')}")
    print(f"Progress: {metrics.get('progress', 'N/A')}")
    print(f"Min Sep: {metrics.get('min_sep', 'N/A')}")
    print(f"Steps: {metrics.get('steps', 'N/A')}")
    
    agents = trace_samples[0]['agents'].keys()
    
    for agent_id in sorted(agents):
        print(f"\n--- Agent: {agent_id} ---")
        agent_samples = [s['agents'][agent_id] for s in trace_samples]
        
        # Initial and Final metrics
        first = agent_samples[0]
        last = agent_samples[-1]
        
        print(f"Start Distance to Goal: {first['distance_to_goal']:.4f}")
        print(f"Final Distance to Goal: {last['distance_to_goal']:.4f}")
        print(f"Start Route Progress: {first['route_progress']:.4f}")
        print(f"Final Route Progress: {last['route_progress']:.4f}")
        print(f"Start Cross Track Error: {first['cross_track_error']:.4f}")
        print(f"Final Cross Track Error: {last['cross_track_error']:.4f}")
        print(f"Start Heading Error: {first['heading_error']:.4f}")
        print(f"Final Heading Error: {last['heading_error']:.4f}")
        
        # Mean/Final values
        final_linear_x = last['linear_x']
        final_angular_z = last['angular_z']
        mean_linear_x = np.mean([s['linear_x'] for s in agent_samples])
        mean_angular_z = np.mean([s['angular_z'] for s in agent_samples])
        
        print(f"Final Linear X: {final_linear_x:.4f}, Mean Linear X: {mean_linear_x:.4f}")
        print(f"Final Angular Z: {final_angular_z:.4f}, Mean Angular Z: {mean_angular_z:.4f}")
        
        # 5 Equidistant points
        indices = np.linspace(0, len(agent_samples) - 1, 5, dtype=int)
        print("\nEquidistant points (Progress, Distance, CTE, Heading Err, Action):")
        for idx in indices:
            s = agent_samples[idx]
            # Assuming action is in the sample, if not might be in 'raw_observation' or named differently
            # Looking at typical structure, often 'action' is a key or 'last_action'
            action = s.get('action', s.get('last_action', 'N/A'))
            print(f"T={idx}: Progr={s['route_progress']:.4f}, Dist={s['distance_to_goal']:.4f}, CTE={s['cross_track_error']:.4f}, HErr={s['heading_error']:.4f}, Act={action}")

if __name__ == "__main__":
    analyze(sys.argv[1])
