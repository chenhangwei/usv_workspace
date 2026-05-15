import json

def get_agent_data(agent_name, step_data):
    agents = step_data.get('agents', {})
    agent = agents.get(agent_name, {})
    
    # Extract fields
    res = {
        'priority': agent.get('crossing_priority', 'N/A'),
        'dist_goal': round(agent.get('distance_to_goal', 0), 2) if agent.get('distance_to_goal') else 'N/A',
        'route_prog': round(agent.get('route_progress', 0), 4) if agent.get('route_progress') else 'N/A',
        'cte': round(agent.get('cross_track_error', 0), 3) if agent.get('cross_track_error') else 'N/A',
        'he': round(agent.get('heading_error', 0), 3) if agent.get('heading_error') else 'N/A',
        'final_x': round(agent.get('final_linear_x', 0), 3) if agent.get('final_linear_x') else 'N/A',
        'final_z': round(agent.get('final_angular_z', 0), 3) if agent.get('final_angular_z') else 'N/A',
        'raw_x': round(agent.get('raw_linear_x', 0), 3) if agent.get('raw_linear_x') else 'N/A',
        'raw_z': round(agent.get('raw_angular_z', 0), 3) if agent.get('raw_angular_z') else 'N/A',
    }
    
    # Nearest neighbor
    nn = agent.get('nearest_neighbor', {})
    res['nn_dist'] = round(nn.get('distance', 0), 2) if nn.get('distance') else 'N/A'
    res['nn_rel_x'] = round(nn.get('rel_x', 0), 2) if nn.get('rel_x') else 'N/A'
    res['nn_rel_y'] = round(nn.get('rel_y', 0), 2) if nn.get('rel_y') else 'N/A'
    
    # Mask diagnostics
    md = agent.get('mask_diagnostics', {})
    res['deconf'] = md.get('deconf', 'N/A')
    res['safe_fn'] = md.get('safe_finish', 'N/A')
    res['offrt'] = md.get('offroute', 'N/A')
    res['cte_f'] = md.get('cte', 'N/A')
    res['tgts'] = md.get('targets', 'N/A')
    
    return res

def analyze(path, label, steps=None):
    with open(path, 'r') as f:
        data = json.load(f)
    print(f"\n--- {label.upper()} ---")
    if isinstance(data, dict):
        print(f"Top keys: {list(data.keys())}")
        metrics_list = data.get('episode_metrics', [])
        if metrics_list:
            m0 = metrics_list[0]
            print(f"episode_metrics[0] keys: {list(m0.keys())}")
            trace = m0.get('trace_samples', [])
            print(f"trace_samples length: {len(trace)}")
            if steps:
                for i in steps:
                    if i < len(trace):
                        print(f"S{i} | USV_01: {get_agent_data('usv_01', trace[i])}")
                        print(f"S{i} | USV_02: {get_agent_data('usv_02', trace[i])}")

f1 = "/tmp/detgate_fresh320b_seed1460_trace_current/seed1460.json"
f2 = "/tmp/detgate_fresh320b_seed1460_trace_collision_try2/seed1460.json"

analyze(f1, "current")
analyze(f2, "collision_try2", range(58, 65))
analyze(f1, "current_safe", range(45, 53))

with open(f1) as f, open(f2) as g:
    d1 = json.load(f)
    d2 = json.load(g)
    print(f"\nScenario Geometry Equal: {d1.get('scenario_geometry') == d2.get('scenario_geometry')}")
