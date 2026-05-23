import json
import csv

input_file = '/tmp/fresh446_1460_1461_r1/repeat1_seed1461/seed1461.json'
output_file = '/tmp/fresh446_usv02_step600.tsv'

with open(input_file, 'r') as f:
    data = json.load(f)

episode = data['episode_metrics'][0]
trace_samples = episode['trace_samples']
final_agent_metrics = episode['final_agent_metrics']

print("--- FINAL AGENT METRICS ---")
print(json.dumps(final_agent_metrics, indent=2))

fields = ['step', 'route_progress', 'cross_track_error', 'heading_error', 'final_linear_x', 'final_angular_z', 'nearest_id', 'nearest_distance', 'nearest_rel_x', 'nearest_rel_y']

with open(output_file, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=fields, delimiter='\t')
    writer.writeheader()
    for sample in trace_samples:
        step = sample['step']
        if step >= 600:
            agent_data = sample['agents'].get('usv_02')
            if agent_data:
                row = {field: agent_data.get(field) for field in fields if field != 'step'}
                row['step'] = step
                writer.writerow(row)
