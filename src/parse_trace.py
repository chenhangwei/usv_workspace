import json
import csv

input_file = '/tmp/fresh444_1460_1461_r1/repeat1_seed1461/seed1461.json'
output_file = '/tmp/fresh444_usv02_all.tsv'

with open(input_file, 'r') as f:
    data = json.load(f)

trace_samples = data['episode_metrics'][0]['trace_samples']
agent_id = 'usv_02'

headers = [
    'step', 'progress', 'dgoal', 'cte', 'heading_error', 'vx', 'omega', 'sep', 'threat',
    'finish_team_clear', 'finish_neighbor_clear', 'random_deconflict_weighted_active'
]

rows = []
for sample in trace_samples:
    step = sample['step']
    agent_data = sample['agents'][agent_id]
    mask_diag = agent_data.get('mask_diagnostics', {})
    
    row = {
        'step': step,
        'progress': f"{agent_data.get('route_progress', 0):.3f}",
        'dgoal': f"{agent_data.get('distance_to_goal', 0):.3f}",
        'cte': f"{agent_data.get('cross_track_error', 0):.3f}",
        'heading_error': f"{agent_data.get('heading_error', 0):.3f}",
        'vx': f"{agent_data.get('final_linear_x', 0):.3f}",
        'omega': f"{agent_data.get('final_angular_z', 0):.3f}",
        'sep': f"{agent_data.get('nearest_distance', 0):.3f}",
        'threat': f"{mask_diag.get('threat_score', 0):.3f}",
        'finish_team_clear': mask_diag.get('finish_team_clear', False),
        'finish_neighbor_clear': mask_diag.get('finish_neighbor_clear', False),
        'random_deconflict_weighted_active': f"{mask_diag.get('random_deconflict_weighted_active', 0):.3f}"
    }
    rows.append(row)

with open(output_file, 'w', newline='') as f:
    writer = csv.DictWriter(f, fieldnames=headers, delimiter='\t')
    writer.writeheader()
    writer.writerows(rows)

# Output every 40 steps to console
print('\t'.join(headers))
for row in rows:
    step = int(row['step'])
    # If step is 1, 40, 80, 120... or essentially step-1 is multiple of 40? 
    # Or just if step % 40 == 0 or step == 1.
    if step == 1 or step % 40 == 0:
        print('\t'.join(str(row[h]) for h in headers))
