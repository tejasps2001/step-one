import os
import glob
import matplotlib.pyplot as plt

REPORT_DIR = "reports/batch_results"

def parse_report(filepath):
    """Parses the generated ONE UavPathPlanningReport text file."""
    with open(filepath, 'r') as f:
        lines = f.readlines()
        
    distances = []
    times = []
    finished_count = 0
    total_count = 0
    collisions = 0
    
    in_table = False
    for line in lines:
        line = line.strip()
        if line.startswith("NodeID\t"):
            in_table = True
            continue
        if line.startswith("---"):
            in_table = False
            continue
            
        if in_table:
            parts = line.split('\t')
            if len(parts) >= 4:
                dist = float(parts[1])
                time = float(parts[2])
                finished = parts[3].lower() == 'true'
                
                total_count += 1
                if finished:
                    finished_count += 1
                    distances.append(dist)
                    times.append(time)
                    
        if line.startswith("Total Inter-UAV Collisions:"):
            collisions = int(line.split(":")[1].strip())
            
    avg_dist = sum(distances) / len(distances) if distances else 0
    avg_time = sum(times) / len(times) if times else 0
    success_rate = (finished_count / total_count * 100) if total_count > 0 else 0
    
    return {
        'avg_distance': avg_dist,
        'avg_time': avg_time,
        'success_rate': success_rate,
        'collisions': collisions
    }

def main():
    search_pattern = os.path.join(REPORT_DIR, "*_UavPathPlanningReport.txt")
    files = glob.glob(search_pattern)
    
    if not files:
        print(f"No report files found in '{REPORT_DIR}'. Run the simulations first!")
        return
        
    # Group results by Movement Model
    results_by_model = {}
    
    for filepath in files:
        filename = os.path.basename(filepath)
        # Expected format: MODELNAME_SeedX_UavPathPlanningReport.txt
        parts = filename.split('_')
        if len(parts) >= 3:
            model_name = parts[0]
            data = parse_report(filepath)
            
            if model_name not in results_by_model:
                results_by_model[model_name] = {'dist': [], 'time': [], 'succ': [], 'coll': []}
                
            results_by_model[model_name]['dist'].append(data['avg_distance'])
            results_by_model[model_name]['time'].append(data['avg_time'])
            results_by_model[model_name]['succ'].append(data['success_rate'])
            results_by_model[model_name]['coll'].append(data['collisions'])

    # Plotting
    models = list(results_by_model.keys())
    print(f"Found results for models: {', '.join(models)}")
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle('UAV Path Planning Algorithm Comparison (10 Runs)', fontsize=16)
    
    # Helper to plot boxplots
    def do_boxplot(ax, metric_key, title, ylabel):
        data = [results_by_model[m][metric_key] for m in models]
        ax.boxplot(data, labels=models, patch_artist=True, boxprops=dict(facecolor='lightblue'))
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle='--', alpha=0.7, axis='y')

    do_boxplot(axs[0, 0], 'time', 'Average Travel Time to Target', 'Time (simulated seconds)')
    do_boxplot(axs[0, 1], 'dist', 'Average Distance Traveled', 'Distance (meters)')
    do_boxplot(axs[1, 0], 'coll', 'Total Inter-UAV Collisions', 'Number of Collisions')
    
    # Success Rate (Bar chart is usually better for percentages)
    axs[1, 1].bar(models, [sum(results_by_model[m]['succ'])/len(results_by_model[m]['succ']) for m in models], color='lightgreen')
    axs[1, 1].set_title('Average Success Rate (Reached Goal)')
    axs[1, 1].set_ylabel('Percentage (%)')
    axs[1, 1].set_ylim(0, 110)
    for i, m in enumerate(models):
        val = sum(results_by_model[m]['succ'])/len(results_by_model[m]['succ'])
        axs[1, 1].text(i, val + 2, f"{val:.1f}%", ha='center')

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    
    save_path = "comparison_graphs.png"
    plt.savefig(save_path, dpi=300)
    print(f"Graphs successfully saved to {save_path}")
    plt.show()

if __name__ == "__main__":
    main()