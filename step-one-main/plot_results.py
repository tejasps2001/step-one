import os
import glob
import re
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

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
    exec_time = 0.0
    smoothness = 0.0
    
    in_table = False
    for line in lines:
        line = line.strip()
        if line.startswith("NodeID\t"):
            in_table = True
            # Reset table variables in case multiple reports appended to the same file
            distances = []
            times = []
            finished_count = 0
            total_count = 0
            continue
        if line.startswith("---"):
            in_table = False
            continue
            
        if in_table:
            parts = line.split('\t')
            if len(parts) >= 4:
                dist = float(parts[1])
                time = float(parts[2])
                finished = 'true' in parts[3].lower()
                
                total_count += 1
                if finished:
                    finished_count += 1
                    distances.append(dist)
                    times.append(time)
                    
        if line.startswith("Total Inter-UAV Collisions:"):
            collisions = int(line.split(":")[1].strip())
        elif "Execution Time" in line or "Computational Overhead" in line:
            match = re.search(r"([\d\.]+)", line)
            if match: exec_time = float(match.group(1))
        elif "Path Smoothness" in line or "Turn Cost" in line:
            match = re.search(r"([\d\.]+)", line)
            if match: smoothness = float(match.group(1))
            
    avg_dist = sum(distances) / len(distances) if distances else 0
    avg_time = sum(times) / len(times) if times else 0
    success_rate = (finished_count / total_count * 100) if total_count > 0 else 0
    
    return {
        'avg_distance': avg_dist,
        'avg_time': avg_time,
        'success_rate': success_rate,
        'collisions': collisions,
        'exec_time': exec_time,
        'smoothness': smoothness,
        'total_count': total_count,
        'finished_count': finished_count
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
                results_by_model[model_name] = {'dist': [], 'time': [], 'succ': [], 'coll': [], 'exec': [], 'smooth': []}
                
            results_by_model[model_name]['dist'].append(data['avg_distance'])
            results_by_model[model_name]['time'].append(data['avg_time'])
            results_by_model[model_name]['succ'].append(data['success_rate'])
            results_by_model[model_name]['coll'].append(data['collisions'])
            results_by_model[model_name]['exec'].append(data['exec_time'])
            results_by_model[model_name]['smooth'].append(data['smoothness'])
            
            # Print parsing details to help identify stale/old files lowering the average
            print(f"Parsed {filename}: {data['finished_count']}/{data['total_count']} reached ({data['success_rate']:.1f}%)")

    # Plotting
    models = list(results_by_model.keys())
    print(f"Found results for models: {', '.join(models)}")
    
    plots = [
        ('time', 'Average Travel Time to Target', 'Time (simulated seconds)'),
        ('dist', 'Average Distance Traveled', 'Distance (meters)'),
        ('coll_total', 'Total Inter-UAV Collisions', 'Total Number of Collisions'),
        ('succ', 'Average Success Rate (Reached Goal)', 'Percentage (%)'),
        ('exec', 'Computational Overhead', 'Execution Time (s)'),
        ('smooth', 'Path Smoothness / Turn Cost', 'Turn Cost')
    ]

    for i in range(0, len(plots), 2):
        pair = plots[i:i+2]
        fig, axs = plt.subplots(1, len(pair), figsize=(12, 6))
        fig.suptitle(f'UAV Path Planning Algorithm Comparison (Part {i//2 + 1})', fontsize=16)
        
        if len(pair) == 1:
            axs = [axs]
            
        for j, (metric_key, title, ylabel) in enumerate(pair):
            if metric_key == 'succ':
                data_to_plot = [sum(results_by_model[m]['succ'])/len(results_by_model[m]['succ']) for m in models]
                axs[j].bar(models, data_to_plot, color='lightgreen')
                axs[j].set_title(title, fontsize=14)
                axs[j].set_ylabel(ylabel, fontsize=12)
                axs[j].set_ylim(0, 110)
                for k, m in enumerate(models):
                    val = data_to_plot[k]
                    axs[j].text(k, val + 2, f"{val:.1f}%", ha='center')
            elif metric_key == 'coll_total':
                data_to_plot = [sum(results_by_model[m]['coll']) for m in models]
                axs[j].bar(models, data_to_plot, color='salmon')
                axs[j].set_title(title, fontsize=14)
                axs[j].set_ylabel(ylabel, fontsize=12)
                axs[j].yaxis.set_major_locator(MaxNLocator(integer=True))
                for k, m in enumerate(models):
                    val = data_to_plot[k]
                    axs[j].text(k, val + 0.2, str(val), ha='center')
            else:
                data_to_plot = [results_by_model[m][metric_key] for m in models]
                if any(len(d) > 0 for d in data_to_plot):
                    axs[j].boxplot(data_to_plot, labels=models, patch_artist=True, boxprops=dict(facecolor='lightblue'))
                    axs[j].set_title(title, fontsize=14)
                    axs[j].set_ylabel(ylabel, fontsize=12)
            axs[j].grid(True, linestyle='--', alpha=0.7, axis='y')
                
        plt.tight_layout(rect=[0, 0.03, 1, 0.95])
        filename = f"comparison_graphs_part_{i//2 + 1}.png"
        plt.savefig(filename, dpi=300)
        print(f"Graphs successfully saved to {filename}")

    plt.show()

if __name__ == "__main__":
    main()