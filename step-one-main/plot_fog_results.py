import os
import glob
import re
import matplotlib.pyplot as plt

REPORT_DIR = "reports/batch_fog_results"

def parse_fog_report(filepath):
    """Parses the generated ONE FogPerformanceReport text file."""
    data = {}
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            # Extract numeric values from the specific report lines using regex
            if line.startswith("Average Swarm Coverage:"):
                match = re.search(r"([\d\.]+)", line)
                if match: data['coverage_pct'] = float(match.group(1))
            elif line.startswith("Total Weighted Coverage Score:"):
                match = re.search(r"([\d\.]+)", line)
                if match: data['weighted_score'] = float(match.group(1))
            elif line.startswith("Max High-Priority Starvation Time:"):
                match = re.search(r"([\d\.]+)", line)
                if match: data['max_starvation'] = float(match.group(1))
            elif line.startswith("Total Distance Traveled:"):
                match = re.search(r"([\d\.]+)", line)
                if match: data['distance'] = float(match.group(1))
            elif line.startswith("Path Stability / Target Churn Count:"):
                match = re.search(r"(\d+)", line)
                if match: data['churn'] = int(match.group(1))
            elif line.startswith("Average Algorithm Execution Time:"):
                match = re.search(r"([\d\.]+)", line)
                if match: data['exec_time'] = float(match.group(1))
    return data

def main():
    search_pattern = os.path.join(REPORT_DIR, "*_FogPerformanceReport.txt")
    files = glob.glob(search_pattern)
    
    if not files:
        print(f"No report files found in '{REPORT_DIR}'. Run the simulations first!")
        return
        
    # Group results by Movement Model
    results_by_model = {}
    
    for filepath in files:
        filename = os.path.basename(filepath)
        # Expected format: MODELNAME_SeedX_FogPerformanceReport.txt
        parts = filename.split('_')
        if len(parts) >= 3:
            model_name = parts[0]
            data = parse_fog_report(filepath)
            
            if model_name not in results_by_model:
                results_by_model[model_name] = {
                    'coverage_pct': [], 'weighted_score': [], 
                    'max_starvation': [], 'distance': [], 
                    'churn': [], 'exec_time': []
                }
                
            for key in results_by_model[model_name]:
                if key in data:
                    results_by_model[model_name][key].append(data[key])

    # Plotting
    models = list(results_by_model.keys())
    print(f"Found results for models: {', '.join(models)}")
    
    plots = [
        ('coverage_pct', 'Average Swarm Coverage', 'Coverage (%)'),
        ('weighted_score', 'Total Weighted Coverage Score', 'Score'),
        ('max_starvation', 'Max High-Priority Starvation Time', 'Time (s)'),
        ('distance', 'Total Distance Traveled', 'Distance (m)'),
        ('churn', 'Target Churn Count (Path Stability)', 'Count'),
        ('exec_time', 'Computational Overhead', 'Execution Time (ms)')
    ]

    for i in range(0, len(plots), 2):
        pair = plots[i:i+2]
        fig, axs = plt.subplots(1, len(pair), figsize=(12, 6))
        
        if len(pair) == 1:
            axs = [axs]  # Ensure axs is iterable if an odd number of plots exists
            
        for j, (metric_key, title, ylabel) in enumerate(pair):
            data_to_plot = [results_by_model[m][metric_key] for m in models]
            if any(len(d) > 0 for d in data_to_plot):
                axs[j].boxplot(data_to_plot, labels=models, patch_artist=True, boxprops=dict(facecolor='lightblue'))
                axs[j].set_title(title, fontsize=14)
                axs[j].set_ylabel(ylabel, fontsize=12)
                axs[j].grid(True, linestyle='--', alpha=0.7, axis='y')
                
        plt.tight_layout()
        filename = f"fog_comparison_part_{i//2 + 1}.png"
        plt.savefig(filename, dpi=300)
        print(f"Graphs successfully saved to {filename}")

    plt.show()

if __name__ == "__main__":
    main()