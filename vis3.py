import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import CheckButtons
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
import warnings
warnings.filterwarnings('ignore')

def read_tum_trajectory(file_path):
    """Read trajectory file in TUM format"""
    try:
        print(f"Attempting to read file: {file_path}")
        data = np.loadtxt(file_path)
        print(f"Successfully loaded file: {file_path}, shape: {data.shape}")
        # Extract position information (x, y, z), corresponding to columns 1, 2, 3
        positions = data[:, 1:4]
        
        # If trajectory has orientation data (quaternions), extract that too
        if data.shape[1] >= 8:
            # columns 4,5,6,7 are quaternion (x,y,z,w)
            quaternions = data[:, 4:8]
            timestamps = data[:, 0] if data.shape[1] >= 8 else np.arange(len(positions))
            return positions, quaternions, timestamps
        else:
            timestamps = data[:, 0] if data.shape[1] >= 4 else np.arange(len(positions))
            return positions, None, timestamps
            
    except Exception as e:
        print(f"Error loading {file_path}: {e}")
        return None, None, None

def align_trajectories_umeyama(reference, target):
    """
    Align two trajectories using Umeyama alignment (similarity transformation)
    Returns the aligned target trajectory and transformation parameters
    """
    # Ensure same length
    min_len = min(len(reference), len(target))
    ref_aligned = reference[:min_len].copy()
    tgt_aligned = target[:min_len].copy()
    
    # Center the trajectories
    ref_centroid = np.mean(ref_aligned, axis=0)
    tgt_centroid = np.mean(tgt_aligned, axis=0)
    
    ref_centered = ref_aligned - ref_centroid
    tgt_centered = tgt_aligned - tgt_centroid
    
    # Compute covariance matrix
    H = tgt_centered.T @ ref_centered
    
    # SVD
    U, S, Vt = np.linalg.svd(H)
    
    # Compute rotation
    R = Vt.T @ U.T
    
    # Ensure proper rotation (det(R) = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Compute scale
    scale = np.trace(np.diag(S)) / np.trace(tgt_centered.T @ tgt_centered)
    
    # Compute translation
    t = ref_centroid - scale * R @ tgt_centroid
    
    # Apply transformation
    aligned_target = scale * (tgt_aligned @ R.T) + t
    
    return aligned_target, R, t, scale

def calculate_ate(reference, target, alignment=True):
    """
    Calculate Absolute Trajectory Error (ATE)
    ATE measures the absolute distances between corresponding trajectory points
    """
    min_length = min(len(reference), len(target))
    ref_aligned = reference[:min_length]
    tgt_aligned = target[:min_length]
    
    if alignment:
        tgt_aligned, R, t, scale = align_trajectories_umeyama(ref_aligned, tgt_aligned)
        print(f"Umeyama Alignment - Scale: {scale:.6f}, Translation: {np.linalg.norm(t):.6f}m")
    
    # Calculate absolute errors
    errors = np.sqrt(np.sum((ref_aligned - tgt_aligned) ** 2, axis=1))
    
    ate_rmse = np.sqrt(np.mean(errors ** 2))
    ate_mean = np.mean(errors)
    ate_median = np.median(errors)
    ate_std = np.std(errors)
    ate_max = np.max(errors)
    ate_min = np.min(errors)
    
    return {
        'errors': errors,
        'rmse': ate_rmse,
        'mean': ate_mean,
        'median': ate_median,
        'std': ate_std,
        'max': ate_max,
        'min': ate_min,
        'aligned_target': tgt_aligned if alignment else tgt_aligned
    }

def calculate_rte(reference, target, delta=1.0, alignment=True):
    """
    Calculate Relative Trajectory Error (RTE)
    RTE measures the consistency of relative motions
    delta: time/frame interval for relative motion calculation
    """
    min_length = min(len(reference), len(target))
    ref_aligned = reference[:min_length]
    tgt_aligned = target[:min_length]
    
    if alignment:
        tgt_aligned, _, _, _ = align_trajectories_umeyama(ref_aligned, tgt_aligned)
    
    step_size = max(1, int(delta))
    relative_errors = []
    
    for i in range(len(ref_aligned) - step_size):
        # Calculate relative motion vectors
        ref_rel = ref_aligned[i + step_size] - ref_aligned[i]
        tgt_rel = tgt_aligned[i + step_size] - tgt_aligned[i]
        
        # Calculate relative error
        rel_error = np.linalg.norm(ref_rel - tgt_rel)
        relative_errors.append(rel_error)
    
    relative_errors = np.array(relative_errors)
    
    rte_rmse = np.sqrt(np.mean(relative_errors ** 2))
    rte_mean = np.mean(relative_errors)
    rte_median = np.median(relative_errors)
    rte_std = np.std(relative_errors)
    rte_max = np.max(relative_errors)
    
    return {
        'errors': relative_errors,
        'rmse': rte_rmse,
        'mean': rte_mean,
        'median': rte_median,
        'std': rte_std,
        'max': rte_max,
        'step_size': step_size
    }

def calculate_rpe(reference, target, distances=[1, 2, 5, 10], alignment=True):
    """
    Calculate Relative Pose Error (RPE) at different distance intervals
    """
    min_length = min(len(reference), len(target))
    ref_aligned = reference[:min_length]
    tgt_aligned = target[:min_length]
    
    if alignment:
        tgt_aligned, _, _, _ = align_trajectories_umeyama(ref_aligned, tgt_aligned)
    
    rpe_results = {}
    
    for dist in distances:
        step_size = max(1, int(dist))
        if step_size >= len(ref_aligned):
            continue
            
        pose_errors = []
        
        for i in range(len(ref_aligned) - step_size):
            # Calculate relative poses
            ref_rel = ref_aligned[i + step_size] - ref_aligned[i]
            tgt_rel = tgt_aligned[i + step_size] - tgt_aligned[i]
            
            # Translation error
            trans_error = np.linalg.norm(ref_rel - tgt_rel)
            pose_errors.append(trans_error)
        
        pose_errors = np.array(pose_errors)
        
        rpe_results[f'dist_{dist}'] = {
            'errors': pose_errors,
            'rmse': np.sqrt(np.mean(pose_errors ** 2)),
            'mean': np.mean(pose_errors),
            'median': np.median(pose_errors),
            'std': np.std(pose_errors),
            'max': np.max(pose_errors)
        }
    
    return rpe_results

def calculate_trajectory_metrics(reference, target, timestamps_ref=None, timestamps_tgt=None):
    """
    Calculate comprehensive trajectory evaluation metrics
    """
    print(f"\n{'='*80}")
    print(f"COMPREHENSIVE TRAJECTORY EVALUATION METRICS")
    print(f"{'='*80}")
    
    # Basic statistics
    print(f"Reference trajectory points: {len(reference)}")
    print(f"Target trajectory points: {len(target)}")
    print(f"Evaluation points (min): {min(len(reference), len(target))}")
    
    # Calculate ATE (Absolute Trajectory Error)
    print(f"\n{'-'*40} ATE (Absolute Trajectory Error) {'-'*40}")
    ate_results = calculate_ate(reference, target, alignment=True)
    print(f"ATE RMSE:     {ate_results['rmse']:.6f} m")
    print(f"ATE Mean:     {ate_results['mean']:.6f} m") 
    print(f"ATE Median:   {ate_results['median']:.6f} m")
    print(f"ATE Std:      {ate_results['std']:.6f} m")
    print(f"ATE Max:      {ate_results['max']:.6f} m")
    print(f"ATE Min:      {ate_results['min']:.6f} m")
    
    # Calculate RTE (Relative Trajectory Error)
    print(f"\n{'-'*40} RTE (Relative Trajectory Error) {'-'*40}")
    rte_results = calculate_rte(reference, target, delta=1.0, alignment=True)
    print(f"RTE RMSE:     {rte_results['rmse']:.6f} m")
    print(f"RTE Mean:     {rte_results['mean']:.6f} m")
    print(f"RTE Median:   {rte_results['median']:.6f} m")
    print(f"RTE Std:      {rte_results['std']:.6f} m")
    print(f"RTE Max:      {rte_results['max']:.6f} m")
    
    # Calculate RPE (Relative Pose Error) at different distances
    print(f"\n{'-'*40} RPE (Relative Pose Error) {'-'*40}")
    rpe_results = calculate_rpe(reference, target, distances=[1, 2, 5, 10], alignment=True)
    for dist_key, rpe_data in rpe_results.items():
        dist = dist_key.split('_')[1]
        print(f"RPE (dist={dist}):  RMSE={rpe_data['rmse']:.6f}m, Mean={rpe_data['mean']:.6f}m, Max={rpe_data['max']:.6f}m")
    
    # Additional analysis
    print(f"\n{'-'*40} Additional Analysis {'-'*40}")
    
    # Trajectory length comparison
    ref_length = np.sum(np.sqrt(np.sum(np.diff(reference, axis=0)**2, axis=1)))
    tgt_length = np.sum(np.sqrt(np.sum(np.diff(target[:min(len(reference), len(target))], axis=0)**2, axis=1)))
    length_diff = abs(ref_length - tgt_length)
    length_ratio = tgt_length / ref_length if ref_length > 0 else 0
    
    print(f"Reference trajectory length: {ref_length:.6f} m")
    print(f"Target trajectory length:    {tgt_length:.6f} m")
    print(f"Length difference:           {length_diff:.6f} m")
    print(f"Length ratio:                {length_ratio:.6f}")
    
    # Bounding box analysis
    ref_min, ref_max = np.min(reference, axis=0), np.max(reference, axis=0)
    tgt_min, tgt_max = np.min(target, axis=0), np.max(target, axis=0)
    ref_range = ref_max - ref_min
    tgt_range = tgt_max - tgt_min
    
    print(f"\nReference bounding box: X=[{ref_min[0]:.3f}, {ref_max[0]:.3f}], Y=[{ref_min[1]:.3f}, {ref_max[1]:.3f}], Z=[{ref_min[2]:.3f}, {ref_max[2]:.3f}]")
    print(f"Target bounding box:    X=[{tgt_min[0]:.3f}, {tgt_max[0]:.3f}], Y=[{tgt_min[1]:.3f}, {tgt_max[1]:.3f}], Z=[{tgt_min[2]:.3f}, {tgt_max[2]:.3f}]")
    print(f"Reference range:        X={ref_range[0]:.3f}m, Y={ref_range[1]:.3f}m, Z={ref_range[2]:.3f}m")
    print(f"Target range:           X={tgt_range[0]:.3f}m, Y={tgt_range[1]:.3f}m, Z={tgt_range[2]:.3f}m")
    
    return {
        'ate': ate_results,
        'rte': rte_results, 
        'rpe': rpe_results,
        'trajectory_length': {'reference': ref_length, 'target': tgt_length, 'ratio': length_ratio},
        'bounding_box': {'reference': (ref_min, ref_max), 'target': (tgt_min, tgt_max)}
    }

def calculate_errors(reference, target):
    """Calculate basic errors between reference and target trajectories (legacy compatibility)"""
    # Ensure both trajectories have the same number of points
    min_length = min(len(reference), len(target))
    reference = reference[:min_length]
    target = target[:min_length]
    
    # Calculate Euclidean distance between corresponding points
    errors = np.sqrt(np.sum((reference - target) ** 2, axis=1))
    
    # Overall error statistics
    mean_error = np.mean(errors)
    max_error = np.max(errors)
    median_error = np.median(errors)
    
    return errors, mean_error, max_error, median_error

def calculate_segment_errors(reference, target, segment_size=100):
    """Calculate errors for segments of the trajectory (legacy compatibility)"""
    min_length = min(len(reference), len(target))
    reference = reference[:min_length]
    target = target[:min_length]
    
    errors = np.sqrt(np.sum((reference - target) ** 2, axis=1))
    
    # Divide into segments
    num_segments = max(1, min_length // segment_size)
    segment_mean_errors = []
    
    for i in range(num_segments):
        start_idx = i * segment_size
        end_idx = min((i + 1) * segment_size, min_length)
        segment_errors = errors[start_idx:end_idx]
        segment_mean_errors.append(np.mean(segment_errors))
    
    return segment_mean_errors

def plot_error_analysis(metrics_dict, reference_label, target_label):
    """Create detailed error analysis plots"""
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'Error Analysis: {reference_label} vs {target_label}', fontsize=16)
    
    # ATE Error plot
    ate_errors = metrics_dict['ate']['errors']
    axes[0, 0].plot(ate_errors, 'b-', alpha=0.7)
    axes[0, 0].axhline(metrics_dict['ate']['mean'], color='r', linestyle='--', 
                      label=f"Mean: {metrics_dict['ate']['mean']:.4f}m")
    axes[0, 0].axhline(metrics_dict['ate']['rmse'], color='g', linestyle='--', 
                      label=f"RMSE: {metrics_dict['ate']['rmse']:.4f}m")
    axes[0, 0].set_title('ATE (Absolute Trajectory Error)')
    axes[0, 0].set_xlabel('Point Index')
    axes[0, 0].set_ylabel('Error (m)')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # RTE Error plot
    rte_errors = metrics_dict['rte']['errors']
    axes[0, 1].plot(rte_errors, 'r-', alpha=0.7)
    axes[0, 1].axhline(metrics_dict['rte']['mean'], color='b', linestyle='--', 
                      label=f"Mean: {metrics_dict['rte']['mean']:.4f}m")
    axes[0, 1].axhline(metrics_dict['rte']['rmse'], color='g', linestyle='--', 
                      label=f"RMSE: {metrics_dict['rte']['rmse']:.4f}m")
    axes[0, 1].set_title('RTE (Relative Trajectory Error)')
    axes[0, 1].set_xlabel('Point Index')
    axes[0, 1].set_ylabel('Relative Error (m)')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Error histogram
    axes[1, 0].hist(ate_errors, bins=50, alpha=0.7, color='blue', edgecolor='black')
    axes[1, 0].axvline(metrics_dict['ate']['mean'], color='r', linestyle='--', 
                      label=f"Mean: {metrics_dict['ate']['mean']:.4f}m")
    axes[1, 0].axvline(metrics_dict['ate']['median'], color='g', linestyle='--', 
                      label=f"Median: {metrics_dict['ate']['median']:.4f}m")
    axes[1, 0].set_title('ATE Error Distribution')
    axes[1, 0].set_xlabel('Error (m)')
    axes[1, 0].set_ylabel('Frequency')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # RPE comparison plot
    rpe_data = metrics_dict['rpe']
    distances = [int(k.split('_')[1]) for k in rpe_data.keys()]
    rmse_values = [rpe_data[k]['rmse'] for k in rpe_data.keys()]
    mean_values = [rpe_data[k]['mean'] for k in rpe_data.keys()]
    
    axes[1, 1].plot(distances, rmse_values, 'o-', label='RMSE', color='red')
    axes[1, 1].plot(distances, mean_values, 's-', label='Mean', color='blue')
    axes[1, 1].set_title('RPE vs Distance Interval')
    axes[1, 1].set_xlabel('Distance Interval')
    axes[1, 1].set_ylabel('Error (m)')
    axes[1, 1].legend()
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    return fig

def plot_trajectories():
    # File paths
    files = [
        "trajectory_after_optimization.txt",  # This is considered as optimization2
        "standard_trajectory_no_loop.txt",
        "standard_trajectory_with_loop.txt"
    ]
    
    # Legend labels
    labels = [
        "optimization2",
        "Before loop", 
        "With Loop"
    ]
    
    # Colors for trajectories
    colors = ['blue', 'green', 'red', 'purple']
    
    print("Creating figure...")
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 12))
    
    # 3D plot
    ax1 = fig.add_subplot(221, projection='3d')
    # 2D plots
    ax2 = fig.add_subplot(222)  # X-Y plane
    ax3 = fig.add_subplot(223)  # X-Z plane
    ax4 = fig.add_subplot(224)  # Y-Z plane
    
    # Store all plot objects
    plot_lines = []
    scatter_points = []
    
    # Store trajectory data
    all_trajectories = []
    trajectory_dict = {}  # For easy lookup by label
    trajectory_timestamps = {}
    
    # Read each trajectory
    for file_path, label in zip(files, labels):
        positions, quaternions, timestamps = read_tum_trajectory(file_path)
        if positions is not None:
            all_trajectories.append((positions, label))
            trajectory_dict[label] = positions
            trajectory_timestamps[label] = timestamps
    
    # Use first trajectory (optimization2) as reference
    reference_trajectory, reference_label = all_trajectories[0]
    
    # Plot each trajectory and calculate comprehensive metrics
    for positions, label in all_trajectories:
        color = colors[labels.index(label)]
        print(f"Plotting {label} with {len(positions)} points")
        
        # 3D plot
        line1 = ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2], 
                label=label, color=color)[0]
        scatter1 = ax1.scatter(positions[0, 0], positions[0, 1], positions[0, 2], 
                    color=color, marker='o', s=50)  # Mark starting point
        
        # 2D plots
        line2 = ax2.plot(positions[:, 0], positions[:, 1], label=label, color=color)[0]
        scatter2 = ax2.scatter(positions[0, 0], positions[0, 1], color=color, marker='o', s=50)
        
        line3 = ax3.plot(positions[:, 0], positions[:, 2], color=color)[0]
        scatter3 = ax3.scatter(positions[0, 0], positions[0, 2], color=color, marker='o', s=50)
        
        line4 = ax4.plot(positions[:, 1], positions[:, 2], color=color)[0]
        scatter4 = ax4.scatter(positions[0, 1], positions[0, 2], color=color, marker='o', s=50)
        
        # Save plot objects for visibility control
        plot_lines.append([line1, line2, line3, line4])
        scatter_points.append([scatter1, scatter2, scatter3, scatter4])
        
        # Calculate comprehensive metrics if not the reference trajectory
        if label != reference_label:
            print(f"\n\n{'#'*100}")
            print(f"EVALUATING: {reference_label} (Reference) vs {label} (Target)")
            print(f"{'#'*100}")
            
            # Calculate comprehensive trajectory metrics
            metrics = calculate_trajectory_metrics(
                reference_trajectory, positions,
                trajectory_timestamps.get(reference_label),
                trajectory_timestamps.get(label)
            )
            
            # Create detailed error analysis plots
            error_fig = plot_error_analysis(metrics, reference_label, label)
            
            # Legacy compatibility: Basic error calculations
            print(f"\n{'-'*40} Legacy Error Analysis {'-'*40}")
            errors, mean_error, max_error, median_error = calculate_errors(reference_trajectory, positions)
            print(f"Legacy Mean Error:    {mean_error:.6f} meters")
            print(f"Legacy Max Error:     {max_error:.6f} meters") 
            print(f"Legacy Median Error:  {median_error:.6f} meters")
            
            # Calculate and print segment errors
            segment_size = 100
            segment_errors = calculate_segment_errors(reference_trajectory, positions, segment_size)
            
            print(f"\n----- Segment Errors ({segment_size} points per segment) -----")
            for i, seg_error in enumerate(segment_errors):
                print(f"Segment {i+1}: Mean Error = {seg_error:.6f} meters")
            
            # Calculate keyframe differences (assuming every 10th point is a keyframe)
            keyframe_interval = 10
            keyframes_idx = np.arange(0, min(len(reference_trajectory), len(positions)), keyframe_interval)
            
            print(f"\n----- Keyframe Differences (every {keyframe_interval}th point) -----")
            for idx in keyframes_idx:
                if idx < min(len(reference_trajectory), len(positions)):
                    kf_error = np.sqrt(np.sum((reference_trajectory[idx] - positions[idx]) ** 2))
                    print(f"KF at index {idx}: Difference = {kf_error:.6f} meters")
    
    # Special comparison: Before Loop vs With Loop
    if "Before loop" in trajectory_dict and "With Loop" in trajectory_dict:
        no_loop_trajectory = trajectory_dict["Before loop"]
        with_loop_trajectory = trajectory_dict["With Loop"]
        
        print(f"\n\n{'#'*100}")
        print(f"SPECIAL COMPARISON: Before Loop (Reference) vs With Loop (Target)")
        print(f"{'#'*100}")
        
        # Calculate comprehensive metrics for loop comparison
        loop_metrics = calculate_trajectory_metrics(
            no_loop_trajectory, with_loop_trajectory,
            trajectory_timestamps.get("Before loop"),
            trajectory_timestamps.get("With Loop")
        )
        
        # Create error analysis plot for loop comparison
        loop_error_fig = plot_error_analysis(loop_metrics, "Before loop", "With Loop")
        
        # Legacy analysis
        print(f"\n{'-'*40} Legacy Loop Comparison {'-'*40}")
        errors, mean_error, max_error, median_error = calculate_errors(no_loop_trajectory, with_loop_trajectory)
        print(f"Legacy Mean Error:    {mean_error:.6f} meters")
        print(f"Legacy Max Error:     {max_error:.6f} meters")
        print(f"Legacy Median Error:  {median_error:.6f} meters")
        
        # Calculate and print segment errors
        segment_size = 100
        segment_errors = calculate_segment_errors(no_loop_trajectory, with_loop_trajectory, segment_size)
        
        print(f"\n----- Segment Errors ({segment_size} points per segment) -----")
        for i, seg_error in enumerate(segment_errors):
            print(f"Segment {i+1}: Mean Error = {seg_error:.6f} meters")
        
        # Calculate keyframe differences
        keyframe_interval = 10
        keyframes_idx = np.arange(0, min(len(no_loop_trajectory), len(with_loop_trajectory)), keyframe_interval)
        
        print(f"\n----- Keyframe Differences (every {keyframe_interval}th point) -----")
        for idx in keyframes_idx:
            if idx < min(len(no_loop_trajectory), len(with_loop_trajectory)):
                kf_error = np.sqrt(np.sum((no_loop_trajectory[idx] - with_loop_trajectory[idx]) ** 2))
                print(f"KF at index {idx}: Difference = {kf_error:.6f} meters")
    
    # Set labels and titles
    ax1.set_xlabel('X (m)')  
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    ax1.set_title('3D Trajectory Comparison')
    
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.set_title('X-Y Plane')
    ax2.grid(True)
    ax2.legend()
    
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Z (m)')
    ax3.set_title('X-Z Plane')  
    ax3.grid(True)
    
    ax4.set_xlabel('Y (m)')
    ax4.set_ylabel('Z (m)')
    ax4.set_title('Y-Z Plane')
    ax4.grid(True)
    
    # Add checkboxes for visibility control
    print("Setting up checkboxes...")
    # Create a small axis for checkboxes
    plt.subplots_adjust(bottom=0.2)  # Make space for checkboxes
    checkbox_ax = plt.axes([0.01, 0.01, 0.15, 0.15])
    # Initial state: all checked
    visibility = [True] * len(labels)
    # Create checkboxes
    checkbox = CheckButtons(checkbox_ax, labels, visibility)
    
    # Checkbox callback function
    def update_visibility(label):
        index = labels.index(label)
        # Toggle visibility of corresponding trajectory
        for line in plot_lines[index]:
            line.set_visible(not line.get_visible())
        for scatter in scatter_points[index]:
            scatter.set_visible(not scatter.get_visible())
        # Redraw figure
        fig.canvas.draw_idle()
    
    # Register callback function
    checkbox.on_clicked(update_visibility)
    
    # Adjust layout
    plt.subplots_adjust(wspace=0.3, hspace=0.3)
    
    print("\n" + "="*100)
    print("TRAJECTORY ANALYSIS COMPLETE!")
    print("="*100)
    print("Ready to display plot...")
    
    # Show main figure - BLOCKING MODE
    plt.figure(fig.number)
    plt.show(block=True)
    print("Plot window closed.")

if __name__ == "__main__":
    print("="*100)
    print("ENHANCED ORB-SLAM3 TRAJECTORY ANALYSIS WITH PROFESSIONAL METRICS")
    print("="*100)
    print("Features:")
    print("- ATE (Absolute Trajectory Error) with Umeyama alignment")
    print("- RTE (Relative Trajectory Error)")
    print("- RPE (Relative Pose Error) at multiple distance intervals")
    print("- Comprehensive trajectory statistics")
    print("- Legacy compatibility with original error metrics")
    print("- Enhanced visualization and error analysis plots")
    print("="*100)
    plot_trajectories()
