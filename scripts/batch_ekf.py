#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Batch EKF Comparison - Confronta EKF vs Stima CSV Originale
Processa tutti i log e genera grafici di confronto.

Uso:
    python batch_ekf.py           # Processa tutti i log
    python batch_ekf.py --dry-run # Preview
"""

import os
import sys
import glob
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# ============================================================================
# CONFIGURAZIONE
# ============================================================================

LOGS_DIR = "../logs"
OUTPUT_BASE = "../logs/sensor_plots"

APPLY_BIAS_CORRECTION = False  # No calibrazione statica
VERBOSE = False

# ============================================================================
# DATA LOADER (copiato da ekf_sensor_fusion.py)
# ============================================================================

def load_kalman_log(filepath):
    """Carica log unificato kalman_*.csv"""
    df = pd.read_csv(filepath)
    
    # Timestamp relativo
    t0 = df['Timestamp'].iloc[0]
    df['timestamp_rel'] = df['Timestamp'] - t0
    
    # IMU data (angoli in radianti)
    imu_data = pd.DataFrame({
        'timestamp_rel': df['timestamp_rel'],
        'roll': np.deg2rad(df['Roll']),
        'pitch': np.deg2rad(df['Pitch']),
        'yaw': np.deg2rad(df['Yaw']),
        'acc_x': df['Acc_X'],
        'acc_y': df['Acc_Y'],
        'acc_z': df['Acc_Z'],
        'gyr_x': df['Gyro_X'].fillna(0),
        'gyr_y': df['Gyro_Y'].fillna(0),
        'gyr_z': df['Gyro_Z'].fillna(0),
    })
    
    # USBL (solo fix validi)
    usbl_mask = (df['USBL_X'] != 0) | (df['USBL_Y'] != 0)
    usbl_data = pd.DataFrame({
        'timestamp_rel': df.loc[usbl_mask, 'timestamp_rel'],
        'usbl_x': df.loc[usbl_mask, 'USBL_X'],
        'usbl_y': df.loc[usbl_mask, 'USBL_Y']
    }).reset_index(drop=True)
    
    # Depth
    depth_data = pd.DataFrame({
        'timestamp_rel': df['timestamp_rel'],
        'depth': df['Depth']
    })
    
    # CSV Estimate
    csv_estimate = pd.DataFrame({
        'timestamp_rel': df['timestamp_rel'],
        'est_x': df['Est_X'],
        'est_y': df['Est_Y'],
        'est_z': df['Est_Z']
    })
    
    return imu_data, usbl_data, depth_data, csv_estimate

# ============================================================================
# EKF CLASS
# ============================================================================

class EKF_AUV:
    """EKF con update USBL X,Y diretto"""
    
    def __init__(self, dt=0.02, var_usbl_x=1.0, var_usbl_y=1.0, var_depth=0.01):
        self.dt = dt
        
        # Stato: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        
        # Covarianza stato
        self.P = np.diag([1.0, 1.0, 0.1, 0.01, 0.01, 0.01])
        
        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.01, 0.5, 0.5, 0.1])
        
        # Measurement noise
        self.R_usbl = np.diag([var_usbl_x * 1.5, var_usbl_y * 1.5])
        self.R_depth = np.array([[max(var_depth, 0.01)]])
        
        self.acc_weight = 0.1
        
    def set_initial_state(self, x, y, z):
        self.x[0] = x
        self.x[1] = y
        self.x[2] = z
        
    def predict(self, acc_body, roll, pitch, yaw):
        """Prediction step"""
        R = self._rotation_matrix(roll, pitch, yaw)
        acc_world = R @ acc_body
        
        # Constant velocity model
        x_pred = self.x.copy()
        x_pred[0] = self.x[0] + self.x[3] * self.dt
        x_pred[1] = self.x[1] + self.x[4] * self.dt
        x_pred[2] = self.x[2] + self.x[5] * self.dt
        
        x_pred[3] = self.x[3] + acc_world[0] * self.dt * self.acc_weight
        x_pred[4] = self.x[4] + acc_world[1] * self.dt * self.acc_weight
        x_pred[5] = self.x[5] + acc_world[2] * self.dt * self.acc_weight
        
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt
        
        self.P = F @ self.P @ F.T + self.Q
        self.x = x_pred
        
    def update_usbl(self, x_meas, y_meas):
        """Update con coordinate USBL dirette"""
        H = np.zeros((2, 6))
        H[0, 0] = 1.0
        H[1, 1] = 1.0
        
        z = np.array([x_meas, y_meas])
        z_pred = self.x[0:2]
        y = z - z_pred
        
        S = H @ self.P @ H.T + self.R_usbl
        K = self.P @ H.T @ np.linalg.inv(S)
        
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        
    def update_depth(self, depth_meas):
        """Update con depth"""
        H = np.zeros((1, 6))
        H[0, 2] = 1.0
        
        y = depth_meas - self.x[2]
        S = H @ self.P @ H.T + self.R_depth
        K = self.P @ H.T / S[0, 0]
        
        self.x = self.x + K.flatten() * y
        I = np.eye(6)
        self.P = (I - np.outer(K, H)) @ self.P
        
    def _rotation_matrix(self, roll, pitch, yaw):
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
    
    def get_state(self):
        return self.x.copy()

# ============================================================================
# EKF RUNNER
# ============================================================================

def run_ekf(imu_data, usbl_data, depth_data):
    """Esegue EKF e ritorna traiettoria"""
    
    # Calcola dt
    dt_imu = np.diff(imu_data['timestamp_rel'].values)
    dt_mean = np.mean(dt_imu) if len(dt_imu) > 0 else 0.3
    
    # Stima varianze USBL
    if len(usbl_data) > 1:
        var_usbl_x = np.var(np.diff(usbl_data['usbl_x'].values)) / 2
        var_usbl_y = np.var(np.diff(usbl_data['usbl_y'].values)) / 2
    else:
        var_usbl_x, var_usbl_y = 1.0, 1.0
    
    var_depth = np.var(np.diff(depth_data['depth'].values)) / 2 if len(depth_data) > 1 else 0.01
    
    # Init EKF
    ekf = EKF_AUV(dt=dt_mean, var_usbl_x=var_usbl_x, var_usbl_y=var_usbl_y, var_depth=var_depth)
    
    # Stato iniziale dal primo fix USBL
    if len(usbl_data) > 0:
        ekf.set_initial_state(
            usbl_data.iloc[0]['usbl_x'],
            usbl_data.iloc[0]['usbl_y'],
            depth_data.iloc[0]['depth']
        )
    
    # Prepara dati
    time_imu = imu_data['timestamp_rel'].values
    time_usbl = usbl_data['timestamp_rel'].values
    time_depth = depth_data['timestamp_rel'].values
    
    trajectory = []
    idx_usbl = 0
    idx_depth = 0
    
    for i in range(len(imu_data)):
        t = time_imu[i]
        
        acc_body = np.array([
            imu_data.iloc[i]['acc_x'],
            imu_data.iloc[i]['acc_y'],
            imu_data.iloc[i]['acc_z']
        ])
        
        roll = imu_data.iloc[i]['roll']
        pitch = imu_data.iloc[i]['pitch']
        yaw = imu_data.iloc[i]['yaw']
        
        ekf.predict(acc_body, roll, pitch, yaw)
        
        # Update USBL
        if idx_usbl < len(usbl_data) and abs(t - time_usbl[idx_usbl]) < dt_mean:
            ekf.update_usbl(
                usbl_data.iloc[idx_usbl]['usbl_x'],
                usbl_data.iloc[idx_usbl]['usbl_y']
            )
            idx_usbl += 1
        
        # Update Depth
        if idx_depth < len(depth_data) and abs(t - time_depth[idx_depth]) < dt_mean:
            ekf.update_depth(depth_data.iloc[idx_depth]['depth'])
            idx_depth += 1
        
        state = ekf.get_state()
        trajectory.append([t, state[0], state[1], state[2], state[3], state[4], state[5]])
    
    return np.array(trajectory)

# ============================================================================
# PLOTTING
# ============================================================================

def plot_ekf_comparison(trajectory, csv_estimate, usbl_data, output_dir, log_name):
    """Genera grafici di confronto EKF vs CSV estimate"""
    
    fig = plt.figure(figsize=(18, 12))
    
    time_ekf = trajectory[:, 0]
    x_ekf = trajectory[:, 1]
    y_ekf = trajectory[:, 2]
    z_ekf = trajectory[:, 3]
    
    time_csv = csv_estimate['timestamp_rel'].values
    x_csv = csv_estimate['est_x'].values
    y_csv = csv_estimate['est_y'].values
    z_csv = csv_estimate['est_z'].values
    
    time_usbl = usbl_data['timestamp_rel'].values
    usbl_x = usbl_data['usbl_x'].values
    usbl_y = usbl_data['usbl_y'].values
    
    # --- Subplot 1: Mappa 2D ---
    ax1 = fig.add_subplot(221, aspect='equal')
    
    # USBL fixes
    ax1.scatter(usbl_x, usbl_y, c='cyan', s=20, alpha=0.5, label=f'USBL ({len(usbl_data)})', zorder=2)
    
    # CSV estimate
    ax1.plot(x_csv, y_csv, '--', color='gray', linewidth=1.5, alpha=0.7, label='CSV Original')
    
    # EKF
    points = np.array([x_ekf, y_ekf]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(time_ekf.min(), time_ekf.max())
    lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=2, zorder=3)
    lc.set_array(time_ekf[:-1])
    ax1.add_collection(lc)
    plt.colorbar(lc, ax=ax1, label='Tempo [s]')
    
    # Transceiver
    ax1.plot(0, 0, 'k^', markersize=12, label='Transceiver')
    
    # Start/End
    ax1.plot(x_ekf[0], y_ekf[0], 'go', markersize=10, zorder=5)
    ax1.plot(x_ekf[-1], y_ekf[-1], 'rs', markersize=10, zorder=5)
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title(f'{log_name} - Traiettoria 2D', fontweight='bold')
    ax1.legend(loc='upper right', fontsize=8)
    ax1.grid(True, alpha=0.3)
    
    # Auto limits
    all_x = np.concatenate([x_ekf, x_csv, usbl_x])
    all_y = np.concatenate([y_ekf, y_csv, usbl_y])
    margin = max(np.ptp(all_x), np.ptp(all_y)) * 0.1 + 1
    ax1.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax1.set_ylim(all_y.min() - margin, all_y.max() + margin)
    
    # --- Subplot 2: X nel tempo ---
    ax2 = fig.add_subplot(222)
    ax2.plot(time_usbl, usbl_x, 'co', markersize=3, alpha=0.5, label='USBL X')
    ax2.plot(time_csv, x_csv, '--', color='gray', linewidth=1, alpha=0.7, label='CSV X')
    ax2.plot(time_ekf, x_ekf, 'b-', linewidth=1.5, label='EKF X')
    ax2.set_xlabel('Tempo [s]')
    ax2.set_ylabel('X [m]')
    ax2.set_title('Confronto X', fontweight='bold')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)
    
    # --- Subplot 3: Y nel tempo ---
    ax3 = fig.add_subplot(223)
    ax3.plot(time_usbl, usbl_y, 'co', markersize=3, alpha=0.5, label='USBL Y')
    ax3.plot(time_csv, y_csv, '--', color='gray', linewidth=1, alpha=0.7, label='CSV Y')
    ax3.plot(time_ekf, y_ekf, 'g-', linewidth=1.5, label='EKF Y')
    ax3.set_xlabel('Tempo [s]')
    ax3.set_ylabel('Y [m]')
    ax3.set_title('Confronto Y', fontweight='bold')
    ax3.legend(fontsize=8)
    ax3.grid(True, alpha=0.3)
    
    # --- Subplot 4: Z (profondità) ---
    ax4 = fig.add_subplot(224)
    ax4.plot(time_csv, z_csv, '--', color='gray', linewidth=1, alpha=0.7, label='CSV Z')
    ax4.plot(time_ekf, z_ekf, 'r-', linewidth=1.5, label='EKF Z')
    ax4.set_xlabel('Tempo [s]')
    ax4.set_ylabel('Z [m]')
    ax4.set_title('Confronto Z (Profondità)', fontweight='bold')
    ax4.legend(fontsize=8)
    ax4.grid(True, alpha=0.3)
    
    plt.suptitle(f'EKF vs CSV Estimate - {log_name}', fontsize=14, fontweight='bold')
    plt.tight_layout()
    
    outpath = os.path.join(output_dir, 'ekf_comparison.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    plt.close(fig)
    
    return outpath

# ============================================================================
# BATCH PROCESSING
# ============================================================================

def find_all_logs(logs_dir):
    """Trova tutti i kalman_*.csv"""
    logs = []
    giorno_dirs = sorted(glob.glob(os.path.join(logs_dir, "Giorno*")))
    
    for giorno_dir in giorno_dirs:
        if os.path.isdir(giorno_dir):
            giorno_name = os.path.basename(giorno_dir)
            csv_files = sorted(glob.glob(os.path.join(giorno_dir, "kalman_*.csv")))
            
            for csv_file in csv_files:
                log_name = os.path.splitext(os.path.basename(csv_file))[0]
                logs.append({
                    'path': csv_file,
                    'giorno': giorno_name,
                    'name': log_name
                })
    
    return logs

def process_single_log(log_info, output_base, dry_run=False):
    """Processa un singolo log"""
    output_dir = os.path.join(output_base, log_info['giorno'], log_info['name'])
    
    if dry_run:
        print(f"  [DRY-RUN] {log_info['giorno']}/{log_info['name']}")
        return True
    
    print(f"\n{'='*50}")
    print(f"{log_info['giorno']}/{log_info['name']}")
    print(f"{'='*50}")
    
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Carica dati
        imu_data, usbl_data, depth_data, csv_estimate = load_kalman_log(log_info['path'])
        print(f"  Righe: {len(imu_data)}, USBL fix: {len(usbl_data)}")
        
        # Run EKF
        trajectory = run_ekf(imu_data, usbl_data, depth_data)
        print(f"  EKF completato")
        
        # Plot
        outpath = plot_ekf_comparison(trajectory, csv_estimate, usbl_data, output_dir, log_info['name'])
        print(f"  ✓ Salvato: {outpath}")
        
        return True
        
    except Exception as e:
        print(f"  ✗ ERRORE: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Batch EKF comparison')
    parser.add_argument('--logs-dir', type=str, default=LOGS_DIR)
    parser.add_argument('--output', type=str, default=OUTPUT_BASE)
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    
    print("="*60)
    print("BATCH EKF COMPARISON")
    print("="*60)
    
    logs = find_all_logs(args.logs_dir)
    
    if not logs:
        print(f"Nessun log trovato in {args.logs_dir}")
        sys.exit(1)
    
    print(f"\nTrovati {len(logs)} log")
    
    success = 0
    failed = 0
    
    for log in logs:
        if process_single_log(log, args.output, args.dry_run):
            success += 1
        else:
            failed += 1
    
    print(f"\n{'='*60}")
    print(f"Completati: {success}, Falliti: {failed}")
    print(f"Output: {os.path.abspath(args.output)}")
    print("="*60)

if __name__ == "__main__":
    main()
