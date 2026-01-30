#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Sensor Data Analyzer - Visualizzazione Dati Sensori
Genera grafici PNG per ogni gruppo di sensori dal log kalman_*.csv

Uso:
    python analyze_sensors.py --log ../logs/Giorno1/kalman_HE9.csv
    python analyze_sensors.py  # usa DEFAULT_LOG_PATH
"""

import os
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# ============================================================================
# CONFIGURAZIONE
# ============================================================================

# Path di default al log CSV
DEFAULT_LOG_PATH = "../logs/Giorno1/kalman_HE13.csv"

# Directory output per i grafici
OUTPUT_DIR = "sensor_plots"

# ============================================================================
# DATA LOADER
# ============================================================================

def load_kalman_log(filepath):
    """Carica log kalman_*.csv"""
    print(f"\nCaricamento: {filepath}")
    df = pd.read_csv(filepath)
    
    # Timestamp relativo
    t0 = df['Timestamp'].iloc[0]
    df['time'] = df['Timestamp'] - t0
    
    print(f"  Righe: {len(df)}")
    print(f"  Durata: {df['time'].iloc[-1]:.1f} s")
    
    return df

# ============================================================================
# PLOT FUNCTIONS
# ============================================================================

def plot_orientation(df, output_dir):
    """Plot Roll, Pitch, Yaw"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    time = df['time'].values
    
    # Roll
    axes[0].plot(time, df['Roll'].values, 'b-', linewidth=0.8)
    axes[0].set_ylabel('Roll [°]', fontsize=12)
    axes[0].set_title('Orientamento AHRS (Xsens MTI-670)', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Pitch
    axes[1].plot(time, df['Pitch'].values, 'g-', linewidth=0.8)
    axes[1].set_ylabel('Pitch [°]', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Yaw
    axes[2].plot(time, df['Yaw'].values, 'r-', linewidth=0.8)
    axes[2].set_ylabel('Yaw [°]', fontsize=12)
    axes[2].set_xlabel('Tempo [s]', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'orientation.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_acceleration(df, output_dir):
    """Plot Accelerazioni"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    time = df['time'].values
    
    axes[0].plot(time, df['Acc_X'].values, 'b-', linewidth=0.8)
    axes[0].set_ylabel('Acc X [m/s²]', fontsize=12)
    axes[0].set_title('Accelerazione IMU (Body Frame)', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    axes[1].plot(time, df['Acc_Y'].values, 'g-', linewidth=0.8)
    axes[1].set_ylabel('Acc Y [m/s²]', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    axes[2].plot(time, df['Acc_Z'].values, 'r-', linewidth=0.8)
    axes[2].set_ylabel('Acc Z [m/s²]', fontsize=12)
    axes[2].set_xlabel('Tempo [s]', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=-9.81, color='purple', linestyle='--', alpha=0.5, label='g')
    axes[2].legend()
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'acceleration.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_gyroscope(df, output_dir):
    """Plot Giroscopio"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    time = df['time'].values
    
    # Filtra NaN
    gyr_x = df['Gyro_X'].fillna(0).values
    gyr_y = df['Gyro_Y'].fillna(0).values
    gyr_z = df['Gyro_Z'].fillna(0).values
    
    axes[0].plot(time, gyr_x, 'b-', linewidth=0.8)
    axes[0].set_ylabel('Gyro X [rad/s]', fontsize=12)
    axes[0].set_title('Velocità Angolare (Giroscopio)', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    axes[1].plot(time, gyr_y, 'g-', linewidth=0.8)
    axes[1].set_ylabel('Gyro Y [rad/s]', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    axes[2].plot(time, gyr_z, 'r-', linewidth=0.8)
    axes[2].set_ylabel('Gyro Z [rad/s]', fontsize=12)
    axes[2].set_xlabel('Tempo [s]', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    axes[2].axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'gyroscope.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_magnetometer(df, output_dir):
    """Plot Magnetometro"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    
    time = df['time'].values
    
    # Filtra NaN
    mag_x = df['Mag_X'].fillna(0).values
    mag_y = df['Mag_Y'].fillna(0).values
    mag_z = df['Mag_Z'].fillna(0).values
    
    axes[0].plot(time, mag_x, 'b-', linewidth=0.8)
    axes[0].set_ylabel('Mag X', fontsize=12)
    axes[0].set_title('Campo Magnetico (Magnetometro)', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    axes[1].plot(time, mag_y, 'g-', linewidth=0.8)
    axes[1].set_ylabel('Mag Y', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    
    axes[2].plot(time, mag_z, 'r-', linewidth=0.8)
    axes[2].set_ylabel('Mag Z', fontsize=12)
    axes[2].set_xlabel('Tempo [s]', fontsize=12)
    axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'magnetometer.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_usbl(df, output_dir):
    """Plot USBL (posizione e mappa)"""
    # Filtra fix validi
    usbl_mask = (df['USBL_X'] != 0) | (df['USBL_Y'] != 0)
    usbl_time = df.loc[usbl_mask, 'time'].values
    usbl_x = df.loc[usbl_mask, 'USBL_X'].values
    usbl_y = df.loc[usbl_mask, 'USBL_Y'].values
    
    fig = plt.figure(figsize=(16, 6))
    
    # --- Timeline X,Y ---
    ax1 = fig.add_subplot(121)
    ax1.plot(usbl_time, usbl_x, 'bo-', markersize=3, linewidth=0.8, label='USBL X')
    ax1.plot(usbl_time, usbl_y, 'go-', markersize=3, linewidth=0.8, label='USBL Y')
    ax1.set_xlabel('Tempo [s]', fontsize=12)
    ax1.set_ylabel('Posizione [m]', fontsize=12)
    ax1.set_title(f'USBL Timeline ({len(usbl_x)} fix)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # --- Mappa 2D ---
    ax2 = fig.add_subplot(122, aspect='equal')
    
    # Colora per tempo
    scatter = ax2.scatter(usbl_x, usbl_y, c=usbl_time, cmap='viridis', s=20, alpha=0.7)
    plt.colorbar(scatter, ax=ax2, label='Tempo [s]')
    
    # Transceiver
    ax2.plot(0, 0, 'k^', markersize=12, label='Transceiver')
    
    # Start/End
    ax2.plot(usbl_x[0], usbl_y[0], 'go', markersize=10, label='Start')
    ax2.plot(usbl_x[-1], usbl_y[-1], 'rs', markersize=10, label='End')
    
    ax2.set_xlabel('X [m] (Est)', fontsize=12)
    ax2.set_ylabel('Y [m] (Nord)', fontsize=12)
    ax2.set_title('USBL Posizioni', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='upper right')
    ax2.axhline(y=0, color='k', linewidth=0.5, alpha=0.3)
    ax2.axvline(x=0, color='k', linewidth=0.5, alpha=0.3)
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'usbl.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_depth(df, output_dir):
    """Plot profondità e temperatura"""
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    
    time = df['time'].values
    
    # Profondità
    axes[0].plot(time, df['Depth'].values, 'b-', linewidth=1)
    axes[0].set_ylabel('Profondità [m]', fontsize=12)
    axes[0].set_title('Profondimetro MS5837-30BA', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].invert_yaxis()
    axes[0].fill_between(time, df['Depth'].values, alpha=0.3)
    
    # Temperatura
    axes[1].plot(time, df['Temp'].values, 'r-', linewidth=1)
    axes[1].set_ylabel('Temperatura [°C]', fontsize=12)
    axes[1].set_xlabel('Tempo [s]', fontsize=12)
    axes[1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'depth_temp.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_kalman_estimate(df, output_dir):
    """Plot stime Kalman originali"""
    fig = plt.figure(figsize=(16, 10))
    
    time = df['time'].values
    
    # --- Posizione ---
    ax1 = fig.add_subplot(221)
    ax1.plot(time, df['Est_X'].values, 'b-', linewidth=0.8, label='Est X')
    ax1.plot(time, df['Est_Y'].values, 'g-', linewidth=0.8, label='Est Y')
    ax1.set_ylabel('Posizione [m]', fontsize=12)
    ax1.set_xlabel('Tempo [s]', fontsize=12)
    ax1.set_title('Stima Kalman Originale (X, Y)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # --- Z ---
    ax2 = fig.add_subplot(222)
    ax2.plot(time, df['Est_Z'].values, 'r-', linewidth=0.8, label='Est Z')
    ax2.plot(time, -df['Depth'].values, 'c--', linewidth=0.5, alpha=0.5, label='-Depth')
    ax2.set_ylabel('Z [m]', fontsize=12)
    ax2.set_xlabel('Tempo [s]', fontsize=12)
    ax2.set_title('Stima Z vs Profondità', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # --- Mappa 2D ---
    ax3 = fig.add_subplot(223, aspect='equal')
    ax3.plot(df['Est_X'].values, df['Est_Y'].values, 'b-', linewidth=0.8, alpha=0.7)
    ax3.plot(df['Est_X'].iloc[0], df['Est_Y'].iloc[0], 'go', markersize=10, label='Start')
    ax3.plot(df['Est_X'].iloc[-1], df['Est_Y'].iloc[-1], 'rs', markersize=10, label='End')
    ax3.set_xlabel('X [m]', fontsize=12)
    ax3.set_ylabel('Y [m]', fontsize=12)
    ax3.set_title('Traiettoria Stimata', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # --- Accelerazione filtrata (Stati 6,7,8) ---
    ax4 = fig.add_subplot(224)
    state6 = df['State_6'].fillna(0).values
    state7 = df['State_7'].fillna(0).values
    state8 = df['State_8'].fillna(0).values
    ax4.plot(time, state6, 'b-', linewidth=0.5, alpha=0.7, label='Acc X filt')
    ax4.plot(time, state7, 'g-', linewidth=0.5, alpha=0.7, label='Acc Y filt')
    ax4.plot(time, state8, 'r-', linewidth=0.5, alpha=0.7, label='Acc Z filt')
    ax4.set_ylabel('Accelerazione [m/s²]', fontsize=12)
    ax4.set_xlabel('Tempo [s]', fontsize=12)
    ax4.set_title('Accelerazione Filtrata (Stati KF)', fontsize=14, fontweight='bold')
    ax4.grid(True, alpha=0.3)
    ax4.legend()
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'kalman_estimate.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

def plot_overview(df, output_dir):
    """Plot panoramico di tutti i sensori"""
    fig, axes = plt.subplots(6, 1, figsize=(16, 20), sharex=True)
    
    time = df['time'].values
    
    # Yaw
    axes[0].plot(time, df['Yaw'].values, 'b-', linewidth=0.8)
    axes[0].set_ylabel('Yaw [°]')
    axes[0].set_title('OVERVIEW - Tutti i Sensori', fontsize=16, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    
    # Depth
    axes[1].plot(time, df['Depth'].values, 'c-', linewidth=0.8)
    axes[1].set_ylabel('Depth [m]')
    axes[1].invert_yaxis()
    axes[1].grid(True, alpha=0.3)
    
    # Acc magnitude
    acc_mag = np.sqrt(df['Acc_X'].values**2 + df['Acc_Y'].values**2 + df['Acc_Z'].values**2)
    axes[2].plot(time, acc_mag.values if hasattr(acc_mag, 'values') else acc_mag, 'g-', linewidth=0.5)
    axes[2].axhline(y=9.81, color='purple', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('|Acc| [m/s²]')
    axes[2].grid(True, alpha=0.3)
    
    # Gyro magnitude
    gyr_x = df['Gyro_X'].fillna(0)
    gyr_y = df['Gyro_Y'].fillna(0)
    gyr_z = df['Gyro_Z'].fillna(0)
    gyr_mag = np.sqrt(gyr_x**2 + gyr_y**2 + gyr_z**2)
    axes[3].plot(time, gyr_mag.values if hasattr(gyr_mag, 'values') else gyr_mag, 'orange', linewidth=0.5)
    axes[3].set_ylabel('|Gyro| [rad/s]')
    axes[3].grid(True, alpha=0.3)
    
    # USBL X,Y (solo fix validi)
    usbl_mask = (df['USBL_X'] != 0) | (df['USBL_Y'] != 0)
    usbl_time = df.loc[usbl_mask, 'time'].values
    usbl_x = df.loc[usbl_mask, 'USBL_X'].values
    usbl_y = df.loc[usbl_mask, 'USBL_Y'].values
    axes[4].plot(usbl_time, usbl_x, 'bo', markersize=2, label='X')
    axes[4].plot(usbl_time, usbl_y, 'gs', markersize=2, label='Y')
    axes[4].set_ylabel('USBL [m]')
    axes[4].grid(True, alpha=0.3)
    axes[4].legend(loc='upper right')
    
    # Temperature
    axes[5].plot(time, df['Temp'].values, 'r-', linewidth=0.8)
    axes[5].set_ylabel('Temp [°C]')
    axes[5].set_xlabel('Tempo [s]', fontsize=12)
    axes[5].grid(True, alpha=0.3)
    
    plt.tight_layout()
    outpath = os.path.join(output_dir, 'overview.png')
    plt.savefig(outpath, dpi=150, bbox_inches='tight')
    print(f"  Salvato: {outpath}")
    plt.close(fig)

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Analisi e visualizzazione sensori')
    parser.add_argument('--log', type=str, default=DEFAULT_LOG_PATH,
                        help=f'Path al file kalman_*.csv (default: {DEFAULT_LOG_PATH})')
    parser.add_argument('--output', type=str, default=OUTPUT_DIR,
                        help=f'Directory output PNG (default: {OUTPUT_DIR})')
    args = parser.parse_args()
    
    print("="*70)
    print("SENSOR DATA ANALYZER")
    print("="*70)
    
    # Crea directory output
    os.makedirs(args.output, exist_ok=True)
    print(f"\nOutput directory: {args.output}")
    
    # Carica dati
    df = load_kalman_log(args.log)
    
    # Genera grafici
    print("\nGenerazione grafici...")
    
    plot_overview(df, args.output)
    plot_orientation(df, args.output)
    plot_acceleration(df, args.output)
    plot_gyroscope(df, args.output)
    plot_magnetometer(df, args.output)
    plot_usbl(df, args.output)
    plot_depth(df, args.output)
    plot_kalman_estimate(df, args.output)
    
    print("\n" + "="*70)
    print(f"Completato! {8} grafici salvati in: {args.output}/")
    print("="*70)
