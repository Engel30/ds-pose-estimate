#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Sensor Fusion per AUV - Mare Aperto
Fonde IMU, USBL (coordinate X,Y), e Depth sensor con Extended Kalman Filter

Caratteristiche:
- Caricamento log unificato kalman_*.csv
- Modello constant-velocity (più stabile per AUV)
- Update USBL con coordinate X,Y dirette (non range-only)
- Correzione bias IMU
- Confronto con stima CSV originale

Uso:
    python ekf_sensor_fusion.py --log ../logs/Giorno1/kalman_HE9.csv
"""

import os
import argparse
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection

# ============================================================================
# CONFIGURAZIONE
# ============================================================================

# Path di default al log CSV (modifica qui per evitare argomento CLI)
DEFAULT_LOG_PATH = "../logs/Giorno1/kalman_HE9.csv"

# Posizione transceiver USBL (origine del sistema di riferimento)
TRANSCEIVER_POSITION = (0.0, 0.0, 0.0)

# FLAGS
APPLY_BIAS_CORRECTION = True
VERBOSE = True
SHOW_CSV_ESTIMATE = True  # Toggle per mostrare stima dal CSV originale

# ============================================================================
# DATA LOADER
# ============================================================================

def load_kalman_log(filepath):
    """
    Carica log unificato kalman_*.csv
    
    Colonne attese (23):
    0:Timestamp, 1:Roll, 2:Pitch, 3:Yaw, 4:Acc_X, 5:Acc_Y, 6:Acc_Z,
    7:USBL_X, 8:USBL_Y, 9:Depth, 10:Est_X, 11:Est_Y, 12:Est_Z, 13:Temp,
    14:State_6, 15:State_7, 16:State_8, 17:Gyro_X, 18:Gyro_Y, 19:Gyro_Z,
    20:Mag_X, 21:Mag_Y, 22:Mag_Z
    
    Returns:
        imu_data: DataFrame con acc, gyro, orientamento
        usbl_data: DataFrame con fix USBL validi (X,Y != 0)
        depth_data: DataFrame con profondità
        csv_estimate: DataFrame con stime originali (Est_X, Est_Y, Est_Z)
    """
    print(f"\nCaricamento log: {filepath}")
    
    # Leggi CSV
    df = pd.read_csv(filepath)
    
    print(f"  Righe totali: {len(df)}")
    print(f"  Colonne: {len(df.columns)}")
    
    # Converti timestamp in relativo
    t0 = df['Timestamp'].iloc[0]
    df['timestamp_rel'] = df['Timestamp'] - t0
    
    # --- IMU DATA ---
    # Converti angoli da gradi a radianti
    imu_data = pd.DataFrame({
        'timestamp_rel': df['timestamp_rel'],
        'roll': np.deg2rad(df['Roll']),
        'pitch': np.deg2rad(df['Pitch']),
        'yaw': np.deg2rad(df['Yaw']),
        'acc_x': df['Acc_X'],
        'acc_y': df['Acc_Y'],
        'acc_z': df['Acc_Z'],
        'gyr_x': df['Gyro_X'].fillna(0),  # Prima riga potrebbe avere NaN
        'gyr_y': df['Gyro_Y'].fillna(0),
        'gyr_z': df['Gyro_Z'].fillna(0),
        'mag_x': df['Mag_X'].fillna(0),
        'mag_y': df['Mag_Y'].fillna(0),
        'mag_z': df['Mag_Z'].fillna(0)
    })
    
    # --- USBL DATA ---
    # Filtra solo righe con fix valido (X,Y != 0)
    usbl_mask = (df['USBL_X'] != 0) | (df['USBL_Y'] != 0)
    usbl_data = pd.DataFrame({
        'timestamp_rel': df.loc[usbl_mask, 'timestamp_rel'],
        'usbl_x': df.loc[usbl_mask, 'USBL_X'],
        'usbl_y': df.loc[usbl_mask, 'USBL_Y']
    }).reset_index(drop=True)
    
    print(f"  Fix USBL validi: {len(usbl_data)} / {len(df)}")
    
    # --- DEPTH DATA ---
    depth_data = pd.DataFrame({
        'timestamp_rel': df['timestamp_rel'],
        'depth': df['Depth']
    })
    
    # --- CSV ESTIMATE (per confronto) ---
    csv_estimate = pd.DataFrame({
        'timestamp_rel': df['timestamp_rel'],
        'est_x': df['Est_X'],
        'est_y': df['Est_Y'],
        'est_z': df['Est_Z']
    })
    
    return imu_data, usbl_data, depth_data, csv_estimate

# ============================================================================
# ANALISI AUTOMATICA COVARIANZE
# ============================================================================

def analyze_sensor_noise(imu_data, usbl_data, depth_data, imu_calibration=None):
    """
    Analizza i dati dei sensori per stimare le matrici di covarianza
    """
    print("\n" + "="*70)
    print("ANALISI RUMORE SENSORI")
    print("="*70)
    
    # --- IMU ---
    if imu_calibration is not None:
        print("\n✓ Usando calibrazione statica IMU")
        bias_acc = np.array(imu_calibration['bias_acc'])
        bias_gyr = np.array(imu_calibration['bias_gyr'])
        var_acc = np.array(imu_calibration['var_acc'])
        var_gyr = np.array(imu_calibration['var_gyr'])
        
        print(f"\nIMU (da calibrazione statica):")
        print(f"  Bias accelerometro: [{bias_acc[0]:.6f}, {bias_acc[1]:.6f}, {bias_acc[2]:.6f}] m/s²")
        print(f"  Std accelerometro:  [{np.sqrt(var_acc[0]):.6f}, {np.sqrt(var_acc[1]):.6f}, {np.sqrt(var_acc[2]):.6f}] m/s²")
        print(f"  Bias giroscopio:    [{bias_gyr[0]:.6f}, {bias_gyr[1]:.6f}, {bias_gyr[2]:.6f}] rad/s")
        print(f"  Std giroscopio:     [{np.sqrt(var_gyr[0]):.6f}, {np.sqrt(var_gyr[1]):.6f}, {np.sqrt(var_gyr[2]):.6f}] rad/s")
    else:
        print("\n⚠ ATTENZIONE: Stimando bias/noise da dati in movimento")
        print("  Questo NON è corretto! Usa calibrazione statica.")
        
        acc_x = imu_data['acc_x'].values
        acc_y = imu_data['acc_y'].values
        acc_z = imu_data['acc_z'].values
        
        gyr_x = imu_data['gyr_x'].values
        gyr_y = imu_data['gyr_y'].values
        gyr_z = imu_data['gyr_z'].values
        
        bias_acc = np.array([np.mean(acc_x), np.mean(acc_y), np.mean(acc_z)])
        bias_gyr = np.array([np.mean(gyr_x), np.mean(gyr_y), np.mean(gyr_z)])
        var_acc = np.array([np.var(acc_x), np.var(acc_y), np.var(acc_z)])
        var_gyr = np.array([np.var(gyr_x), np.var(gyr_y), np.var(gyr_z)])
        
        print(f"\nIMU (stima approssimativa):")
        print(f"  Bias accelerometro: [{bias_acc[0]:.4f}, {bias_acc[1]:.4f}, {bias_acc[2]:.4f}] m/s²")
        print(f"  Std accelerometro:  [{np.sqrt(var_acc[0]):.4f}, {np.sqrt(var_acc[1]):.4f}, {np.sqrt(var_acc[2]):.4f}] m/s²")
        print(f"  Bias giroscopio:    [{bias_gyr[0]:.4f}, {bias_gyr[1]:.4f}, {bias_gyr[2]:.4f}] rad/s")
        print(f"  Std giroscopio:     [{np.sqrt(var_gyr[0]):.4f}, {np.sqrt(var_gyr[1]):.4f}, {np.sqrt(var_gyr[2]):.4f}] rad/s")
    
    # --- USBL ---
    # Stima varianza dalle coordinate X,Y
    usbl_x = usbl_data['usbl_x'].values
    usbl_y = usbl_data['usbl_y'].values
    
    # Usa varianza differenziale (più robusta)
    if len(usbl_x) > 1:
        var_usbl_x = np.var(np.diff(usbl_x)) / 2  # Varianza differenze / 2
        var_usbl_y = np.var(np.diff(usbl_y)) / 2
    else:
        var_usbl_x = 1.0  # Default
        var_usbl_y = 1.0
    
    print(f"\nUSBL:")
    print(f"  Fix validi: {len(usbl_data)}")
    print(f"  X range: [{usbl_x.min():.2f}, {usbl_x.max():.2f}] m")
    print(f"  Y range: [{usbl_y.min():.2f}, {usbl_y.max():.2f}] m")
    print(f"  Std X: {np.sqrt(var_usbl_x):.3f} m")
    print(f"  Std Y: {np.sqrt(var_usbl_y):.3f} m")
    
    # --- DEPTH ---
    depth = depth_data['depth'].values
    var_depth = np.var(np.diff(depth)) / 2 if len(depth) > 1 else 0.01
    print(f"\nDepth:")
    print(f"  Range: [{depth.min():.3f}, {depth.max():.3f}] m")
    print(f"  Std depth: {np.sqrt(var_depth):.4f} m")
    
    return {
        'bias_acc': bias_acc,
        'bias_gyr': bias_gyr,
        'var_acc': var_acc,
        'var_gyr': var_gyr,
        'var_usbl_x': var_usbl_x,
        'var_usbl_y': var_usbl_y,
        'var_depth': var_depth
    }

# ============================================================================
# EXTENDED KALMAN FILTER
# ============================================================================

class EKF_AUV:
    """
    Extended Kalman Filter per AUV
    
    Stato (6D): [x, y, z, vx, vy, vz]
    - Posizione 3D
    - Velocità 3D
    
    Modello: Constant velocity (più stabile del constant acceleration)
    Update USBL: Coordinate X,Y dirette (osservazione lineare)
    """
    
    def __init__(self, sensor_noise, dt=0.02, config=None, initial_state=None):
        """
        Inizializza EKF
        
        Args:
            sensor_noise: dict con varianze sensori
            dt: time step [s]
            config: dict con parametri di configurazione (opzionale)
            initial_state: dict con posizione e velocità iniziali (opzionale)
                          {'position': [x, y, z], 'velocity': [vx, vy, vz]}
        """
        self.dt = dt
        self.sensor_noise = sensor_noise
        
        # Carica configurazione (usa default se non fornita)
        if config is None:
            config = {
                'Q_POSITION': 0.01,
                'Q_VELOCITY': 0.5,
                'Q_VELOCITY_Z': 0.1,
                'R_USBL_FACTOR': 1.5,
                'R_DEPTH_MIN': 0.01,
                'P0_POSITION_XY': 1.0,
                'P0_POSITION_Z': 0.1,
                'P0_VELOCITY': 0.01,
                'ACC_INTEGRATION_WEIGHT': 0.1
            }
        
        self.config = config
        self.acc_weight = config.get('ACC_INTEGRATION_WEIGHT', 0.1)
        
        # Stato: [x, y, z, vx, vy, vz]
        self.x = np.zeros(6)
        
        # Applica stato iniziale se fornito
        if initial_state is not None:
            if 'position' in initial_state:
                pos = initial_state['position']
                self.x[0:3] = pos
                if VERBOSE:
                    print(f"\nPosizione iniziale: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] m")
            if 'velocity' in initial_state:
                vel = initial_state['velocity']
                self.x[3:6] = vel
                if VERBOSE:
                    print(f"Velocità iniziale:  [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}] m/s")
        
        # Covarianza stato
        self.P = np.diag([
            config.get('P0_POSITION_XY', 1.0),
            config.get('P0_POSITION_XY', 1.0),
            config.get('P0_POSITION_Z', 0.1),
            config.get('P0_VELOCITY', 0.01),
            config.get('P0_VELOCITY', 0.01),
            config.get('P0_VELOCITY', 0.01)
        ])
        
        # Process noise Q
        q_pos = config.get('Q_POSITION', 0.01)
        q_vel = config.get('Q_VELOCITY', 0.5)
        q_vel_z = config.get('Q_VELOCITY_Z', 0.1)
        
        self.Q = np.diag([
            q_pos, q_pos, q_pos,
            q_vel, q_vel, q_vel_z
        ])
        
        # Measurement noise R per USBL (2x2 per X,Y)
        r_usbl_factor = config.get('R_USBL_FACTOR', 1.5)
        r_depth_min = config.get('R_DEPTH_MIN', 0.01)
        
        self.R_usbl = np.diag([
            sensor_noise['var_usbl_x'] * r_usbl_factor,
            sensor_noise['var_usbl_y'] * r_usbl_factor
        ])
        self.R_depth = np.array([[max(sensor_noise['var_depth'], r_depth_min)]])
        
        if VERBOSE:
            print(f"\n" + "="*70)
            print("INIZIALIZZAZIONE EKF")
            print("="*70)
            print(f"\nMatrice Q (process noise):")
            print(f"  Posizione: {q_pos} m²")
            print(f"  Velocità:  {q_vel} m²/s²")
            print(f"  Velocità Z: {q_vel_z} m²/s²")
            print(f"\nMatrice R (measurement noise):")
            print(f"  USBL X: {self.R_usbl[0,0]:.4f} m²")
            print(f"  USBL Y: {self.R_usbl[1,1]:.4f} m²")
            print(f"  Depth:  {self.R_depth[0,0]:.4f} m²")
            print(f"\nACC integration weight: {self.acc_weight}")
        
    def predict(self, acc_body, gyro, roll, pitch, yaw):
        """
        Prediction step - integra IMU
        
        Args:
            acc_body: accelerazione in body frame [ax, ay, az] (m/s²)
            gyro: velocità angolare [gx, gy, gz] (rad/s) 
            roll, pitch, yaw: orientamento (rad)
        """
        # Rotazione da body frame a world frame
        R_body_to_world = self._rotation_matrix(roll, pitch, yaw)
        acc_world = R_body_to_world @ acc_body
        
        # State transition
        x_pred = self.x.copy()
        x_pred[0] = self.x[0] + self.x[3] * self.dt  # x
        x_pred[1] = self.x[1] + self.x[4] * self.dt  # y
        x_pred[2] = self.x[2] + self.x[5] * self.dt  # z
        
        # Velocità: piccola correzione da accelerazione (peso configurabile)
        x_pred[3] = self.x[3] + acc_world[0] * self.dt * self.acc_weight  # vx
        x_pred[4] = self.x[4] + acc_world[1] * self.dt * self.acc_weight  # vy
        x_pred[5] = self.x[5] + acc_world[2] * self.dt * self.acc_weight  # vz
        
        # Jacobiano della transizione
        F = np.eye(6)
        F[0, 3] = self.dt
        F[1, 4] = self.dt
        F[2, 5] = self.dt
        
        # Covarianza prediction
        P_pred = F @ self.P @ F.T + self.Q
        
        self.x = x_pred
        self.P = P_pred
        
    def update_usbl_position(self, x_meas, y_meas):
        """
        Update step - correzione con coordinate USBL dirette
        
        Misura: h(x) = [x, y] (osservazione lineare!)
        
        Args:
            x_meas: coordinata X misurata dall'USBL (m)
            y_meas: coordinata Y misurata dall'USBL (m)
        """
        # Jacobiano: H = [[1,0,0,0,0,0], [0,1,0,0,0,0]]
        H = np.zeros((2, 6))
        H[0, 0] = 1.0  # ∂h1/∂x
        H[1, 1] = 1.0  # ∂h2/∂y
        
        # Misura e predizione
        z = np.array([x_meas, y_meas])
        z_pred = self.x[0:2]
        
        # Innovation
        y = z - z_pred
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_usbl
        
        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form per stabilità numerica)
        I = np.eye(6)
        IKH = I - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ self.R_usbl @ K.T
        
    def update_depth(self, depth_measured):
        """
        Update step - correzione con depth sensor
        
        Args:
            depth_measured: profondità misurata (m)
        """
        # Jacobiano: H = [0, 0, 1, 0, 0, 0]
        H = np.zeros((1, 6))
        H[0, 2] = 1.0
        
        # Innovation (depth è positivo, z è negativo sott'acqua)
        y = depth_measured - self.x[2]
        
        # Innovation covariance
        S = H @ self.P @ H.T + self.R_depth
        
        # Kalman gain
        K = self.P @ H.T / S[0, 0]
        
        # State update
        self.x = self.x + K.flatten() * y
        
        # Covariance update
        I = np.eye(6)
        self.P = (I - np.outer(K, H)) @ self.P
        
    def _rotation_matrix(self, roll, pitch, yaw):
        """Matrice di rotazione da body frame a world frame (ZYX)"""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        return R
    
    def get_state(self):
        """Ritorna stato corrente"""
        return self.x.copy()
    
    def get_covariance(self):
        """Ritorna covarianza corrente"""
        return self.P.copy()

# ============================================================================
# FUSIONE SENSORI
# ============================================================================

def run_sensor_fusion(imu_data, usbl_data, depth_data, sensor_noise, config=None, initial_state=None):
    """
    Esegue la fusione sensoriale con EKF
    
    Returns:
        trajectory: array Nx7 [timestamp, x, y, z, vx, vy, vz]
    """
    print("\n" + "="*70)
    print("ESECUZIONE FUSIONE SENSORIALE")
    print("="*70)
    
    # Calcola dt medio dall'IMU
    dt_imu = np.diff(imu_data['timestamp_rel'].values)
    dt_mean = np.mean(dt_imu)
    print(f"\nFrequenza IMU: {1/dt_mean:.1f} Hz (dt = {dt_mean:.4f} s)")
    
    # Inizializza EKF
    ekf = EKF_AUV(sensor_noise, dt=dt_mean, config=config, initial_state=initial_state)
    
    # Prepara dati
    time_imu = imu_data['timestamp_rel'].values
    time_usbl = usbl_data['timestamp_rel'].values
    time_depth = depth_data['timestamp_rel'].values if len(depth_data) > 0 else np.array([])
    
    # Bias correction
    bias_acc = sensor_noise['bias_acc'] if APPLY_BIAS_CORRECTION else np.zeros(3)
    bias_gyr = sensor_noise['bias_gyr'] if APPLY_BIAS_CORRECTION else np.zeros(3)
    
    # Storage trajectory
    trajectory = []
    
    # Indici per sincronizzazione sensori
    idx_usbl = 0
    idx_depth = 0
    
    print(f"\nInizio fusione...")
    print(f"  Campioni IMU:   {len(imu_data)}")
    print(f"  Fix USBL:       {len(usbl_data)}")
    print(f"  Misure depth:   {len(depth_data)}")
    
    # Loop principale
    for i in range(len(imu_data)):
        t = time_imu[i]
        
        # Lettura IMU (corretta per bias)
        acc_body = np.array([
            imu_data.iloc[i]['acc_x'] - bias_acc[0],
            imu_data.iloc[i]['acc_y'] - bias_acc[1],
            imu_data.iloc[i]['acc_z'] - bias_acc[2]
        ])
        
        gyro = np.array([
            imu_data.iloc[i]['gyr_x'] - bias_gyr[0],
            imu_data.iloc[i]['gyr_y'] - bias_gyr[1],
            imu_data.iloc[i]['gyr_z'] - bias_gyr[2]
        ])
        
        roll = imu_data.iloc[i]['roll']
        pitch = imu_data.iloc[i]['pitch']
        yaw = imu_data.iloc[i]['yaw']
        
        # Prediction step
        ekf.predict(acc_body, gyro, roll, pitch, yaw)
        
        # Update USBL (se disponibile)
        if idx_usbl < len(usbl_data) and abs(t - time_usbl[idx_usbl]) < dt_mean:
            x_meas = usbl_data.iloc[idx_usbl]['usbl_x']
            y_meas = usbl_data.iloc[idx_usbl]['usbl_y']
            ekf.update_usbl_position(x_meas, y_meas)
            idx_usbl += 1
            
            if VERBOSE and idx_usbl % 10 == 0:
                print(f"  t={t:6.2f}s: USBL update #{idx_usbl}, pos=({x_meas:.2f}, {y_meas:.2f})")
        
        # Update Depth (se disponibile)
        if len(time_depth) > 0 and idx_depth < len(depth_data):
            if abs(t - time_depth[idx_depth]) < dt_mean:
                depth_meas = depth_data.iloc[idx_depth]['depth']
                ekf.update_depth(depth_meas)
                idx_depth += 1
        
        # Salva stato
        state = ekf.get_state()
        trajectory.append([t, state[0], state[1], state[2], 
                          state[3], state[4], state[5]])
    
    trajectory = np.array(trajectory)
    
    print(f"\nFusione completata!")
    print(f"  Traiettoria: {len(trajectory)} punti")
    print(f"  USBL update: {idx_usbl}/{len(usbl_data)}")
    
    return trajectory

# ============================================================================
# VISUALIZZAZIONE
# ============================================================================

def plot_results(trajectory, usbl_data, depth_data, csv_estimate, show_csv=True):
    """
    Visualizza risultati della fusione sensoriale
    """
    print("\n" + "="*70)
    print("VISUALIZZAZIONE")
    print("="*70)
    
    fig = plt.figure(figsize=(18, 7))
    
    # Estrai dati
    time_ekf = trajectory[:, 0]
    x_ekf = trajectory[:, 1]
    y_ekf = trajectory[:, 2]
    z_ekf = trajectory[:, 3]
    vx = trajectory[:, 4]
    vy = trajectory[:, 5]
    
    time_usbl = usbl_data['timestamp_rel'].values
    usbl_x = usbl_data['usbl_x'].values
    usbl_y = usbl_data['usbl_y'].values
    
    time_csv = csv_estimate['timestamp_rel'].values
    csv_x = csv_estimate['est_x'].values
    csv_y = csv_estimate['est_y'].values
    csv_z = csv_estimate['est_z'].values
    
    # --- SUBPLOT 1: Mappa 2D ---
    ax1 = fig.add_subplot(131, aspect='equal')
    
    # Transceiver USBL (origine)
    ax1.plot(0, 0, 'k^', markersize=15, label='USBL Transceiver', zorder=10)
    
    # Fix USBL sparsi
    ax1.scatter(usbl_x, usbl_y, c='cyan', s=30, alpha=0.6, 
                edgecolors='blue', linewidths=0.5,
                label=f'USBL Fixes ({len(usbl_data)})', zorder=3)
    
    # Stima CSV originale (opzionale)
    if show_csv and SHOW_CSV_ESTIMATE:
        ax1.plot(csv_x, csv_y, '--', color='gray', 
                 linewidth=1.5, alpha=0.7, label='CSV Estimate (Original)')
    
    # Traccia EKF (colorata per tempo)
    points = np.array([x_ekf, y_ekf]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    norm = plt.Normalize(time_ekf.min(), time_ekf.max())
    lc = LineCollection(segments, cmap='plasma', norm=norm, linewidth=2.5, 
                       label='EKF New', zorder=4)
    lc.set_array(time_ekf[:-1])
    ax1.add_collection(lc)
    
    # Marker start/end
    ax1.plot(x_ekf[0], y_ekf[0], 'go', markersize=12, label='Start', zorder=6)
    ax1.plot(x_ekf[-1], y_ekf[-1], 'rs', markersize=12, label='End', zorder=6)
    
    # Colorbar
    cbar = plt.colorbar(lc, ax=ax1, label='Tempo [s]')
    
    # Assi auto
    all_x = np.concatenate([x_ekf, usbl_x])
    all_y = np.concatenate([y_ekf, usbl_y])
    margin = max(np.ptp(all_x), np.ptp(all_y)) * 0.1 + 1
    ax1.set_xlim(all_x.min() - margin, all_x.max() + margin)
    ax1.set_ylim(all_y.min() - margin, all_y.max() + margin)
    
    ax1.set_xlabel('X [m] (Est)', fontsize=12)
    ax1.set_ylabel('Y [m] (Nord)', fontsize=12)
    ax1.set_title('Traiettoria EKF (XY Update)', fontsize=14, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend(loc='upper right', fontsize=8)
    ax1.axhline(y=0, color='k', linewidth=0.5, alpha=0.3)
    ax1.axvline(x=0, color='k', linewidth=0.5, alpha=0.3)
    
    # --- SUBPLOT 2: Confronto X,Y ---
    ax2 = fig.add_subplot(132)
    
    ax2.plot(time_usbl, usbl_x, 'o', color='steelblue', markersize=4, 
             alpha=0.6, label='USBL X')
    ax2.plot(time_usbl, usbl_y, 's', color='forestgreen', markersize=4, 
             alpha=0.6, label='USBL Y')
    ax2.plot(time_ekf, x_ekf, '-', color='blue', linewidth=1.5, 
             alpha=0.8, label='EKF X')
    ax2.plot(time_ekf, y_ekf, '-', color='green', linewidth=1.5, 
             alpha=0.8, label='EKF Y')
    
    if show_csv and SHOW_CSV_ESTIMATE:
        ax2.plot(time_csv, csv_x, '--', color='lightblue', linewidth=1, 
                 alpha=0.5, label='CSV X')
        ax2.plot(time_csv, csv_y, '--', color='lightgreen', linewidth=1, 
                 alpha=0.5, label='CSV Y')
    
    ax2.set_xlabel('Tempo [s]', fontsize=12)
    ax2.set_ylabel('Posizione [m]', fontsize=12)
    ax2.set_title('Confronto Coordinate X,Y', fontsize=14, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend(loc='best', fontsize=8)
    
    # --- SUBPLOT 3: Profondità + Velocità ---
    ax3 = fig.add_subplot(133)
    
    # Profondità
    depth = depth_data['depth'].values
    time_depth = depth_data['timestamp_rel'].values
    ax3.plot(time_depth, depth, 'c-', linewidth=1, alpha=0.5, label='Depth Sensor')
    ax3.plot(time_ekf, z_ekf, 'b-', linewidth=1.5, label='EKF Z')
    
    if show_csv and SHOW_CSV_ESTIMATE:
        ax3.plot(time_csv, csv_z, '--', color='gray', linewidth=1, 
                 alpha=0.5, label='CSV Z')
    
    ax3.set_xlabel('Tempo [s]', fontsize=12)
    ax3.set_ylabel('Profondità [m]', fontsize=12)
    ax3.set_title('Profondità', fontsize=14, fontweight='bold')
    ax3.grid(True, alpha=0.3)
    ax3.legend(loc='best', fontsize=8)
    ax3.invert_yaxis()  # Profondità cresce verso il basso
    
    plt.tight_layout()
    
    # Statistiche finali
    print(f"\n=== STATISTICHE FINALI ===")
    print(f"\nTraiettoria EKF:")
    print(f"  Durata: {time_ekf[-1] - time_ekf[0]:.2f} s")
    
    dist = np.sum(np.sqrt(np.diff(x_ekf)**2 + np.diff(y_ekf)**2))
    print(f"  Distanza percorsa: {dist:.2f} m")
    print(f"  Posizione iniziale: ({x_ekf[0]:.3f}, {y_ekf[0]:.3f}, {z_ekf[0]:.3f}) m")
    print(f"  Posizione finale:   ({x_ekf[-1]:.3f}, {y_ekf[-1]:.3f}, {z_ekf[-1]:.3f}) m")
    
    speed_2d = np.sqrt(vx**2 + vy**2)
    print(f"\nVelocità:")
    print(f"  Media 2D: {np.mean(speed_2d):.3f} m/s")
    print(f"  Max 2D:   {np.max(speed_2d):.3f} m/s")
    
    # Errore vs USBL
    usbl_errors = []
    for i, t_usbl in enumerate(time_usbl):
        idx = np.argmin(np.abs(time_ekf - t_usbl))
        error = np.sqrt((x_ekf[idx] - usbl_x[i])**2 + (y_ekf[idx] - usbl_y[i])**2)
        usbl_errors.append(error)
    
    usbl_errors = np.array(usbl_errors)
    print(f"\nErrore vs USBL:")
    print(f"  Media: {np.mean(usbl_errors):.3f} m")
    print(f"  RMS:   {np.sqrt(np.mean(usbl_errors**2)):.3f} m")
    print(f"  Max:   {np.max(usbl_errors):.3f} m")
    
    # Errore vs CSV estimate
    if show_csv and SHOW_CSV_ESTIMATE:
        csv_errors = np.sqrt((x_ekf - csv_x)**2 + (y_ekf - csv_y)**2)
        print(f"\nErrore vs CSV Estimate:")
        print(f"  Media: {np.mean(csv_errors):.3f} m")
        print(f"  RMS:   {np.sqrt(np.mean(csv_errors**2)):.3f} m")
    
    plt.savefig('ekf_fusion_result.png', dpi=150, bbox_inches='tight')
    print(f"\nGrafico salvato: ekf_fusion_result.png")
    
    return fig

# ============================================================================
# MAIN
# ============================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='EKF Sensor Fusion per AUV')
    parser.add_argument('--log', type=str, default=DEFAULT_LOG_PATH,
                        help=f'Path al file kalman_*.csv (default: {DEFAULT_LOG_PATH})')
    parser.add_argument('--no-csv-estimate', action='store_true',
                        help='Non mostrare stima CSV originale')
    parser.add_argument('--calibration', type=str, default=None,
                        help='Path a imu_calibration.json')
    args = parser.parse_args()
    
    # Override globale
    if args.no_csv_estimate:
        SHOW_CSV_ESTIMATE = False
    
    print("="*70)
    print("EKF SENSOR FUSION - AUV NAVIGATION (Mare Aperto)")
    print("="*70)
    
    # Carica calibrazione IMU statica (se disponibile)
    imu_calibration = None
    if args.calibration:
        try:
            with open(args.calibration, 'r') as f:
                imu_calibration = json.load(f)
            print(f"✓ Calibrazione IMU caricata da {args.calibration}")
        except FileNotFoundError:
            print(f"⚠ {args.calibration} non trovato")
    
    # Carica dati dal log unificato
    imu_data, usbl_data, depth_data, csv_estimate = load_kalman_log(args.log)
    
    # Analisi rumore sensori
    sensor_noise = analyze_sensor_noise(imu_data, usbl_data, depth_data, imu_calibration)
    
    # Stato iniziale dal primo fix USBL
    initial_state = None
    if len(usbl_data) > 0:
        initial_state = {
            'position': [usbl_data.iloc[0]['usbl_x'], 
                        usbl_data.iloc[0]['usbl_y'], 
                        depth_data.iloc[0]['depth']],
            'velocity': [0.0, 0.0, 0.0]
        }
    
    # Esegui fusione
    trajectory = run_sensor_fusion(imu_data, usbl_data, depth_data, 
                                   sensor_noise, initial_state=initial_state)
    
    # Visualizza
    fig = plot_results(trajectory, usbl_data, depth_data, csv_estimate, 
                       show_csv=SHOW_CSV_ESTIMATE)
    
    plt.show()