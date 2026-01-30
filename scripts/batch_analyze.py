#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Batch Sensor Analyzer - Processa tutti i log kalman_*.csv
Genera grafici per ogni log in struttura mirrored:
    logs/Giorno1/kalman_HE9.csv -> sensor_plots/Giorno1/kalman_HE9/

Uso:
    python batch_analyze.py           # Processa tutti i log
    python batch_analyze.py --dry-run # Mostra cosa verrebbe processato
"""

import os
import sys
import glob
import argparse

# Importa le funzioni di plotting da analyze_sensors
from analyze_sensors import (
    load_kalman_log,
    plot_overview,
    plot_orientation,
    plot_acceleration,
    plot_gyroscope,
    plot_magnetometer,
    plot_usbl,
    plot_depth,
    plot_kalman_estimate
)

# ============================================================================
# CONFIGURAZIONE
# ============================================================================

LOGS_DIR = "../logs"
OUTPUT_BASE = "../logs/sensor_plots"

# ============================================================================
# BATCH PROCESSING
# ============================================================================

def find_all_logs(logs_dir):
    """Trova tutti i file kalman_*.csv nelle sottocartelle Giorno*"""
    logs = []
    
    # Cerca cartelle Giorno*
    giorno_dirs = sorted(glob.glob(os.path.join(logs_dir, "Giorno*")))
    
    for giorno_dir in giorno_dirs:
        if os.path.isdir(giorno_dir):
            giorno_name = os.path.basename(giorno_dir)
            
            # Trova tutti i kalman_*.csv
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
    """Processa un singolo log e genera i grafici"""
    # Crea output directory: sensor_plots/Giorno1/kalman_HE9/
    output_dir = os.path.join(output_base, log_info['giorno'], log_info['name'])
    
    if dry_run:
        print(f"  [DRY-RUN] {log_info['path']} -> {output_dir}")
        return True
    
    print(f"\n{'='*60}")
    print(f"Processing: {log_info['giorno']}/{log_info['name']}")
    print(f"{'='*60}")
    
    # Crea directory
    os.makedirs(output_dir, exist_ok=True)
    
    try:
        # Carica dati
        df = load_kalman_log(log_info['path'])
        
        # Genera tutti i grafici
        print("\nGenerazione grafici...")
        plot_overview(df, output_dir)
        plot_orientation(df, output_dir)
        plot_acceleration(df, output_dir)
        plot_gyroscope(df, output_dir)
        plot_magnetometer(df, output_dir)
        plot_usbl(df, output_dir)
        plot_depth(df, output_dir)
        plot_kalman_estimate(df, output_dir)
        
        print(f"✓ Completato: {output_dir}")
        return True
        
    except Exception as e:
        print(f"✗ ERRORE: {e}")
        return False

def main():
    parser = argparse.ArgumentParser(description='Batch processa tutti i log kalman')
    parser.add_argument('--logs-dir', type=str, default=LOGS_DIR,
                        help=f'Directory con log (default: {LOGS_DIR})')
    parser.add_argument('--output', type=str, default=OUTPUT_BASE,
                        help=f'Directory output (default: {OUTPUT_BASE})')
    parser.add_argument('--dry-run', action='store_true',
                        help='Mostra cosa verrebbe processato senza eseguire')
    args = parser.parse_args()
    
    print("="*70)
    print("BATCH SENSOR ANALYZER")
    print("="*70)
    
    # Trova tutti i log
    logs = find_all_logs(args.logs_dir)
    
    if not logs:
        print(f"\n✗ Nessun log trovato in {args.logs_dir}")
        sys.exit(1)
    
    print(f"\nTrovati {len(logs)} log da processare:")
    for log in logs:
        print(f"  - {log['giorno']}/{log['name']}")
    
    if args.dry_run:
        print("\n[DRY-RUN MODE]")
    
    # Processa ogni log
    success = 0
    failed = 0
    
    for log in logs:
        if process_single_log(log, args.output, args.dry_run):
            success += 1
        else:
            failed += 1
    
    # Summary
    print("\n" + "="*70)
    print("RIEPILOGO")
    print("="*70)
    print(f"  Processati con successo: {success}")
    print(f"  Falliti: {failed}")
    print(f"  Output: {os.path.abspath(args.output)}")
    print("="*70)

if __name__ == "__main__":
    main()
