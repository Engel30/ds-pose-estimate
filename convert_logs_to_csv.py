#!/usr/bin/env python3
"""
Script per convertire i file di log kalman_*.txt in CSV con header.
Mantiene la stessa struttura di cartelle e nomi file originali.

Basato sull'analisi del codice MAIN_jetson.py e LOG_ANALYSIS.md
"""

import os
import csv
import argparse
from pathlib import Path


# Header delle colonne basato sull'analisi del codice MAIN_jetson.py (funzione fusion)
# e LOG_ANALYSIS.md
KALMAN_HEADER = [
    "Timestamp",      # 0: Unix Epoch Time (time.time())
    "Roll",           # 1: Angolo Roll (gradi) - da AHRS Xsens
    "Pitch",          # 2: Angolo Pitch (gradi) - da AHRS Xsens
    "Yaw",            # 3: Angolo Yaw (gradi) - da AHRS Xsens
    "Acc_X",          # 4: Accelerazione X (body frame)
    "Acc_Y",          # 5: Accelerazione Y (body frame)
    "Acc_Z",          # 6: Accelerazione Z (body frame)
    "USBL_X",         # 7: Posizione USBL X (ENU, metri) - 0 se nessuna lettura
    "USBL_Y",         # 8: Posizione USBL Y (ENU, metri) - 0 se nessuna lettura
    "Depth",          # 9: Profondita (m) - da sensore pressione MS5837
    "Est_X",          # 10: Stima Filtro X (output Kalman)
    "Est_Y",          # 11: Stima Filtro Y (output Kalman)
    "Est_Z",          # 12: Stima Filtro Z (output Kalman)
    "Temp",           # 13: Temperatura (C) - da sensore pressione
    "State_6",        # 14: Stato interno filtro (bias/vel)
    "State_7",        # 15: Stato interno filtro (bias/vel)
    "State_8",        # 16: Stato interno filtro (bias/vel)
    "Gyro_X",         # 17: Giroscopio X (raw)
    "Gyro_Y",         # 18: Giroscopio Y (raw)
    "Gyro_Z",         # 19: Giroscopio Z (raw)
    "Mag_X",          # 20: Magnetometro X (raw)
    "Mag_Y",          # 21: Magnetometro Y (raw)
    "Mag_Z",          # 22: Magnetometro Z (raw)
]


def convert_kalman_log(input_path: Path, output_path: Path) -> dict:
    """
    Converte un file kalman log in CSV con header.

    Args:
        input_path: Percorso del file .txt originale
        output_path: Percorso del file .csv di output

    Returns:
        Dizionario con statistiche della conversione
    """
    stats = {
        "input_file": str(input_path),
        "output_file": str(output_path),
        "rows_processed": 0,
        "rows_with_full_data": 0,
        "rows_with_partial_data": 0,
        "errors": []
    }

    rows = []

    with open(input_path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue

            # Separa per tab o spazi multipli
            fields = line.split()

            if len(fields) < 14:
                stats["errors"].append(f"Linea {line_num}: solo {len(fields)} campi (minimo 14)")
                continue

            stats["rows_processed"] += 1

            # Normalizza la riga a 23 colonne
            # Alcune righe (es. prima riga) potrebbero avere meno campi
            if len(fields) >= 23:
                stats["rows_with_full_data"] += 1
                row = fields[:23]
            else:
                stats["rows_with_partial_data"] += 1
                # Padding con valori vuoti per le colonne mancanti
                row = fields + [''] * (23 - len(fields))

            rows.append(row)

    # Scrivi il CSV
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(KALMAN_HEADER)
        writer.writerows(rows)

    return stats


def process_logs_directory(logs_dir: Path, in_place: bool = True) -> list:
    """
    Processa tutti i file kalman_*.txt nelle sottocartelle.

    Args:
        logs_dir: Directory radice contenente le cartelle Giorno*
        in_place: Se True, crea i CSV nella stessa cartella dei .txt

    Returns:
        Lista delle statistiche per ogni file processato
    """
    all_stats = []

    # Trova tutti i file kalman_*.txt ricorsivamente
    kalman_files = list(logs_dir.glob("**/kalman_*.txt"))

    if not kalman_files:
        print(f"Nessun file kalman_*.txt trovato in {logs_dir}")
        return all_stats

    print(f"Trovati {len(kalman_files)} file da convertire\n")

    for txt_file in sorted(kalman_files):
        # Genera il nome del file CSV (stesso nome, estensione .csv)
        csv_file = txt_file.with_suffix('.csv')

        print(f"Convertendo: {txt_file.relative_to(logs_dir)}")

        try:
            stats = convert_kalman_log(txt_file, csv_file)
            all_stats.append(stats)

            print(f"  -> {csv_file.name}")
            print(f"     Righe: {stats['rows_processed']} "
                  f"(complete: {stats['rows_with_full_data']}, "
                  f"parziali: {stats['rows_with_partial_data']})")

            if stats["errors"]:
                print(f"     Avvisi: {len(stats['errors'])}")
                for err in stats["errors"][:3]:  # Mostra solo i primi 3
                    print(f"       - {err}")
                if len(stats["errors"]) > 3:
                    print(f"       ... e altri {len(stats['errors']) - 3}")
            print()

        except Exception as e:
            print(f"  ERRORE: {e}\n")
            all_stats.append({
                "input_file": str(txt_file),
                "error": str(e)
            })

    return all_stats


def main():
    parser = argparse.ArgumentParser(
        description="Converte i file kalman log in CSV con header"
    )
    parser.add_argument(
        "logs_dir",
        nargs="?",
        default="logs",
        help="Directory contenente i log (default: logs)"
    )

    args = parser.parse_args()
    logs_path = Path(args.logs_dir)

    if not logs_path.exists():
        print(f"Errore: la directory '{logs_path}' non esiste")
        return 1

    print("=" * 60)
    print("Conversione Log Kalman -> CSV")
    print("=" * 60)
    print(f"Directory: {logs_path.absolute()}\n")

    stats = process_logs_directory(logs_path)

    # Riepilogo finale
    print("=" * 60)
    print("RIEPILOGO")
    print("=" * 60)

    total_files = len(stats)
    successful = sum(1 for s in stats if "error" not in s)
    total_rows = sum(s.get("rows_processed", 0) for s in stats)

    print(f"File processati: {successful}/{total_files}")
    print(f"Righe totali: {total_rows}")
    print(f"\nI file CSV sono stati creati nelle stesse cartelle dei file originali.")

    return 0


if __name__ == "__main__":
    exit(main())
