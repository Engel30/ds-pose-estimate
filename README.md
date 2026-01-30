# DS State Estimate

Strumenti per stima dello stato e analisi dati sensori AUV DiveSafe.

---

## 📂 Struttura Repository

```
ds-state-estimate/
├── README.md                  # Questo file
├── KALMAN_LOG_COLUMNS.md      # Documentazione formato log (23 colonne)
├── convert_logs_to_csv.py     # Conversione log raw → CSV
│
├── logs/                      # Log sperimentali
│   ├── Giorno1/               # kalman_HE9.csv, kalman_LC14.csv, ...
│   ├── Giorno2/
│   ├── Giorno3/
│   └── sensor_plots/          # Output grafici (generato)
│       ├── Giorno1/
│       │   ├── kalman_HE9/    # 8 PNG per log
│       │   └── ...
│       └── ...
│
├── scripts/
│   ├── ekf_sensor_fusion.py   # EKF sensor fusion
│   ├── analyze_sensors.py     # Generazione grafici singolo log
│   └── batch_analyze.py       # Batch processing tutti i log
│
└── old-code/                  # Codice legacy
```

---

## 🚀 Quick Start

```bash
cd scripts

# Batch: genera grafici per TUTTI i log (14 log → 112 PNG)
python batch_analyze.py

# Singolo log: analisi sensori
python analyze_sensors.py --log ../logs/Giorno1/kalman_HE9.csv

# EKF Sensor Fusion
python ekf_sensor_fusion.py
```

---

## Scripts

### batch_analyze.py

Processa tutti i `kalman_*.csv` in `logs/Giorno*` e genera grafici in struttura mirrored.

```bash
python batch_analyze.py           # Processa tutto
python batch_analyze.py --dry-run # Preview senza eseguire
```

**Output:** `logs/sensor_plots/Giorno*/kalman_*/` (8 PNG per log)

---

### analyze_sensors.py

Genera 8 grafici PNG per un singolo log:

| Grafico | Contenuto |
|---------|-----------|
| `overview.png` | Panoramica tutti i sensori |
| `orientation.png` | Roll, Pitch, Yaw |
| `acceleration.png` | Acc X,Y,Z |
| `gyroscope.png` | Gyro X,Y,Z |
| `magnetometer.png` | Mag X,Y,Z |
| `usbl.png` | Timeline + mappa USBL |
| `depth_temp.png` | Profondità + temperatura |
| `kalman_estimate.png` | Stima KF originale |

```bash
python analyze_sensors.py --log ../logs/Giorno1/kalman_HE9.csv
```

---

### ekf_sensor_fusion.py

EKF che fonde:
- **IMU/AHRS** (Xsens MTI-670): Roll, Pitch, Yaw, Acc, Gyro
- **USBL** (Evologics): Coordinate X,Y dirette
- **Profondimetro** (MS5837-30BA): Profondità

```bash
python ekf_sensor_fusion.py
python ekf_sensor_fusion.py --no-csv-estimate
```

**Output:** `ekf_fusion_result.png`

---

## Configurazione

Modifica `DEFAULT_LOG_PATH` in cima agli script per cambiare log di default:

```python
DEFAULT_LOG_PATH = "../logs/Giorno1/kalman_HE9.csv"
```

---

## Formato Log

I file `kalman_*.csv` contengono 23 colonne. Vedi [KALMAN_LOG_COLUMNS.md](KALMAN_LOG_COLUMNS.md).

---

## Dipendenze

```bash
pip install numpy pandas matplotlib
```
