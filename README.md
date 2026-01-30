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
│       └── Giorno*/kalman_*/  # 9 PNG per ogni log
│
├── scripts/
│   ├── ekf_sensor_fusion.py   # EKF interattivo (singolo log)
│   ├── analyze_sensors.py     # Grafici sensori (singolo log)
│   ├── batch_analyze.py       # Batch grafici sensori (tutti i log)
│   └── batch_ekf.py           # Batch EKF comparison (tutti i log)
│
└── old-code/                  # Codice legacy
```

---

## 🚀 Quick Start

```bash
cd scripts

# Batch: genera TUTTI i grafici per tutti i log
python batch_analyze.py   # 8 grafici sensori per log
python batch_ekf.py       # 1 grafico EKF comparison per log

# Singolo log
python analyze_sensors.py --log ../logs/Giorno1/kalman_HE9.csv
python ekf_sensor_fusion.py --log ../logs/Giorno1/kalman_HE9.csv
```

---

## Scripts

| Script | Descrizione |
|--------|-------------|
| `batch_analyze.py` | Genera grafici sensori per tutti i log |
| `batch_ekf.py` | Esegue EKF e confronta con stima CSV per tutti i log |
| `analyze_sensors.py` | Grafici sensori per singolo log |
| `ekf_sensor_fusion.py` | EKF interattivo con visualizzazione |

---

## 📊 Output Grafici (sensor_plots/)

Ogni cartella `sensor_plots/Giorno*/kalman_*/` contiene **9 PNG**:

### Grafici Sensori (da `batch_analyze.py`)

| File | Contenuto |
|------|-----------|
| `overview.png` | Panoramica: Yaw, Depth, |Acc|, |Gyro|, USBL, Temp |
| `orientation.png` | Roll, Pitch, Yaw nel tempo |
| `acceleration.png` | Accelerazione X, Y, Z |
| `gyroscope.png` | Velocità angolare X, Y, Z |
| `magnetometer.png` | Campo magnetico X, Y, Z |
| `usbl.png` | Timeline USBL X,Y + mappa posizioni |
| `depth_temp.png` | Profondità + temperatura |
| `kalman_estimate.png` | Stima KF originale (dal CSV) |

### Grafico EKF (da `batch_ekf.py`)

| File | Contenuto |
|------|-----------|
| `ekf_comparison.png` | **4 subplot**: mappa 2D, X vs tempo, Y vs tempo, Z vs tempo. Confronta EKF (nuovo) vs CSV estimate (originale) vs USBL fixes |

---

## EKF

L'EKF usa update **USBL X,Y diretto** (non range-only):
- **Stato**: `[x, y, z, vx, vy, vz]`
- **Prediction**: IMU (acc, gyro, orientamento)
- **Update USBL**: coordinate X,Y dirette → osservazione lineare
- **Update Depth**: profondimetro

---

## Formato Log

File `kalman_*.csv` con 23 colonne. Vedi [KALMAN_LOG_COLUMNS.md](KALMAN_LOG_COLUMNS.md).

---

## Dipendenze

```bash
pip install numpy pandas matplotlib
```
