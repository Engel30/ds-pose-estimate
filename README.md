# DS State Estimate

Strumenti per stima dello stato e analisi dati sensori AUV DiveSafe.

---

## � Struttura Repository

```
ds-state-estimate/
├── README.md                  # Questo file
├── KALMAN_LOG_COLUMNS.md      # Documentazione formato log (23 colonne)
├── convert_logs_to_csv.py     # Conversione log raw → CSV
├── logs/                      # Log sperimentali
│   ├── Giorno1/               # kalman_HE9.csv, ecc.
│   └── ...
├── scripts/                   # Script principali
│   ├── ekf_sensor_fusion.py   # EKF sensor fusion
│   ├── analyze_sensors.py     # Generazione grafici sensori
│   └── sensor_plots/          # Output grafici PNG
└── old-code/                  # Codice legacy
```

---

## 🚀 Quick Start

```bash
cd scripts

# Analisi sensori (genera 8 grafici PNG)
python analyze_sensors.py

# EKF Sensor Fusion
python ekf_sensor_fusion.py
```

---

## Scripts

### ekf_sensor_fusion.py

EKF che fonde:
- **IMU/AHRS** (Xsens MTI-670): Roll, Pitch, Yaw, Acc, Gyro
- **USBL** (Evologics): Coordinate X,Y dirette
- **Profondimetro** (MS5837-30BA): Profondità

```bash
python ekf_sensor_fusion.py --log ../logs/Giorno1/kalman_HE9.csv
python ekf_sensor_fusion.py --no-csv-estimate  # Senza confronto CSV
```

**Output:** `ekf_fusion_result.png`

---

### analyze_sensors.py

Genera 8 grafici PNG per ogni gruppo di sensori:

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
python analyze_sensors.py --output sensor_plots
```

---

## Configurazione

Modifica `DEFAULT_LOG_PATH` in cima agli script:

```python
DEFAULT_LOG_PATH = "../logs/Giorno1/kalman_HE9.csv"
```

---

## Formato Log

I file `kalman_*.csv` contengono 23 colonne. Vedi [KALMAN_LOG_COLUMNS.md](KALMAN_LOG_COLUMNS.md) per dettagli.

---

## Dipendenze

```bash
pip install numpy pandas matplotlib
```
