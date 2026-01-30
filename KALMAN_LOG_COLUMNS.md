# Documentazione Colonne CSV - Log Kalman DiveSafe

Questo documento descrive in dettaglio ogni colonna dei file `kalman_*.csv`, basandosi sull'analisi del codice sorgente in `MainDPV/HE_DPV-DS_USBL/`.

---

## Panoramica del Sistema

Il sistema di navigazione DiveSafe utilizza:
- **IMU/AHRS**: Xsens MTI-670 (orientamento, accelerazione, giroscopio, magnetometro)
- **Profondimetro**: MS5837-30BA (pressione/profondità, temperatura)
- **Posizionamento acustico**: Modem Evologics (ricezione coordinate ENU via messaggi RECVIM)
- **Fusione sensori**: Filtro di Kalman a 9 stati

---

## Colonne del CSV

| # | Nome | Unità | Sorgente | Descrizione |
|---|------|-------|----------|-------------|
| 0 | `Timestamp` | secondi (Unix Epoch) | `time.time()` | Tempo di sistema Jetson al momento della registrazione |
| 1 | `Roll` | gradi | Xsens MTI-670 | Angolo di rollio (rotazione attorno all'asse X body) |
| 2 | `Pitch` | gradi | Xsens MTI-670 | Angolo di beccheggio (rotazione attorno all'asse Y body) |
| 3 | `Yaw` | gradi | Xsens MTI-670 | Angolo di imbardata (rotazione attorno all'asse Z body) |
| 4 | `Acc_X` | m/s² | Xsens MTI-670 | Accelerazione lineare asse X (body frame) |
| 5 | `Acc_Y` | m/s² | Xsens MTI-670 | Accelerazione lineare asse Y (body frame) |
| 6 | `Acc_Z` | m/s² | Xsens MTI-670 | Accelerazione lineare asse Z (body frame) |
| 7 | `USBL_X` | metri | Modem Evologics | Posizione X in frame ENU (Est). **0 se nessun fix** |
| 8 | `USBL_Y` | metri | Modem Evologics | Posizione Y in frame ENU (Nord). **0 se nessun fix** |
| 9 | `Depth` | metri | MS5837-30BA | Profondità dal sensore di pressione |
| 10 | `Est_X` | metri | Filtro Kalman | Posizione X stimata (stato x[0]) |
| 11 | `Est_Y` | metri | Filtro Kalman | Posizione Y stimata (stato x[1]) |
| 12 | `Est_Z` | metri | Filtro Kalman | Posizione Z stimata (stato x[2], negativa = sott'acqua) |
| 13 | `Temp` | °C | MS5837-30BA | Temperatura acqua |
| 14 | `State_6` | m/s² | Filtro Kalman | Accelerazione filtrata asse X (stato x[6]) |
| 15 | `State_7` | m/s² | Filtro Kalman | Accelerazione filtrata asse Y (stato x[7]) |
| 16 | `State_8` | m/s² | Filtro Kalman | Accelerazione filtrata asse Z (stato x[8]) |
| 17 | `Gyro_X` | rad/s | Xsens MTI-670 | Velocità angolare asse X |
| 18 | `Gyro_Y` | rad/s | Xsens MTI-670 | Velocità angolare asse Y |
| 19 | `Gyro_Z` | rad/s | Xsens MTI-670 | Velocità angolare asse Z |
| 20 | `Mag_X` | normalizzato | Xsens MTI-670 | Campo magnetico asse X |
| 21 | `Mag_Y` | normalizzato | Xsens MTI-670 | Campo magnetico asse Y |
| 22 | `Mag_Z` | normalizzato | Xsens MTI-670 | Campo magnetico asse Z |

---

## Dettaglio Filtro di Kalman

### Vettore di Stato (9 elementi)

Il filtro di Kalman implementato in `lib/KF.py` utilizza un vettore di stato a 9 componenti:

```
x = [X, Y, Z, Vx, Vy, Vz, Ax_filt, Ay_filt, Az_filt]ᵀ
     ├─────┤  ├────────┤  ├──────────────────────────┤
     Posizione  Velocità    Accelerazione filtrata
     (ENU)      (ENU)       (body → ENU)
```

| Indice | Stato | Significato |
|--------|-------|-------------|
| x[0] | X | Posizione Est (metri) |
| x[1] | Y | Posizione Nord (metri) |
| x[2] | Z | Posizione Up (metri, negativo = profondità) |
| x[3] | Vx | Velocità Est (m/s) |
| x[4] | Vy | Velocità Nord (m/s) |
| x[5] | Vz | Velocità Up (m/s) |
| x[6] | Ax_filt | Accelerazione filtrata X (m/s²) |
| x[7] | Ay_filt | Accelerazione filtrata Y (m/s²) |
| x[8] | Az_filt | Accelerazione filtrata Z (m/s²) |

### Modello Dinamico

La matrice di transizione `A` (da `KF.py:9-14`) implementa:

```
Posizione(k+1) = Posizione(k) + T·R·Velocità(k) + (T²/2)·R·Accelerazione(k)
Velocità(k+1)  = Velocità(k) + T·Accelerazione(k)
Accelerazione(k+1) = Accelerazione(k)  [random walk]
```

Dove:
- `T` = intervallo di campionamento (~0.2s)
- `R = Rxyz(roll, pitch, yaw)` = matrice di rotazione body → ENU

### Modalità di Aggiornamento

Il filtro opera in due modalità:

1. **Solo IMU** (`queue=True`): Usa matrice di osservazione `C1`
   - Osserva: accelerazione filtrata (stati 6-8)
   - Input: `y = [facc_x, facc_y, facc_z]` (accelerazione senza gravità)

2. **IMU + USBL** (`queue=False`): Usa matrice di osservazione `C2`
   - Osserva: accelerazione filtrata + posizione (stati 0-2, 6-8)
   - Input: `y = [facc_x, facc_y, facc_z, usbl_x, usbl_y, -depth]`

### Compensazione Gravità

L'accelerazione loggata (`Acc_X/Y/Z`) è quella **raw** dall'IMU. Il filtro usa invece l'accelerazione **free** (senza gravità):

```python
# Da MAIN_jetson.py:393-395
acc = [ahrs.getAcc()]  # accelerazione raw
g = [0, 0, 9.814]      # gravità
facc = acc - Rxy(-roll, -pitch) * g  # accelerazione free
```

---

## Dettaglio USBL

### Modalità di Ricezione

**IMPORTANTE**: Il sistema **NON** usa la modalità `USBLLONG` del modem Evologics (il codice è commentato in `MAIN_jetson.py:188-201`).

Invece, riceve le coordinate già elaborate dal sistema di superficie tramite **messaggi acustici RECVIM**:

```python
# Da MAIN_jetson.py:223-230
elif wgs84[0]=='2':  # Codice '2' = coordinate ENU
    FIRST_POS_Scouter = True
    xx = float(wgs84[1])      # X in ENU (Est)
    yy = float(wgs84[2])      # Y in ENU (Nord)
    pos_queue.put([xx, yy])   # Solo X e Y!
```

### Architettura del Sistema di Posizionamento

```
┌─────────────────────────────────────────────────────────────────┐
│              PC SUPERFICIE (Sw_withGPSlocalization/MAIN_USBL.py) │
│                                                                  │
│  ┌─────────────┐                                                │
│  │ Modem USBL  │──► USBLLONG ──► x,y,z (body frame modem)       │
│  │ (master)    │         │       campi[4,5,6]                   │
│  └─────────────┘         │       + RSSI[14], Accuracy[16]       │
│                          ▼                                       │
│  ┌─────────────┐    ┌──────────────────────────────────┐        │
│  │ GPS (Base)  │───▶│ Rotazione body→ENU               │        │
│  │ lat,lon     │    │ P_enu = Rzyx(r,p,y) × P_body     │        │
│  └─────────────┘    └──────────────┬───────────────────┘        │
│                                    │                             │
│                                    ▼                             │
│                          ┌─────────────────┐                    │
│                          │ SENDIM "2;X;Y"  │ (solo X e Y!)      │
│                          └────────┬────────┘                    │
└───────────────────────────────────│─────────────────────────────┘
                                    │ Trasmissione acustica
                                    ▼
┌─────────────────────────────────────────────────────────────────┐
│              JETSON UNDERWATER (HE_DPV-DS_USBL/MAIN_jetson.py)   │
│                                                                  │
│  ┌─────────────┐    ┌─────────────┐    ┌────────────────┐       │
│  │ Modem USBL  │───▶│ RECVIM      │───▶│ pos_queue      │       │
│  │ (slave)     │    │ "2;X;Y"     │    │ [X, Y]         │       │
│  └─────────────┘    └─────────────┘    └───────┬────────┘       │
│                                                 │                │
│  ┌─────────────┐                               │                │
│  │ MS5837      │───► depth ────────────────────┤                │
│  │ Profondim.  │                               ▼                │
│  └─────────────┘                    ┌────────────────────┐      │
│                                     │   Filtro Kalman    │      │
│  ┌─────────────┐                    │   X,Y da USBL      │      │
│  │ Xsens IMU   │───► acc,gyro ─────▶│   Z da depth       │      │
│  │ MTI-670     │                    │   + integrazione   │      │
│  └─────────────┘                    └────────────────────┘      │
└─────────────────────────────────────────────────────────────────┘
```

### Formato Messaggio USBLLONG (Evologics)

Il PC di superficie riceve dal modem USBL messaggi in formato `USBLLONG`:

```
USBLLONG,<timestamp>,<meas_time>,<remote_addr>,<X>,<Y>,<Z>,<E>,<N>,<U>,
         <roll>,<pitch>,<yaw>,<prop_time>,<rssi>,<integrity>,<accuracy>
```

| Campo | Indice | Descrizione |
|-------|--------|-------------|
| X | 4 | Posizione X nel body frame del modem (m) |
| Y | 5 | Posizione Y nel body frame del modem (m) |
| Z | 6 | Posizione Z nel body frame del modem (m) |
| RSSI | 14 | Received Signal Strength Indicator |
| Accuracy | 16 | Precisione stimata della misura (m) |

**Nota**: I campi E, N, U (7,8,9) sono le coordinate ENU calcolate internamente dal modem, ma il codice usa X, Y, Z (body frame) e applica una propria rotazione.

### Implicazioni

1. **USBL fornisce solo X e Y** - La coordinata Z viene **sempre** dal profondimetro
2. **Coordinate relative** - X e Y sono in metri, relative alla posizione della base GPS
3. **Sparse updates** - Quando `USBL_X = 0` e `USBL_Y = 0`, significa che non c'era un nuovo fix USBL in quell'istante
4. **Frame ENU** - East-North-Up, con origine sulla base di superficie

---

## Dettaglio Sensori

### Xsens MTI-670 (AHRS)

Driver: `lib/xsens.py`

| Output | Byte Offset | Formato | Note |
|--------|-------------|---------|------|
| Euler (Roll, Pitch, Yaw) | 19-30 | 3×float32 LE | Gradi |
| Acceleration | 34-45 | 3×float32 LE | m/s², body frame, include gravità |
| Free Acceleration | 49-60 | 3×float32 LE | m/s², senza gravità (non loggata) |
| Rate of Turn (Gyro) | 64-75 | 3×float32 LE | rad/s |
| Magnetic Field | 79-90 | 3×float32 LE | Normalizzato o Gauss |

Comunicazione: Seriale 460800 baud, pacchetti ~100 byte.

### MS5837-30BA (Profondimetro)

Driver: `lib/ms5837.py`

| Parametro | Valore |
|-----------|--------|
| Interfaccia | I2C (bus 1, addr 0x76) |
| Modello | 30BA (0-30 bar) |
| Densità fluido | 1029 kg/m³ (acqua salata) |
| Calcolo profondità | `(P_mbar×100 - 101300) / (ρ × 9.80665)` |
| Risoluzione temperatura | 0.01 °C |

---

## Note sui Dati

### Righe con Dati Parziali

La prima riga di ogni log ha solo 17 colonne (manca Gyro e Mag) perché viene scritta prima del loop principale:

```python
# Inizializzazione (MAIN_jetson.py:376-383)
f_kalman.write('{0} {1} {2} {3} '.format(time.time(), roll, pitch, yaw))
f_kalman.write('{0} {1} {2} '.format(0, 0, 0))  # Acc = 0
f_kalman.write('{0} {1} {2}'.format(posi[0], posi[1], sensor.depth()))
f_kalman.write('{0} {1} {2} {3} '.format(posi[0], posi[1], sensor.depth(), sensor.temperature()))
f_kalman.write('{0} {1} {2} '.format(0, 0, 0))  # State 6,7,8 = 0
# Mancano Gyro e Mag!
```

### Interpretazione USBL_X = USBL_Y = 0

Quando entrambi sono zero, può significare:
1. Nessun nuovo fix USBL disponibile (caso più comune)
2. Il veicolo è esattamente sull'origine (improbabile)

Per distinguere, controllare se `Est_X` e `Est_Y` cambiano: se il filtro sta integrando l'IMU, la posizione stimata evolve anche senza fix USBL.

### Frequenza di Campionamento

- **Log rate**: ~5 Hz (sleep 0.2s nel loop)
- **USBL update rate**: ~0.1-0.5 Hz (dipende dalla comunicazione acustica)
- **IMU internal rate**: Molto più alto, ma downsampled dal driver

---

## Sistemi di Riferimento

### Body Frame (Veicolo)
- X: Avanti (prua)
- Y: Destra (tribordo)
- Z: Basso (chiglia)

### ENU Frame (Navigazione)
- E (Est): Positivo verso Est
- N (Nord): Positivo verso Nord
- U (Up): Positivo verso l'alto (Z negativo = profondità)

### Trasformazione Body → ENU
```
v_ENU = Rxyz(roll, pitch, yaw) × v_body
```

Dove `Rxyz = Rx(roll) × Ry(pitch) × Rz(yaw)` (rotazioni intrinseche XYZ).
