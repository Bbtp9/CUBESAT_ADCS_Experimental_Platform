# MATLAB & Simulink Workspace Reference

Acest folder conține toate scripturile MATLAB și modelele Simulink organizate pe categorii clare, gata pentru simulări ideale sau teste în timp real Hardware-in-the-Loop (HIL) cu CubeSat-ul.

---

## Structura Directoarelor (Directory Structure)

### 📈 [1_Pure_Simulation/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab_simulink/1_Pure_Simulation/)
*Simulări ideale fără interfață hardware (control PD și LQR).*
* **`Cubesat_Control_PD.slx`**: Modelul Simulink utilizând arhitectura de control PD.
* **`Cubesat_Control_LQR.slx`**: Modelul Simulink utilizând arhitectura de control Optimal LQR.
* **`init_simulation_pd.m`**: Script MATLAB care inițializează parametrii fizici ai CubeSat-ului și calculează coeficienții (gains) pentru simularea PD.
* **`init_simulation_lqr.m`**: Script MATLAB care inițializează parametrii și calculează matricele de control LQR (prin rezolvarea ecuației Riccati).
* **`plot_cubesat_results.m`**: Script utilitar pentru reprezentarea grafică a rezultatelor simulărilor.
* **`Motor_Satellite_Dynamics.slx`**: Submodelul Simulink ce definește dinamica actuatorului (reaction wheel) și a satelitului.

### 🧲 [2_Magnetometer_Calibration/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab_simulink/2_Magnetometer_Calibration/)
*Calibrarea Hard-Iron a senzorului magnetic.*
* **`magnetometer_calibration.m`**: Scriptul utilizat pentru calcularea offset-urilor de calibrare prin centrarea cercului în $(0,0)$.
* **`mag1.txt`**: Setul de date brute citite de la magnetometru în timpul unei rotații complete de 360 de grade.
* **`README.md`**: Ghid explicativ detaliat privind modelul matematic de calibrare Hard-Iron și transpunerea lui în codul C++ Arduino.

### 📶 [3_Real_Time_HIL/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab_simulink/3_Real_Time_HIL/)
*Sistemul final de control Hardware-in-the-Loop în timp real prin Bluetooth BLE.*
* **`MATLAB_to_BLE_DETUMBL.m`**: Scriptul final de control în timp real pentru etapa de amortizare a vitezei unghiulare (Detumbling / Rate Damping). Scrie comenzile de cuplu motor și citește datele senzorilor prin BLE la ~10 Hz.
* **`MATLAB_to_BLE_POINTING.m`**: Scriptul final de control în timp real pentru etapa de orientare precisă (Pointing Control).
* **`PID1.slx` & `PID2.slx`**: Modelele Simulink HIL cu blocuri PID configurate.
* **`compare_detumble.m`**: Analizează și compară grafic rezultatele obținute în diferite rulări ale experimentului de Detumbling (salvate în `Detumble_Archive/`).
* **`compare_pointing.m`**: Analizează și compară grafic rezultatele experimentelor de Pointing (salvate în `Pointing_Archive/`).
* **`compare_sim_real.m`**: Script pentru validarea modelelor, comparând răspunsul sistemului din simulările teoretice cu datele reale achiziționate din HIL.
* **`Test_CubeSat_BLE_HIL.m`**: Test wireless simplu de conectare și recepție a telemetriei (11 parametri).
* **`BLEHILSystemObject.m`**: Custom System Object MATLAB folosit în Simulink pentru comunicarea prin Bluetooth.
* **`SerialHILSystemObject.m`**: Custom System Object pentru comunicarea serială cablată.

### 🗄️ [Archive_Old_Trials/](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab_simulink/Archive_Old_Trials/)
*Arhiva tuturor încercărilor anterioare, a fișierelor de test și a codurilor random, sortate pe categorii:*
* **`Root_Detumbling/`**: Coduri inițiale de detumble mutate din rădăcina proiectului.
* **`Root_Pointing/`**: Coduri inițiale de pointing mutate din rădăcina proiectului.
* **`Old_Scripts/`**: Colecția de scripturi vechi de testare și integrare.
* **`Other_Random/`**: Fișiere temporare, cache de compilare și grafice reziduale scoase din calea de lucru.

---

## Mod de Utilizare (How to Run)
1. **Pentru Simulări**: Rulează `init_simulation_pd.m` sau `init_simulation_lqr.m`, apoi deschide și pornește modelul Simulink corespunzător din `1_Pure_Simulation/`.
2. **Pentru Rulări în Timp Real HIL**: Rulează direct scriptul `MATLAB_to_BLE_DETUMBL.m` sau `MATLAB_to_BLE_POINTING.m` din `3_Real_Time_HIL/` pentru conectarea automată la satelit prin BLE.
