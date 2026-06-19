# Calibrarea Magnetometrului (Magnetometer Calibration)

Acest folder conține scriptul MATLAB și setul de date utilizat pentru calibrarea magnetometrului QMC5883P utilizat pe CubeSat.

## Fișiere (Files)
- [magnetometer_calibration.m](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab_simulink/2_Magnetometer_Calibration/magnetometer_calibration.m): Scriptul de calibrare care citește datele brute, calculează mediile și trasează graficul datelor brute vs. calibrate.
- [mag1.txt](file:///Users/bbtp/Desktop/THESIS/CUBESAT_THESI_VS/software/matlab_simulink/2_Magnetometer_Calibration/mag1.txt): Datele brute achiziționate de la magnetometru în timpul unei rotații complete de 360 de grade în planul orizontal.

---

## Metodologie de Calibrare (Hard-Iron Calibration)

Distorsiunea de tip **Hard-Iron** este cauzată de magneți permanenți sau componente magnetice de pe structura CubeSat-ului. Aceasta adaugă un offset (polarizare) constant la măsurătorile magnetometrului, deplasând centrul cercului de măsură în planul 2D (X-Y) departe de originea $(0,0)$.

### 1. Calculul Offset-urilor în MATLAB
Scriptul citește coloanele de date brute $M_x$, $M_y$ din `mag1.txt` și le calculează media aritmetică:
- $X_{\text{offset}} = \text{mean}(M_x) \approx 163$
- $Y_{\text{offset}} = \text{mean}(M_y) \approx -57$

Pentru a centra norul de puncte în originea $(0,0)$, scădem aceste offset-uri din măsurătorile brute:
$$M_{x0} = M_x - X_{\text{offset}}$$
$$M_{y0} = M_y - Y_{\text{offset}}$$

### 2. Implementarea în Codul Arduino (ESP32)
Aceste offset-uri calculate în MATLAB sunt introduse direct în codul din `CubeSat_HIL_BLE.ino` pentru corecția în timp real a datelor înainte de trimiterea prin Bluetooth (BLE) către MATLAB/Simulink:

```cpp
// Corecție Hard-Iron bazată pe calibrare:
// mx_calibrated = mx - 163
// my_calibrated = my - (-57) = my + 57
heading = atan2((float)my + 57, (float)mx - 163) * 180.0 / PI;

if (heading < 0) {
  heading += 360.0;
}
```

Această corecție asigură că unghiul de orientare (yaw/heading) calculat este precis și centrat corect.
