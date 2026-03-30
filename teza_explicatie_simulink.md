# Capitol/Secțiune Sugerată: "Implementarea și Simularea Sistemului de Control al Atitudinii"

## 1. Descrierea Generală a Sistemului
Pentru validarea design-ului de control al atitudinii CubeSat-ului, a fost dezvoltat un model complet de simulare folosind mediul **MATLAB & Simulink**. Sistemul combină două faze esențiale misiunii unui satelit miniatural:
- **Detumbling:** Faza de frânare și anulare a rotației libere scapate de sub control (apărute după lansarea din dispenser).
- **Pointing:** Faza de orientare precisă către un unghi țintă (pentru transmitere date, orientare panouri solare sau camere).

Fluxul de simulare este controlat de două scripturi MATLAB principale (`init_simulation.m` și `plot_cubesat_results.m`) ce setează automat parametrii fizici, calculează și rulează modelul `Cubesat_Control_System.slx`, și finisează datele pentru interpretare vizuală.

---

## 2. Modelul Simulink (`Cubesat_Control_System.slx`)

Modelul Simulink a fost structurat în două subsisteme majore: **Attitude_Controller** (creierul) și **Cubesat_Dynamics** (fizica mișcării).

### 2.1 Subsistemul Dinamic (`Cubesat_Dynamics`)
Acest bloc simulează ecuațiile mișcării rotaționale folosind trei integratoare înlănțuite, transpunând cuplul/momentul motorului în cinetica satelitului.
Pe baza principiului acțiunii și reacțiunii, orice cuplu aplicat de motor asupra momentului său de inerție intern ($10^{-5}\ kg\cdot m^2$) generează o reacțiune de cuplu invers pe șasiul satelitului ($0.002\ kg\cdot m^2$).

1. **Viteza unghiulară a Rotorului ($\omega_{rotor}$):** 
   Se obține prin integrarea accelerației rotorului.
   $$ \dot{\omega}_{rotor} = \frac{M}{J_{rotor}} \quad \xrightarrow{\text{integrator}} \quad \omega_{rotor} $$
2. **Viteza unghiulară a Satelitului ($\omega_{s}$):**
   Se obține prin integrarea accelerației satelitului, cauzată de cuplul reacționar ($-M$).
   $$ \dot{\omega}_{s} = \frac{-M}{J_{satelit}} \quad \xrightarrow{\text{integrator}} \quad \omega_{s} $$
3. **Unghiul de Atitudine a Satelitului ($\theta$ sau $P_s$):**
   Se obține prin integrarea vitezei unghiulare a satelitului.
   $$ \dot{\theta} = \omega_{s} \quad \xrightarrow{\text{integrator}} \quad \theta $$

### 2.2 Subsistemul de Control (`Attitude_Controller`)
Acest bloc decide ce valoare de moment ($\tau$) trebuie cerută la motor pe baza erorii curente.

- **Legea pentru Detumbling (Frânare Pura):**  
  Când satelitul se învârte necontrolat, controllerul folosește un termen exclusiv derivativ (fără țintă unghiulară) pentru a tinde să aducă derivata (viteza) fix la zero.
  $$ \tau_{detumble} = - K_{d\_detumble} \cdot \omega_{s} $$
  *(În cod: $K_{d\_detumble} = 0.03$)*

- **Legea pentru Pointing (Control PD):**  
  Odată oprit, satelitul trebuie orientat spre unghiul de referință dorit ($\theta_{ref}$). Legea este de de tip Proporțional-Derivativ.
  $$ \tau_{pointing} = K_{p} \cdot e - K_{d} \cdot \omega_{s} $$
  *(Eroarea: $e = \theta_{ref} - \theta$; În cod: $K_p = 0.02, K_{d} = 0.01$)*

- **Logica de Comutare Unidirecțională (One-Way Latch & Switch):**  
  Pentru a preveni ca "Controllerul de Detumbling" să intervină eronat în timp ce PD-ul de Pointing rotește satelitul spre țintă, logica folosește un bloc **Relay** ce declanșează un Switch. 
  Condiția este ca trecerea de la Detumbling la Pointing să se facă doar atunci când modulul vitezei unghiulare scade sub pragul de toleranță $|\omega| < 0.3 \text{ deg/s}$. Pentru a garanta că decizia este definitivă pentru acea manevră (o singură direcție: Detumbling $\rightarrow$ Pointing), "Upper Threshold-ul" care l-ar face să se întoarcă la Detumbling a fost setat artificial la infinit ($10000 \text{ rad/s}$).

- **Saturarea (Clipping):** 
  Orice comandă de cuplu ($\tau_{req}$) calculată de algoritm este trecută printr-un limitator de tip Saturație, configurat la capacitatea fizică maximă a motorului pe Reaction Wheel utilizat: $[-0.002\text{ Nm}, +0.002\text{ Nm}]$.  
  $$ \tau = \text{sat}(\tau_{req}, -\tau_{max}, \tau_{max}) $$
  În final, cerința de cuplu pe corpul satelitului este inversată ($\times -1$) pentru a deveni comanda de cuplu aplicată motorului, așa cum e descris de dinamica reacțiunii amintite la capitolul precedent.

---

## 3. Prezentarea Scripturilor MATLAB

### 3.1 Scriptul de Initializare (`init_simulation.m`)
Este "panoul de control" al sistemului, facilitând simulări interactive, repetabile din Consolă, cu următorii pași:
1. Afișează un prompt și preia din consolă viteza unghiulară inițială $\omega_0$ și ținta de unghi de referință dorită (în grade).
2. Convertește variabilele de la utilizator din grade în radiani. Încărcă toți parametrii modelului ($J_{sat}$, $J_{rotor}$, Gain-urile controlerului PD și ale B-Dot / Detumble, saturația).
3. Lansează funcția `sim` apelând direct blocul `Cubesat_Control_System.slx`.
4. Extrage variabilele de stare la finalizarea simulării și apelează la scriptul de plotare.

### 3.2 Scriptul de Plotare (`plot_cubesat_results.m`)
Extrage baza temporală (`t_out`) și toți vectorii de performanță (theta, omega, comanda de moment a roții). Identifică matematic secundele unde satelitul trece prin viteza pragului de Detumbling pentru a trage o linie verticală roșie ("Switch indicator") peste toate 4 grafice.  
Aplică un sistem strict de nuanțare (limitarea vizuală pe axa y a Cuplului Motorului doar peste plafonul maximal de $\pm 1.5 \times \tau_{max}$) astfel încât să se vadă perfect vizual intervalele saturate unde algoritmul "cere mai mult decât poate motorul oferi". 

---

## 4. Analiza de Caz - Scenariu: Turație de 20 deg/s și Pointing 180°

Graficele au fost generate evaluând capacitatea de reorientare (180°) pe profilul unei rotații aspre de început de tip rezidual ($20 \text{ deg/s}$). Sistemul ilustrează fidel performanța unui PD de capabilități mici pe intervale de timp medii (c. a. 300 s).

### Interpretarea Performanțelor Afișate pe Grafic:
1. **Faza de Detumbling ($t = 0 \rightarrow 4$ secunde):** 
   În plotul "Satellite Angular Velocity", se remarcă instant cum de la o eroare/perturbare inițială ridicată de 20 deg/s, cuplul "Reaction Wheel Motor Torque" cade masiv spre limitatorul său saturat la valoarea negativă $-0.002 \text{ Nm}$. Fiind presat mecanic la valoarea maximală, viteza scade rectiliniu sub sub $-1 \text{ rad/s}^2$ ritm de asimilare constant, atingând $0\text{ deg/s}$ conform așteptărilor sistemului.
   
2. **Accelerarea Fazei de Pointing ($t \approx 4 \rightarrow \approx 20$ secunde):**  
   Imediat după linia roșie punctată indicând stabilizarea, se activează controller-ul PD. Acesta simte că satelitul e la 0° iar ținta finală setată este chiar de partea opusă: $\theta_{ref} = 180°$. Cerând un efort imens, el saturează instant roata pe invers (îi dă $+0.002 \text{ Nm}$). Pe graficul albastru se vede clar cum satelitul începe s-o prindă forță exponențială accelerând la viteze de $\approx 150 \text{ deg/s}$, întru-cât a rămas blocat în accelerație completă preț de aproape zece secunde datorită "disperării" algoritmului PD de acoperi decalajul proporțional uriaș de 180°.

3. **Decelerarea, Fenomenul de Overshoot (Supracompensare) și Stabilizarea ($t = 20 \rightarrow 200$ secunde):**  
   Pe măsură ce unghiul satelitului se apropie victorios de ținta albă de 180°, contribuția valorii derivatorii ($K_d$) devine masivă dominând pe cea a proporționalului ce s-a micșorat o dată cu apropierea. 
   Astfel roata reaction-wheel-ului inversează polaritatea să frâneze, dând din ce în ce mai greu la capacitatea modestă de $ -0.002 \text{Nm} $. 
   Roata nefiind suficient de "grea", inerția de 150 deg/s acumulată propulsează satelitul DUPĂ ținta de 180°, marcând un fenomen pur tehnic numit "Overshoot" (se vede în plotul turcoaz al atitudinii cum atinge $250°$ un spike principal). El corectează adecvat trăgând atitudinea la dreapta spre $\approx 150$ și se amortizează încet cu un număr de $\sim 2$ perioade. Controlerul cu amortizarea sa teoretică de $\zeta \approx 0.79$ dovedește că își aduce satelitul fără discuții fix la $\theta = 180°$ vizibil după secunda $\approx 200$. Roata (graficul 4: Wheel Speed) rămânând echilibrat blocată la un surplus cinetic preluat de a stabiliza.
