clc; clear all; close all;

set(0,'DefaultLegendInterpreter','tex');   % For TeX in legends 

%% PV array: Canadian Solar CS6X-280P
No_Cadenas_Paralelo = 2;     % Number of parallel strings
No_Modulos_PorCadena = 18;   % Modules per string (series)
Irr_in = 0;                  % Irradiance input (placeholder)

%% DC-DC boost converter (component selection & operating point)
% T = 25°C, Pmp = 10.07 kW, Vmp = 640.8 V
vo  = 640.8;       % Minimum input voltage
vdc = 1000;        % Desired DC bus voltage
Fs  = 5e3;         % Switching frequency
Po  = 10.07e3;     % Input power
io  = Po/vo;       % Input current
iL  = io;          % Inductor nominal current
d   = (vdc-vo)/vdc; 
Ro  = vo/io;       % MPPT equivalent resistance
Iload = io*(1-d);                % Load current
Rload = vdc/(Iload);             % Load resistance
Lmin  =(d*(1-d)^2*Rload)/(2*Fs); % Boundary inductor
L     = 68.2729*Lmin;            % Selected inductor (printed on purpose)
ILmax = vo/((1-d)^2*Rload)+(vo*d)/(2*L*Fs);
ILmin = vo/((1-d)^2*Rload)-(vo*d)/(2*L*Fs);
Cload = 470e-6;                  % Output capacitor
dVdc  = (d*vdc)/(Rload*Cload*Fs);% Output ripple estimate (~1 V target)
Co    = 1e-6;                    % Input capacitor (note: 100 uF gives max efficiency)
b     = 16;                      % DPWM bit depth
Dmax  = (2^b-1);
Kdpwm = 1/Dmax;                  % DPWM gain

% Continuous-time small-signal model (states X=[iL; vdc], input U=d).
% Assumes Co << Cload and variable vdc.
A=[0 (d-1)/L;
   1*(1-d)/Cload -1/(Rload*Cload)];     % State matrix
B=[vdc/L
   -iL/(Cload)];                        % Input matrix (vo as disturbance)
B=B*Kdpwm;                              % Include DPWM gain
C=[1 0];                                % Output: control iL
D=[0];                                  % Direct term
X = {'iL' 'vdc'};                       % x1 = iL, x2 = vdc
U = {'d'};                              % Duty cycle input
Y = {'iL'};                             % Output is input current iL
sys_cont = ss(A,B,C,D,'statename',X,'inputname',U,'outputname',Y);
tf(sys_cont);
GsiLd=ans(1,1);                         % TF iL(s)/d(s)
figure(1)
step(((1-d)*Dmax)/2*GsiLd);             % Open-loop (continuous TF)
hold on;

%% Digital PI (based on boost switching frequency)
Kc=4155.3*114.7;    % Proven with ts=0.25 s (2%) 
a =1/114.7;         % PI time constant inverse
num=[a 1];
den=[1 0];
Cs_PI = zpk(Kc*tf(num,den));               % Continuous PI
Ti=a;
Kp1=Ti*Kc;
Kh1=1/io;                                  % Current sensor gain
num=[1];
den=[1];
H_s = Kh1*tf(num,den);                     % Sensor TF
Ts1=1/Fs;                                  % Sampling time
GziLd = c2d(GsiLd,Ts1,'ZOH');              % Discrete plant
step(((1-d)*Dmax)/2*GziLd);                % Open-loop (discrete TF)
hold off;
KI1=(Kp1*Ts1)/Ti;                          % Digital I gain
KP1=Kp1-KI1/2;                             % Digital P gain
num=[(KP1+KI1) -KP1];
den=[1 -1];
Cz_PI=zpk(tf(num,den,Ts1));                % Discrete PI
H_z=c2d(H_s,Ts1,'ZOH');                    % Discrete sensor
TFcls=feedback(Cs_PI*GsiLd,H_s);           % Closed-loop (continuous)
TFclz=feedback(Cz_PI*GziLd,H_z);           % Closed-loop (discrete)
figure(2)
step(TFcls,'r');                           % Closed-loop in "s"
hold on;       
step(TFclz,'--b');                         % Closed-loop in "z"
hold off;

%% Mamdani fuzzy system (placeholder / alignment with original code)
Ts2 = Ts1;

%% Tsukamoto CoDif controller for the boost converter (same params, improved labels)
paso = 0.001;
Univ_discurso=0.015;                         
x=-10:paso:10;                               % Universe of discourse for error e
y=-Univ_discurso:paso:Univ_discurso;         % Universe of discourse for output
a= 1;                                        % Slope for smooth control curve
Ts3=Ts1;

%% Membership functions for fuzzy set A
mA1 = sigmf(x, [-a 0]); % N (Negative)
mA2 = sigmf(x, [a 0]);  % P (Positive)
figure;
% Plot mA1 and mA2 — panel (a)
subplot(1, 3, 1);
plot(x, mA1, 'LineWidth', 1.5, 'Color', [0 0.44 0.74]);      % Color #0071BC
hold on;
plot(x, mA2, 'LineWidth', 1.5, 'Color', [0.847 0.322 0.094]); % Color #D85218
hold off;
set(gca, 'FontSize', 15, 'FontName', 'Times New Roman');
legend('\it{A}\rm_{1}: Negative (N)', '\it{A}\rm_{2}: Positive (P)', 'Location', 'NorthEast');
xlabel('\it{e}', 'FontName', 'Times New Roman');
ylabel('\mu \it{A_i}(\it{e}\rm)', 'FontName', 'Times New Roman');
axis([-7 7 0 1.4]);
grid off;
title('(a) Membership functions (MFs) for the input error \it{e}', 'FontSize', 15, 'FontWeight', 'normal');
% Membership functions for fuzzy set B
mB1 = trimf(y, [-Univ_discurso  Univ_discurso  Univ_discurso]);  % P (Positive)
mB2 = trimf(y, [-Univ_discurso -Univ_discurso Univ_discurso]);   % N (Negative)
% Plot mB1 and mB2 — panel (b)
subplot(1, 3, 2);
plot(y, mB1, 'LineWidth', 1.5, 'Color', [0.847 0.322 0.094]); % Color #D85218
hold on;
plot(y, mB2, 'LineWidth', 1.5, 'Color', [0 0.44 0.74]);       % Color #0071BC
hold off;
set(gca, 'FontSize', 15, 'FontName', 'Times New Roman');
legend('\it{B}\rm_{1}: Positive (P)', '\it{B}\rm_{2}: Negative (N)', 'Location', 'NorthEast');
xlabel('\Delta \it{d}', 'FontName', 'Times New Roman');
ylabel('\mu \it{B_i}(\Delta \it{d}\rm)', 'FontName', 'Times New Roman');
axis([-Univ_discurso Univ_discurso 0 1.3]);
grid off;
title('(b) MFs for variations in the duty cycle \Delta\it{d}\rm', 'FontSize', 15, 'FontWeight', 'normal');
% Inference method and control curve (Tsukamoto)
imax = length(x);
for i = 1:imax
    w  = [mA1(i), mA2(i)];                    % Weights (single-input: min not required)
    z1 = -2 * Univ_discurso * (mA1(i) - 0.5); % B2 inverse mapping (monotone)
    z2 =  2 * Univ_discurso * (mA2(i) - 0.5); % B1 inverse mapping (monotone)
    y(i) = (w(1) * z1 + w(2) * z2) / (w(1) + w(2)); % Weighted average (Tsukamoto)
end
% Plot (c): Control surface (output curve)
subplot(1, 3, 3);
plot(x, y, 'LineWidth', 1.5);
set(gca, 'FontSize', 15, 'FontName', 'Times New Roman');
xlabel('\it{e}', 'FontName', 'Times New Roman');
ylabel('\Delta \it{d}', 'FontName', 'Times New Roman');
axis([-7 7 -0.018 0.018]);
grid off;
title('(c) Control surface', 'FontSize', 15, 'FontWeight', 'normal');

%% Tsukamoto inference example
% Input error sample (choose a value within the x range)
x0 = -1.0;

% Find the closest index to x0 within the universe of discourse
[~, n] = min(abs(x - x0)); % Index of the sample closest to x0

% Rule outputs at x0
w  = [mA1(n), mA2(n)];
z1 = -2 * Univ_discurso * (mA1(n) - 0.5); % z1 from A1 -> B2
z2 =  2 * Univ_discurso * (mA2(n) - 0.5); % z2 from A2 -> B1

% Tsukamoto aggregation (monotone consequents, weighted by firing strengths)
z0 = (w(1) * z1 + w(2) * z2) / (w(1) + w(2));

% Display result
fprintf('For x_0 = %.2f, the output is y_0 = %.4f\n', x0, z0);

%% State-feedback control (discrete)

% 1) Discrete state-space model
[Ad, Bd, Cd, Dd] = c2dm(A, B, C, D, Ts1,'ZOH');
X = {'iL' 'vdc'};    % x = [iL; vdc]
U = {'d'};           % Input: duty d (vo as disturbance)
Y = {'iL'};          % Output: iL
sys_disc = ss(Ad,Bd,Cd,Dd,Ts1,'statename',X,'inputname',U,'outputname',Y);
tf(sys_disc);
GziLd=ans(1,1) ;             % TF iL(z)/d(z)
figure(4);
step(((1-d)*Dmax)/2*GziLd);  % Open-loop (discrete TF)
hold on;
poly(eig(sys_disc));         % Characteristic polynomial (open-loop, discrete)        
roots([ans]);                % Open-loop poles (discrete)

% 2) Augmented system for integral action
Aa = [Ad Bd;zeros(1,length(Ad)) 0];
Ba = [zeros(length(Bd),1);1];
Ca = [Cd 0];
Da = [Dd 0];
sys_a =ss(Aa,Ba,Ca,0,Ts1);
step((1-d)*Dmax/2*tf(sys_a)); % Open-loop (discrete state-space, augmented)
hold off;

% 3) Controllability
Coo = ctrb(sys_a);
rank(Coo); % Should equal size(Aa)

% 4) Open-loop characteristic equation of augmented system
E_Ca=poly(eig(sys_a));
roots(E_Ca);

% 5) Desired poles (discrete) — chosen set
Ed = [0.975;0.99295;0.99];
Ps = poly(Ed);

% 6) State-feedback gains
k = acker(Aa,Ba,Ed);

% 7) Controller gains K = [K1 K2]
Km = [Ad-eye(length(Ad)) Bd;
      Cd*Ad              Cd*Bd];
In = [zeros(1,length(k)-1) 1];
K = (k+In)/Km;
K1 = K(1:end-1);
K2 = K(end);

% 8) Closed-loop system (with integral action)
Af= [Ad-Bd*K1         Bd*K2;
     -Cd*Ad+Cd*Bd*K1  1-Cd*Bd*K2];
Bf = [zeros(length(Bd),1);1];
Cf = Ca;
Cf = [Ca;zeros(0,length(Ca))]; 
slc= ss(Af,Bf,Cf,0,Ts1);

% 9) Closed-loop eigenvalues (should match desired)
eig(Af);

% 10) Closed-loop step response
figure(5)
step(io*slc);
title('Closed Loop (Step)');    

%% DC-AC inverter (base design values)
Fc=2e3;               % Carrier triangular frequency (Hz)
fs=60;                % Grid frequency (Hz) 
ws=2*pi*fs;           % Grid angular frequency (rad/s)
vdc=1000;             % DC bus nominal voltage (V)
S=10e3*3/2;           % Apparent power base (VA)
VL=(381.051/2);       % Line voltage base (rms)
Vf=VL/sqrt(3);        % Phase voltage base (rms) 
vdc_ref=vdc;          % Desired DC bus voltage (V)
Lbase=vdc^2/(ws*S);   % Base L (H)

% Final L selection (rule of thumb: Fc = 2 kHz and THD < 10%)
L_pu=0.3;
Linv=L_pu*Lbase;
Linv=0.0133;
Rinv=(Linv*377/fs/2);  % Internal resistance estimate
Rinv=0.0417;
Cinv=470e-6;           % Inverter input capacitor
Ts4=1/(4*Fc);          % Sampling time (s)
Td=Ts4/2;              % PWM update delay (s)
Ts_inv=1/(100*Fc);     % PWM & non-controller blocks sampling
Vp=1;                  % PWM carrier peak voltage (V)

%% Digital PI (dq currents) neglecting PWM update delay
% Current loop (d-q)
Porcentaje1=1;                      % Max overshoot (%)
Mp1=Porcentaje1/100;
Zeta1=sqrt(log(Mp1)^2/(log(Mp1)^2+pi^2)); % Damping factor
tset1=10e-3;                         % Settling time (s)                                  
Porcentaje=2;                        % Allowed ss error (%)
E1=Porcentaje/100;
wn1=-log(E1)/(Zeta1*tset1);          % Natural frequency (rad/s)
Kp2=2*Linv*Vp*Zeta1*wn1-Rinv*Vp;     % Proportional gain
Ki2=Linv*Vp*wn1^2;                   % Integral gain

% Voltage loop without id filter
Porcentaje=1;                        % Max overshoot (%)
Mp2=Porcentaje/100;                  % Damping factor
Zeta2=sqrt(log(Mp2)^2/(log(Mp2)^2+pi^2));
n=2;                                 % Voltage loop n times slower than current loop   
tset2=tset1*n;                       % Settling time (s)
Porcentaje=4;                        % Allowed error (%)
E2=Porcentaje/100;
wn2=-log(E2)/(Zeta2*tset2);          % Natural frequency (rad/s)
Kp4=2*Cinv*Zeta2*wn2;                % Proportional
Ki4=Cinv*wn2^2;                      % Integral

% PLL controller
Zeta3=0.707;                         % Damping
wn3=(120*pi)/2;                      % Grid angular frequency reference
Kp5=(2*Zeta3*wn3);                   % Proportional
Ki5=wn3^2;                           % Integral

% Continuous-to-discrete equivalent controller constants
KI2=Ki2*Ts4;
KP2=Kp2-KI2/2;
KI4=Ki4*Ts4;
KP4=Kp4-KI4/2;
KI5=Ki5*Ts_inv;
KP5=Kp5*Ts_inv-KI5/2;

%% Transformer ~50 kVA (small building equivalent)
Ptrafo   = 100e3;           % Rated power
Ftrafo   = fs;              % Operating frequency 
VLLsec   = 33000;           % Grid side line-to-line (Vrms)
VLLprim  = VL;              % Inverter side line-to-line (Vrms)

%% Electrical grid (basic references)
VLngrid = 33000/sqrt(3);    % Line-to-neutral (Vrms)
Fred    = fs;               % Grid frequency (Hz)
In = 1e6/(1.73*(Vf));       % Example nominal current
Icc_calc = In/(5/100);      % ~5% short-circuit estimate

%% ======================
% Figure annotations 
% ======================

% Global look for figures (fonts only; data untouched)
set(groot,'defaultAxesFontName','Times New Roman');
set(groot,'defaultTextFontName','Times New Roman');
set(groot,'defaultAxesFontSize',12);
set(groot,'defaultTextFontSize',12);

% --- Fig. 1: Open-loop (continuous vs discrete) using TFs
figure(1); grid on; box on;
title('Fig. 1. Open-loop response (continuous & discrete): i_{in} vs duty d using transfer functions. Note: i_L = i_{in}','FontWeight','normal');
xlabel('Time (s)'); ylabel('Input current i_{in} (A)');
legend('Continuous TF (s-domain)','Discrete TF (z-domain)','Location','southeast');

% --- Fig. 2: Closed-loop with PI (continuous vs discrete)
figure(2); grid on; box on;
title('Fig. 2. Closed-loop response with PI: i_{in} (continuous vs discrete). Note: i_L = i_{in}','FontWeight','normal');
xlabel('Time (s)'); ylabel('Input current i_{in} (A)');
legend('Continuous PI (s-domain)','Discrete PI (z-domain)','Location','southeast');

% --- Fig. 3: Already plotted above — labels/titles were set to English to match your example.

% --- Fig. 4: Open-loop (z-domain): TF vs State-Space
figure(4); grid on; box on;
title('Fig. 4. Open-loop (z-domain): i_{in} vs duty d — transfer function vs state-space. Note: i_L = i_{in}','FontWeight','normal');
xlabel('Time (s)'); ylabel('Input current i_{in} (A)');
legend('Discrete TF (z-domain)','Discrete State-Space (z-domain)','Location','southeast');

% --- Fig. 5: Closed-loop with discrete state-feedback + integral action
figure(5); grid on; box on;
title('Fig. 5. Closed-loop: i_{in} with discrete state-feedback + integral action. Note: i_L = i_{in}','FontWeight','normal');
xlabel('Time (s)'); ylabel('Input current i_{in} (A)');
legend('State-feedback w/ integral (discrete)','Location','southeast');
