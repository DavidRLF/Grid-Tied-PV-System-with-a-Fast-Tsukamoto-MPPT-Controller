clc; clear all; close all;

%% Boost-converter parameters and operating point
% PV array (T = 25 °C, Pmp = 20 W, Vmp = 7 V)
vpv = 7;
vdc = 12;        % Desired output voltage (dc-bus voltage)
Fs = 5e3;          % Switching frequency
Ppv = 20;
ipv = Ppv/vpv;     % Input current
iL = ipv;          % The inductor current is assumed equal to ipv
d =(vdc-vpv)/vdc;  % Duty cycle
Rpv = vpv/ipv;     % Rpv=Rmppt
Idc = ipv*(1-d);   % Load-current calculation
Rdc = vdc/(Idc);   % Load-resistance calculation
Lmin =(d*(1-d)^2*Rdc)/(2*Fs); % Minimum inductance for withtinuous-withduction mode (CCM)
%L = 68.2729*Lmin;  % To ensure CCM operation
L = 4.4e-3;
ILmax = vpv/((1-d)^2*Rdc)+(vpv*d)/(2*L*Fs); % Maximum inductor current
ILmin = vpv/((1-d)^2*Rdc)-(vpv*d)/(2*L*Fs);  % Minimum inductor current
Cdc = 3300e-6;               % Output capacitor 
dVdc = (d*vdc)/(Rdc*Cdc*Fs) % Output-voltage ripple of 1 V
Cpv = 100e-6;                % Input capacitor (Scenarios 2 and 5)
b = 16;                     % DPWM resolution in bits
Ucmax = (2^b-1);            % Maximum control signal (0-Ucmax)
Ucmax = 9000;               % Maximum control signal (0-Ucmax)
Dmax = Ucmax;
Kdpwm = 1/Ucmax;            % DPWM gain (digital PWM model)
UC = d/Kdpwm;               % Define the operating point for UC
X1 = iL;                    % Define the operating point for state X1 
X2 = vdc;                   % Define the operating point for state X2
Ref = ipv;                  % Controller reference
vdc_ref = vdc;          % Desired dc-bus voltage (V)

%% Continuous-time open-loop model
% x1 = ipv (PV-array current), x2 = vdc (output voltage)
% State-space matrices
Amc = [0 -(1-Kdpwm*UC)/L
      (1-Kdpwm*UC)/Cdc -1/(Cdc*Rdc)];
Bmc = [(Kdpwm*X2)/L
      -(Kdpwm*X1)/Cdc]; 
Cmc = [1/2 0     % To control x1 = ipv = iL
      0 1];      % To control x2 = vdc
Dmc = [0
      0];
states = {'x1' 'x2'};
inputs = {'uc'}; 
outputs = {'x1=ipv' 'x2=vdc'};
Sys_Boost_Modelo_Continuo = ss(Amc,Bmc,Cmc,Dmc,'statename',states,'inputname',inputs,'outputname',outputs)
tf(Sys_Boost_Modelo_Continuo)
Gs_uc_to_x1 = (ans(1,1))
Gs_uc_to_x2 = (ans(2,1))
[num_Gs_uc_to_x1, den_Gs_uc_to_x1] = tfdata(Gs_uc_to_x1, 'v');
[num_Gs_uc_to_x2, den_Gs_uc_to_x2] = tfdata(Gs_uc_to_x2, 'v');

%% Discrete-time open-loop model of the boost converter
% x1 = ipv (PV-array current), x2 = vdc (output voltage)
% State-space matrices
Ts1 = 1/Fs;
Amd = [1 (Ts1*(Kdpwm*UC-1))/L
   -(Ts1*(Kdpwm*UC-1))/Cdc 1-Ts1/(Cdc*Rdc)];
Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];
Cmd = [1/2 0      % To control x1 = ipv = iL
       0 1];      % To control x2 = vdc
Dmd = [0
      0];
states = {'x1' 'x2'};
inputs = {'uc'}; 
outputs = {'x1=ipv' 'x2=vdc'};
Sys_Boost_Modelo_Discrete = ss(Amd,Bmd,Cmd,Dmd,Ts1,'statename',states,'inputname',inputs,'outputname',outputs)
tf(Sys_Boost_Modelo_Discrete)
Gz_uc_to_x1=(ans(1,1));
[num_Gz_uc_to_x1, den_Gz_uc_to_x1] = tfdata(Gz_uc_to_x1, 'v');

%% Discrete state-feedback controller with integral action for the boost converter
% System
Cmd_Control = [1 0]; %Control of y = x1 = ipv
Dmd_Control = [0];
Amd = [1 -(Ts1*(1-Kdpwm*UC))/L
   (Ts1*(1-Kdpwm*UC))/Cdc 1-Ts1/(Cdc*Rdc)];
Bmd = [(Kdpwm*Ts1*X2)/L
    -(Kdpwm*Ts1*X1)/Cdc];
% Augmented matrices Aamd and Bamd
% Aamd for the Simulink block-model implementation 
% because the integral action is represented by a third state and is modeled 
% explicitly using an accumulated-sum block 
% with gain Ts in the form x3[k+1]=x3[k]+Ts(X2ref[k]-x2(k))
Aamd = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
        (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                    0;
        -Ts1                       0                                1 ]; 
% Aamd for the script environment, where the integral action is represented by
% a third state modeled in the form x3[k+1]=x3[k]+(X2ref[k]-x2(k)) 
% without Ts because it is already included in the ss(...) function                                             
Aamdprima = [ 1                        -(Ts1*(1-Kdpwm*UC))/L             0;
             (Ts1*(1-Kdpwm*UC))/Cdc   1-Ts1/(Cdc*Rdc)                  0;
             -1                       0                                1 ]; 
Bamd = [ (Kdpwm*Ts1*X2)/L;
         -(Kdpwm*Ts1*X1)/Cdc;
         0 ];
Eamd = [zeros(size(Aamd,1)-1,1); 1];
Camd = [Cmd_Control 0];
% Desired z-plane poles (withverted from withtinuous time)
%Testd = 0.06245;
Testd = 0.008;     % Scenarios 1-5
Testd = 0.007;
zeta_d = 1.0;
p3 = 52;
p3 = 85;
wn_d = 4 / (zeta_d * Testd);
s1 = -zeta_d * wn_d;
s2 = -zeta_d * wn_d;
s3 = -p3;
z1 = exp(s1 * Ts1);
z2 = exp(s2 * Ts1);
z3 = exp(s3 * Ts1);
Polos_Des_Boost_Mod_Disc = [z1 z2 z3];   % Following the design method
% For the trial-and-error-based method
% z1p = exp(s1p * Ts1); % For the trial-and-error-based method
% z2p = exp(s2p * Ts1); % For the trial-and-error-based method
% z3p = exp(s3p * Ts1); % For the trial-and-error-based method
% Polos_Des_Boost_Mod_Disc = [z1p z2p z3p]; % For the trial-and-error-based method
% Check controllability
Co_Boost_Modelo_Disc = ctrb(Aamd, Bamd);
if rank(Co_Boost_Modelo_Disc) < size(Aamd,1)
    error('The augmented system is not fully controllable.')
end
% Compute the gains with acker as used in Simulink
Kmd = acker(Aamd, Bamd, Polos_Des_Boost_Mod_Disc);  
K1y2md = Kmd(1:end-1) % For closed-loop use in Simulink
% Invert the sign of Ki to prevent an opposite integral action
% because the error is Vref-Vo; this would not be required for Vo-Vref
Kimd = -Kmd(end) % For closed-loop use in Simulink with a block model
                 % When using a function block in Simulink,
                 % Kimd must be multiplied by Ts because of the way
                 % the Aamd model and the integrator were formulated
                 % in the function block:
                 % Acc_Int = Ki * ek * Ts + Acc_Int_1;
% Compute the gains with acker as used in the script environment
Kmdprima = acker(Aamdprima, Bamd, Polos_Des_Boost_Mod_Disc); % As used in the script
% Determine the closed-loop system for script-based evaluation
Afmd = Aamdprima-Bamd*Kmdprima; 
% Compute the eigenvalues (denominator roots) of the closed-loop system
% using the Simulink Kmd gains; they must be 
% equal to the desired poles
Polos_Boost_Lazo_Cerrado_Disc = eig(Aamd-Bamd*Kmd)
% Closed-loop boost converter with state-space feedback
% using Kmdprima in the script environment
SLC_Boost_m_SF_Disc = ss(Afmd,Eamd,Camd,0,Ts1);
% Open- and closed-loop boost-converter responses with SF 
% and integral action
figure(1)
% Open loop (x1 versus u1)
subplot(2,1,1)
hold on
step((Ucmax-UC) * Gz_uc_to_x1, 'b')     % Discrete
legend('Discrete-time model')
title('Open-Loop Boost-Converter Model (x1/uc = ipv/(d/Kdpwm))')
ylabel('PV current ipv [A]')
grid on
% Closed loop (x1 versus u1)
subplot(2,1,2)
hold on
step(Ref * SLC_Boost_m_SF_Disc, 'b')       % Discrete
legend('Discrete-time model')
title('Closed-Loop Boost Converter with SF and Integral Action')
ylabel('PV current ipv [A]')
xlabel('Time [s]')
grid on
hold off;
K2=Kimd;
K1=K1y2md;

%% Continuous- and discrete-time PI-controller design
% Using SISO Tool
%      a*(1+b*s)
%Cpi = ---------
%          s
% For Ta = 0.1 s
%a = 5.5e6; %Selected value 1
%b = 0.0065;
%a =80.0e6;
%b = 0.0010;
a = 4.3049e+06;  % Scenarios 1-5
a = 3.8e+06;  % Scenarios 1-5
b = 0.0018;
b = 0.0015;

Kp1 = a*b
Ti1 = Kp1/a
% Controller discretization
KI1=Kp1/Ti1*Ts1;
KP1=Kp1-KI1/2;
Cs_PI_SISO=tf([Kp1 Kp1/Ti1],[1 0]);
Cz_PI=zpk(tf([(KP1+KI1) -KP1],[1 -1],Ts1))   % Discrete controller
TFclz=feedback(Cz_PI*Gz_uc_to_x1,1);
figure(3)
step(Ref * TFclz, 'b'); 
legend('Discrete (Digital)', 'Location', 'Best');
title('Boost Converter Closed-Loop Step Response with PI Control');
xlabel('Time [s]');
ylabel('PV current ipv [A]');
grid on;

%% Comparison of the currently implemented controllers
figure(4)
hold on;
% Step response with state-feedback control (SF)
step(Ref * SLC_Boost_m_SF_Disc, 'b');
% Step response with PI control
step(Ref * TFclz, 'r');
legend('State Feedback (SF)', 'Digital PI', 'Location', 'Best');
title('Boost Converter Closed-Loop Step Responses');
xlabel('Time [s]');
ylabel('PV current i_{pv} [A]');
grid on;
hold off;

%% Nominal parasitic parameters
ESRCpv=0.2;
RL=494e-3;
Ronfet=0.05;
Vfdiode=0.7;
Rondiode=0.05;
ESRCdc=0.2;
Fact1 = 0.7;
Fact2 = 1.3;

%MaMPPTCrtl
Kh1=1/ipv;
Ts2 = Ts1;

%TsuMPPTCtrl
paso = 0.001;
Univ_discurso=0.015*1;                         
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
axis([-7 7 -Univ_discurso Univ_discurso]);
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



    








