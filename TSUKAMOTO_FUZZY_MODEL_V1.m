clc; clear all; close all;
set(0,'DefaultLegendInterpreter','tex');   

%% Tsukamoto CoDif Controller for the boost converter
Fs  = 5e3;                     % Switching frequency of the boost converter
Ts3 = 1 / Fs;                  % Sampling time of the CoDif controller
paso = 0.001;                  % Sampling step for the universes of discourse (plotting)
Univ_discurso = 0.015;         % With 0.18, maximum performance is achieved (design note)
x = -10:paso:10;               % Universe of discourse for e
y = -Univ_discurso:paso:Univ_discurso; % Universe of discourse for Delta_d (output)
a = 1;                         % Slope parameter to smooth the control curve

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
