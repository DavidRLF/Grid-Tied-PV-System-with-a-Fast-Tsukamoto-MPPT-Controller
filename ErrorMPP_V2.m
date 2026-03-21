clc; clear; close all;

%% ================== PHYSICAL PARAMETERS OF THE PV PANEL/ARRAY ==================
Ncell    = 72;          
Voc_STC  = 44.2;        
Isc      = 8.42;        
Ki       = 0.005305;    
Rspanel  = 0.38773;     
Rshpanel = 300.2269;    
Is       = 7.7188e-10;  
n        = 1.0345;      

% Physical constants
k1   = 1.3806e-23;      
q    = 1.60218e-19;     
Tref = 298.15;          
Gref = 1000;            

% Bandgap parameters
Egref = 1.121;          
dEgdT = -0.0002677;     

% Panel geometry
Apanel = 195.4*98.2;    
Acell  = Apanel/Ncell; %#ok<NASGU>
% Acell is computed for completeness, although it is not explicitly used later

% PV array configuration
Nser = 18;              
Npar = 2;               

% Fine calibration factor for the light-generated current IL
% This coefficient slightly adjusts the photocurrent to better match the reference model
gamma_IL = 1.00255;

%% ================== PARAMETERS FOR Eq. (3) ==================
% You can use the nominal array Impp reported in the paper
Impp_STC_array = 15.71;   % A, full array current at STC

%% ================== CONTROL SWITCHES ==================
% Flag to save the results table to an Excel file
do_save = true;

% Output filename
outfile_error = 'dataset_error_Eq3_vs_full_model.xlsx';

% Plot enable/disable flags
plot_error_vs_G = true;
plot_error_surface = true;
plot_Impp_compare = true;

%% ================== SWEEP RANGES ==================
% Temperature range in °C
T_min = 15;   
T_max = 55;   
dT    = 10;
T_vec = T_min:dT:T_max;

% Irradiance range in W/m^2
G_min = 200;  
G_max = 1200; 
dG    = 200;
G_vec = G_min:dG:G_max;

% Array voltage sweep used to build the I-V curve
% The script evaluates the nonlinear PV equation for each voltage point
Va = 0:1:800;

%% ================== NUMERICAL OPTIONS ==================
% Options for the nonlinear solver fsolve
% Tight tolerances are used to obtain accurate numerical solutions
opts = optimoptions('fsolve','Display','off', ...
    'FunctionTolerance',1e-12,'StepTolerance',1e-12);

%% ================== PREALLOCATION ==================
% Number of temperature points
nT = numel(T_vec);

% Number of irradiance points
nG = numel(G_vec);

% Total number of operating conditions to evaluate
N  = nT * nG;

% Preallocate vectors to store results for all operating conditions
T_col          = zeros(N,1);
G_col          = zeros(N,1);
Vmpp_col       = zeros(N,1);
Impp_full_col  = zeros(N,1);
Pmpp_col       = zeros(N,1);
Impp_eq3_col   = zeros(N,1);
ErrAbs_col     = zeros(N,1);
ErrRel_col     = zeros(N,1);

% Row counter for storing each evaluated case
row = 0;

%% ================== MAIN LOOPS ==================
% Outer loop over temperature
for it = 1:nT
    T_degC = T_vec(it);
    Tcell  = 273.15 + T_degC;   % Convert temperature from °C to Kelvin
    
    % Bandgap energy updated with temperature
    Eg     = Egref*(1 + dEgdT*(Tcell - Tref));  

    % Nonlinear PV equation written as a residual function
    % fsolve will search for the current I that makes this expression equal to zero
    % Inputs:
    %   I      -> current to be solved
    %   Va_loc -> array voltage at the current point of the I-V sweep
    %   G_loc  -> irradiance for the current operating condition
    residual = @(I,Va_loc,G_loc) ...
        + gamma_IL * Npar * ( Isc/Apanel + Ki*(Tcell - Tref)/Apanel ) * (G_loc/Gref) * Apanel ...
        - ( Npar*Is*(Tcell/Tref)^3 .* exp( (q*Egref)/(k1*Tref) - (q*Eg)/(k1*Tcell) ) ) ...
            .* ( exp( ( Va_loc + I.*Rspanel*(Nser/Npar) ) ./ ( Nser * n * Ncell * (k1*Tcell/q) ) ) - 1 ) ...
        - ( Va_loc + I.*Rspanel*(Nser/Npar) ) ./ ( Rshpanel*(Nser/Npar) ) .* ( G_loc/Gref ) ...
        - I;

    % Inner loop over irradiance
    for ig = 1:nG
        G = G_vec(ig);

        % Initial seed for the nonlinear solver
        % A reasonable initial guess improves convergence and reduces computation issues
        Iseed = gamma_IL * Npar * ( Isc/Apanel + Ki*(Tcell - Tref)/Apanel ) * (G/Gref) * Apanel;
        Iseed = max(0, Iseed);

        % Solve the I-V curve point by point
        Ivals = zeros(size(Va));
        for k = 1:numel(Va)
            Vk = Va(k);

            % Keep the seed nonnegative before calling the solver
            Iseed    = max(0, Iseed);

            % Solve the nonlinear PV equation at the current voltage point
            Ivals(k) = fsolve(@(I) residual(I, Vk, G), Iseed, opts);

            % Update the seed with the previous solution
            % This helps fsolve follow the I-V curve smoothly from one voltage point to the next
            Iseed    = Ivals(k);
        end

        % Physical magnitudes
        % Negative numerical values are clipped to zero to keep only physically meaningful points
        Ivis = max(Ivals, 0);
        Pvis = max(Va .* Ivis, 0);

        % Keep only finite and physically valid values
        mask = isfinite(Pvis) & isfinite(Ivis) & Va >= 0;
        Vphys = Va(mask);
        Iphys = Ivis(mask);
        Pphys = Pvis(mask);

        % Find the maximum power point from the computed P-V curve
        [Pmpp, iM] = max(Pphys);
        Vmpp       = Vphys(iM);
        Impp_full  = Iphys(iM);

        % Eq. (3): approximate Impp of the full array
        % This simplified expression assumes Impp varies linearly with irradiance
        Impp_eq3 = Impp_STC_array * (G / Gref);

        % Error between the simplified approximation and the full nonlinear model
        ErrAbs = abs(Impp_eq3 - Impp_full);
        ErrRel = 100 * ErrAbs / Impp_full;

        % Store results
        row = row + 1;
        T_col(row)         = T_degC;
        G_col(row)         = G;
        Vmpp_col(row)      = Vmpp;
        Impp_full_col(row) = Impp_full;
        Pmpp_col(row)      = Pmpp;
        Impp_eq3_col(row)  = Impp_eq3;
        ErrAbs_col(row)    = ErrAbs;
        ErrRel_col(row)    = ErrRel;
    end
end

%% ================== RESULTING TABLE ==================
% Build a table with all evaluated operating conditions and the corresponding errors
DataError = table(T_col, G_col, Vmpp_col, Impp_full_col, Pmpp_col, ...
                  Impp_eq3_col, ErrAbs_col, ErrRel_col, ...
    'VariableNames', {'T','G','Vmpp','Impp_full','Pmpp','Impp_Eq3','ErrAbs_A','ErrRel_pct'});

% Display the complete table in the MATLAB command window
disp(DataError);

% Save the table to an Excel spreadsheet if requested
if do_save
    writetable(DataError, outfile_error, 'FileType','spreadsheet');
end

%% ================== GLOBAL SUMMARY ==================
% Compute global error statistics over all evaluated conditions
mean_err = mean(ErrRel_col);
max_err  = max(ErrRel_col);
min_err  = min(ErrRel_col);

fprintf('\n==== SUMMARY OF Eq. (3) ERROR ====\n');
fprintf('Mean relative error    = %.4f %%\n', mean_err);
fprintf('Maximum relative error = %.4f %%\n', max_err);
fprintf('Minimum relative error = %.4f %%\n', min_err);

%% ================== AVERAGE ERROR BY TEMPERATURE ==================
% Report the mean and maximum relative error for each temperature level
fprintf('\n==== AVERAGE RELATIVE ERROR BY TEMPERATURE ====\n');
for it = 1:nT
    Tnow = T_vec(it);
    idxT = (T_col == Tnow);
    fprintf('T = %5.1f °C --> Mean error = %.4f %% | Maximum error = %.4f %%\n', ...
        Tnow, mean(ErrRel_col(idxT)), max(ErrRel_col(idxT)));
end

%% ================== FIGURE 1: ERROR vs G FOR EACH T ==================
if plot_error_vs_G
    figure;
    hold on; grid on; box on;

    % Plot relative error versus irradiance for each temperature
    for it = 1:nT
        Tnow = T_vec(it);
        idxT = (T_col == Tnow);
        plot(G_col(idxT), ErrRel_col(idxT), '-o', 'LineWidth', 1.3, ...
            'DisplayName', sprintf('T = %g °C', Tnow));
    end

    xlabel('Irradiance G (W/m^2)');
    ylabel('Relative error of Eq. (3) (%)');
    title('Relative approximation error of Eq. (3) under different temperatures');
    legend('Location','best');
end

%% ================== FIGURE 2: ERROR SURFACE ==================
if plot_error_surface
    % Get sorted unique temperature and irradiance values
    Tu = unique(T_col, 'sorted');
    Gu = unique(G_col, 'sorted');

    % Build the grid for the surface plot
    [TT, GG] = meshgrid(Tu, Gu);
    EE = nan(numel(Gu), numel(Tu));

    % Fill the error matrix with the corresponding relative error values
    for i = 1:height(DataError)
        iT = find(Tu == DataError.T(i));
        iG = find(Gu == DataError.G(i));
        EE(iG, iT) = DataError.ErrRel_pct(i);
    end

    figure;
    surf(TT, GG, EE);
    shading interp;
    colorbar;
    grid on; box on;
    xlabel('Temperature T (°C)');
    ylabel('Irradiance G (W/m^2)');
    zlabel('Relative error (%)');
    title('Approximation error surface of Eq. (3)');
end

%% ================== FIGURE 3: FULL Impp vs Eq. (3) ==================
if plot_Impp_compare
    figure;
    hold on; grid on; box on;

    % Plot the full-model Impp for each temperature
    for it = 1:nT
        Tnow = T_vec(it);
        idxT = (T_col == Tnow);
        plot(G_col(idxT), Impp_full_col(idxT), '-o', 'LineWidth', 1.4, ...
            'DisplayName', sprintf('Full model, T=%g °C', Tnow));
    end

    % Plot the Eq. (3) approximation as a single reference line
    Gline = unique(G_col);
    Impp_eq3_line = Impp_STC_array * (Gline / Gref);
    plot(Gline, Impp_eq3_line, 'k--', 'LineWidth', 2.0, ...
        'DisplayName', 'Eq. (3)');

    xlabel('Irradiance G (W/m^2)');
    ylabel('I_{mpp} (A)');
    title('Comparison between full-model I_{mpp} and Eq. (3)');
    legend('Location','best');
end