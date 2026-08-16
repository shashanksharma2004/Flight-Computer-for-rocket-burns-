% =========================================================================
% ADVANCED ROCKET FLIGHT SIMULATOR V3.1
% Includes: Live User Inputs, 2D Kinematics, Wind Drift, Sensor Fusion,
% Dual-Deploy Avionics, Live Telemetry Plotting, Post-Flight Dashboard, 
% and Automatic CSV Data Export.
% =========================================================================
clc; clear; close all;

%% 1. MISSION CONTROL: LIVE USER INPUTS
fprintf('========================================\n');
fprintf('  MISSION CONTROL PRE-FLIGHT CHECKLIST  \n');
fprintf('========================================\n');

% Get Crosswind
wind_speed = input('Enter crosswind speed (m/s) [Default 4.5]: ');
if isempty(wind_speed)
    wind_speed = 4.5;
end

% Get Rocket Mass
dry_mass = input('Enter rocket dry mass (kg) [Default 1.2]: ');
if isempty(dry_mass)
    dry_mass = 1.2;
end

% Get Parachute Setting
main_deploy_alt = input('Enter main chute deploy altitude (m) [Default 150.0]: ');
if isempty(main_deploy_alt)
    main_deploy_alt = 150.0;
end

% Get Launch Angle
launch_angle = input('Enter launch rod angle (degrees, 90 is straight up) [Default 86.0]: ');
if isempty(launch_angle)
    launch_angle = 86.0;
end

fprintf('\n>>> ALL PARAMETERS LOCKED IN. <<<\n\n');
input('Press ENTER to initiate launch sequence...', 's');

%% 2. SIMULATION & ENVIRONMENT PARAMETERS
dt = 0.05;                % Simulation time step (20 Hz loop)
gravity = 9.81;           % m/s^2

% Rocket Physical Properties
propellant_mass = 0.4;    % kg
rocket_radius = 0.04;     % meters
A_rocket = pi * rocket_radius^2;
Cd_rocket = 0.45;         % Aerodynamic drag coefficient during flight

% Parachute Properties
CdA_drogue = 1.2 * 0.1;   % Drag Coeff * Area of drogue chute
CdA_main = 1.5 * 0.8;     % Drag Coeff * Area of main chute

% Motor Profile (Simulated F/G Class Motor Thrust Curve)
motor_time =  [0.0, 0.1, 0.3, 0.8, 1.5, 2.0, 2.1];
motor_thrust = [0, 180, 250, 190, 120, 40,   0]; 
burn_time = 2.1;

%% 3. INITIALIZATION
% True State Variables (Physics Engine)
true_x = 0;      true_y = 0.1;   
true_vx = 0;     true_vy = 0;    
true_ax = 0;     true_ay = 0;    
current_mass = dry_mass + propellant_mass;

% Avionics & Sensor Estimates (Flight Computer)
est_y = true_y;
est_vy = 0;
max_est_y = 0;

% Flight Logic Flags
flight_phase = "PRE_LAUNCH";
mach_lockout = false;
drogue_deployed = false;
main_deployed = false;
apogee_detected = false;
descent_samples = 0;

% Data Logging Arrays (For Dashboard & CSV)
log_t = []; log_y = []; log_est_y = []; log_vy = []; 
log_ay = []; log_x = []; log_phase = strings(0);

%% 4. LIVE VISUALIZATION SETUP
fig = figure('Name', 'Live Flight Telemetry', 'Position', [100, 100, 800, 500]);
subplot(1,2,1);
trajectory_line = animatedline('Color', 'b', 'LineWidth', 1.5);
grid on; hold on;
xlabel('Downrange Distance X (m)');
ylabel('Altitude Y (m)');
title('Live 2D Flight Path');
axis([-50, 500, 0, 800]);

subplot(1,2,2);
alt_line = animatedline('Color', 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Altitude (m)');
title('Flight Computer Altitude Estimate');
axis([0, 40, 0, 800]);

fprintf('=== INITIATING LAUNCH SEQUENCE ===\n');
pause(1);

%% 5. MAIN FLIGHT LOOP
t = 0;
while true
    % ---------------------------------------------------------------------
    % A. PHYSICS & AERODYNAMICS
    % ---------------------------------------------------------------------
    
    % 1. Dynamic Mass Calculation (burn fuel over time)
    if t <= burn_time
        current_mass = dry_mass + propellant_mass * (1 - (t / burn_time));
        thrust_mag = interp1(motor_time, motor_thrust, t, 'linear', 0);
        if t > 0.0 && flight_phase == "PRE_LAUNCH"
            flight_phase = "BOOST";
            fprintf('[%.2fs] LAUNCH DETECTED\n', t);
        end
    else
        current_mass = dry_mass;
        thrust_mag = 0;
        if flight_phase == "BOOST"
            flight_phase = "COAST";
            fprintf('[%.2fs] MOTOR BURNOUT\n', t);
        end
    end
    
    % 2. Air Density (Standard Atmosphere Model approximation)
    rho = 1.225 * exp(-0.000118 * true_y);
    
    % 3. Aerodynamic Drag Calculation
    v_rel_x = true_vx - wind_speed;
    v_rel_y = true_vy;
    v_mag = sqrt(v_rel_x^2 + v_rel_y^2);
    
    % Determine active drag profile
    if main_deployed
        active_CdA = CdA_main;
    elseif drogue_deployed
        active_CdA = CdA_drogue;
    else
        active_CdA = Cd_rocket * A_rocket;
    end
    
    drag_force = 0.5 * rho * active_CdA * v_mag^2;
    
    % Split drag into X and Y components
    if v_mag > 0.01
        drag_x = -drag_force * (v_rel_x / v_mag);
        drag_y = -drag_force * (v_rel_y / v_mag);
    else
        drag_x = 0; drag_y = 0;
    end
    
    % 4. Thrust Vectors
    if thrust_mag > 0
        thrust_x = thrust_mag * cosd(launch_angle);
        thrust_y = thrust_mag * sind(launch_angle);
    else
        thrust_x = 0; thrust_y = 0;
    end
    
    % 5. Newtonian Kinematics Update
    true_ax = (thrust_x + drag_x) / current_mass;
    true_ay = (thrust_y + drag_y - (current_mass * gravity)) / current_mass;
    
    true_vx = true_vx + true_ax * dt;
    true_vy = true_vy + true_ay * dt;
    
    true_x = true_x + true_vx * dt;
    true_y = true_y + true_vy * dt;
    
    % Ground collision check
    if true_y <= 0 && t > 0.5
        true_y = 0; true_vx = 0; true_vy = 0;
        flight_phase = "TOUCHDOWN";
    end

    % ---------------------------------------------------------------------
    % B. AVIONICS & FLIGHT COMPUTER
    % ---------------------------------------------------------------------
    
    % 1. Sensor Inputs
    baro_y = true_y + (randn() * 1.5);     
    accel_y = true_ay + (randn() * 0.5);   
    
    % 2. Mach Lockout Protection
    mach_lockout = (flight_phase == "BOOST") || (true_vy > 150);
    
    % 3. Complementary Sensor Fusion Filter
    est_vy = est_vy + (accel_y * dt);
    est_y_pred = est_y + (est_vy * dt);
    
    if mach_lockout || flight_phase == "TOUCHDOWN"
        est_y = est_y_pred;
    else
        est_y = (0.95 * est_y_pred) + (0.05 * baro_y);
    end
    
    % 4. Apogee Detection
    if est_y > max_est_y
        max_est_y = est_y;
    end
    
    if flight_phase == "COAST"
        if est_vy < -1.0 
            descent_samples = descent_samples + 1;
        else
            descent_samples = 0;
        end
        
        if descent_samples >= 3 && ~apogee_detected
            apogee_detected = true;
            flight_phase = "DESCENT_DROGUE";
            drogue_deployed = true;
            fprintf('[%.2fs] APOGEE CONFIRMED AT %.1fm. DROGUE DEPLOYED.\n', t, est_y);
        end
    end
    
    % 5. Main Parachute Deployment
    if apogee_detected && ~main_deployed && (est_y <= main_deploy_alt)
        main_deployed = true;
        flight_phase = "DESCENT_MAIN";
        fprintf('[%.2fs] ALTITUDE %.1fm. MAIN PARACHUTE DEPLOYED.\n', t, est_y);
    end

    % ---------------------------------------------------------------------
    % C. DATA LOGGING & VISUALIZATION
    % ---------------------------------------------------------------------
    
    % Store data for dashboard and CSV export
    log_t(end+1) = t; log_y(end+1) = true_y; log_est_y(end+1) = est_y;
    log_vy(end+1) = true_vy; log_ay(end+1) = true_ay; log_x(end+1) = true_x;
    log_phase(end+1) = flight_phase;
    
    % Update live plots 
    if mod(t, 0.15) < dt 
        addpoints(trajectory_line, true_x, true_y);
        addpoints(alt_line, t, est_y);
        drawnow limitrate;
    end
    
    if flight_phase == "TOUCHDOWN"
        fprintf('[%.2fs] TOUCHDOWN CONFIRMED. MISSION SUCCESS.\n', t);
        fprintf('Total Drift Distance: %.1f meters\n\n', true_x);
        break;
    end
    
    t = t + dt;
end

%% 6. CSV FLIGHT LOG EXPORT
fprintf('Exporting flight logs to CSV...\n');

% Compile arrays into a MATLAB table (transposing rows to columns with ')
telemetry_table = table(log_t', log_phase', log_x', log_y', log_est_y', log_vy', log_ay', ...
    'VariableNames', {'Time_s', 'Flight_Phase', 'Downrange_X_m', 'True_Altitude_m', ...
                      'Filtered_Altitude_m', 'True_Velocity_Y_ms', 'True_Accel_Y_ms2'});

% Write table to CSV file
writetable(telemetry_table, 'flight_telemetry_log.csv');
fprintf('>>> Logs successfully saved to "flight_telemetry_log.csv" <<<\n\n');

%% 7. POST-FLIGHT TELEMETRY DASHBOARD
fprintf('Generating post-flight telemetry dashboard...\n');

dash = figure('Name', 'Post-Flight Dashboard', 'Position', [150, 150, 1000, 600]);

% Subplot 1: Altitude Comparison
subplot(3,1,1);
plot(log_t, log_y, 'k-', 'LineWidth', 1.5); hold on;
plot(log_t, log_est_y, 'r--', 'LineWidth', 1.2);
ylabel('Altitude (m)');
title('Altitude: True Physics vs. Computer Estimate');
legend('True Altitude', 'Filtered Estimate', 'Location', 'best');
grid on;

% Subplot 2: Vertical Velocity
subplot(3,1,2);
plot(log_t, log_vy, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('Velocity (m/s)');
title('Vertical Velocity Profile');
grid on;

% Subplot 3: Acceleration
subplot(3,1,3);
plot(log_t, log_ay, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Accel (m/s^2)');
title('Vertical Acceleration');
grid on;

drawnow;
