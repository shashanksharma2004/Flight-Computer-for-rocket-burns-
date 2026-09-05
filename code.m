% =========================================================================
% Rocket Flight Dynamics & Avionics Simulation
% Model: 2D Kinematics, Wind Drift, Dual-Deploy Recovery, Sensor Fusion
% =========================================================================
clc; clear; close all;

%% 1. User Configuration
disp('--------------------------------------------------');
disp('            PRE-FLIGHT CONFIGURATION              ');
disp('--------------------------------------------------');

wind_speed = input('Crosswind speed (m/s) [Default 4.5]: ');
if isempty(wind_speed), wind_speed = 4.5; end

dry_mass = input('Rocket dry mass (kg) [Default 1.2]: ');
if isempty(dry_mass), dry_mass = 1.2; end

main_deploy_alt = input('Main chute deployment altitude (m) [Default 150]: ');
if isempty(main_deploy_alt), main_deploy_alt = 150.0; end

launch_angle = input('Launch rod angle (deg from horizontal) [Default 86]: ');
if isempty(launch_angle), launch_angle = 86.0; end

disp('Configuration saved.');
input('Press ENTER to run simulation...', 's');

%% 2. System Parameters & Constants
dt = 0.05;          % Time step (20 Hz)
g = 9.81;           % Gravity (m/s^2)

% Rocket specifications
propellant_mass = 0.4;    % kg
rocket_radius   = 0.04;   % m
A_rocket        = pi * rocket_radius^2;
Cd_rocket       = 0.45;

% Parachute specifications
CdA_drogue = 1.2 * 0.1;   % Cd * Area (drogue)
CdA_main   = 1.5 * 0.8;   % Cd * Area (main)

% Motor thrust profile (Class F/G approximation)
motor_time   = [0.0, 0.1, 0.3, 0.8, 1.5, 2.0, 2.1];
motor_thrust = [  0, 180, 250, 190, 120,  40,   0]; 
burn_time    = 2.1;

%% 3. State Initialization
% Physical state
x = 0;   y = 0.1;
vx = 0;  vy = 0;
ax = 0;  ay = 0;
m = dry_mass + propellant_mass;

% Estimator state
est_y = y;
est_vy = 0;
max_est_y = 0;

% Logic flags
phase = "PRE_LAUNCH";
drogue_deployed = false;
main_deployed = false;
apogee_detected = false;
descent_count = 0;

% Data logs
log_t = []; log_y = []; log_est_y = []; 
log_vy = []; log_ay = []; log_x = []; 
log_phase = strings(0);

%% 4. Telemetry Window Setup
fig = figure('Name', 'Live Telemetry', 'Position', [100, 100, 800, 450]);

subplot(1,2,1);
path_line = animatedline('Color', 'b', 'LineWidth', 1.5);
grid on; hold on;
xlabel('Downrange (m)'); ylabel('Altitude (m)');
title('Trajectory (2D)');
axis([-50, 500, 0, 800]);

subplot(1,2,2);
alt_line = animatedline('Color', 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)'); ylabel('Altitude (m)');
title('Estimated Altitude');
axis([0, 40, 0, 800]);

fprintf('\nStarting simulation...\n');
pause(0.5);

%% 5. Simulation Loop
t = 0;
while true
    % --- A. Physical Simulation ---
    
    % Mass and Thrust
    if t <= burn_time
        m = dry_mass + propellant_mass * (1 - (t / burn_time));
        thrust = interp1(motor_time, motor_thrust, t, 'linear', 0);
        
        if t > 0 && phase == "PRE_LAUNCH"
            phase = "BOOST";
            fprintf('[%6.2fs] Event: Ignition / Liftoff\n', t);
        end
    else
        m = dry_mass;
        thrust = 0;
        
        if phase == "BOOST"
            phase = "COAST";
            fprintf('[%6.2fs] Event: Motor Burnout\n', t);
        end
    end
    
    % Atmospheric density approximation
    rho = 1.225 * exp(-0.000118 * y);
    
    % Drag calculation (relative airspeed)
    v_rel_x = vx - wind_speed;
    v_rel_y = vy;
    v_mag = hypot(v_rel_x, v_rel_y);
    
    if main_deployed
        CdA = CdA_main;
    elseif drogue_deployed
        CdA = CdA_drogue;
    else
        CdA = Cd_rocket * A_rocket;
    end
    
    f_drag = 0.5 * rho * CdA * v_mag^2;
    
    if v_mag > 0.01
        fx_drag = -f_drag * (v_rel_x / v_mag);
        fy_drag = -f_drag * (v_rel_y / v_mag);
    else
        fx_drag = 0; 
        fy_drag = 0;
    end
    
    % Forces -> Acceleration
    fx_thrust = thrust * cosd(launch_angle);
    fy_thrust = thrust * sind(launch_angle);
    
    ax = (fx_thrust + fx_drag) / m;
    ay = (fy_thrust + fy_drag - (m * g)) / m;
    
    % Integration (Euler)
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    x  = x  + vx * dt;
    y  = y  + vy * dt;
    
    % Ground collision
    if y <= 0 && t > 0.5
        y = 0; vx = 0; vy = 0;
        phase = "TOUCHDOWN";
    end
    
    % --- B. Avionics / Estimator ---
    
    % Sensor noise simulation
    meas_y  = y  + (randn() * 1.5);
    meas_ay = ay + (randn() * 0.5);
    
    % Complementary Filter
    mach_lockout = (phase == "BOOST") || (vy > 150);
    
    est_vy = est_vy + (meas_ay * dt);
    est_y_pred = est_y + (est_vy * dt);
    
    if mach_lockout || phase == "TOUCHDOWN"
        est_y = est_y_pred;
    else
        est_y = (0.95 * est_y_pred) + (0.05 * meas_y);
    end
    
    % Apogee check
    if est_y > max_est_y
        max_est_y = est_y;
    end
    
    if phase == "COAST"
        if est_vy < -1.0 
            descent_count = descent_count + 1;
        else
            descent_count = 0;
        end
        
        % Confirm apogee after 3 consistent downward readings
        if descent_count >= 3 && ~apogee_detected
            apogee_detected = true;
            phase = "DESCENT_DROGUE";
            drogue_deployed = true;
            fprintf('[%6.2fs] Event: Apogee detected at %.1fm. Drogue deployed.\n', t, est_y);
        end
    end
    
    % Main chute deployment
    if apogee_detected && ~main_deployed && (est_y <= main_deploy_alt)
        main_deployed = true;
        phase = "DESCENT_MAIN";
        fprintf('[%6.2fs] Event: Main chute deployed at %.1fm.\n', t, est_y);
    end
    
    % --- C. Logging & Visualization ---
    
    log_t(end+1)     = t; 
    log_y(end+1)     = y; 
    log_est_y(end+1) = est_y;
    log_vy(end+1)    = vy; 
    log_ay(end+1)    = ay; 
    log_x(end+1)     = x;
    log_phase(end+1) = phase;
    
    % Render plot at ~6 Hz to save CPU
    if mod(t, 0.15) < dt 
        addpoints(path_line, x, y);
        addpoints(alt_line, t, est_y);
        drawnow limitrate;
    end
    
    if phase == "TOUCHDOWN"
        fprintf('[%6.2fs] Event: Touchdown. Landing drift: %.1f m.\n', t, x);
        break;
    end
    
    t = t + dt;
end

%% 6. CSV Export
filename = 'flight_telemetry_log.csv';
telemetry_data = table(log_t', log_phase', log_x', log_y', log_est_y', log_vy', log_ay', ...
    'VariableNames', {'Time_s', 'Phase', 'Downrange_m', 'True_Alt_m', ...
                      'Est_Alt_m', 'Velocity_Y_ms', 'Accel_Y_ms2'});

writetable(telemetry_data, filename);
fprintf('\nData logged to %s\n', filename);

%% 7. Post-Flight Plotting
figure('Name', 'Post-Flight Analysis', 'Position', [150, 150, 900, 600]);

subplot(3,1,1);
plot(log_t, log_y, 'k-', 'LineWidth', 1.5); hold on;
plot(log_t, log_est_y, 'r--', 'LineWidth', 1.2);
ylabel('Altitude (m)');
title('Altitude');
legend('True', 'Filtered', 'Location', 'best');
grid on;

subplot(3,1,2);
plot(log_t, log_vy, 'b-', 'LineWidth', 1.5);
yline(0, 'k--');
ylabel('Velocity (m/s)');
title('Vertical Velocity');
grid on;

subplot(3,1,3);
plot(log_t, log_ay, 'm-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Accel (m/s^2)');
title('Vertical Acceleration');
grid on;
