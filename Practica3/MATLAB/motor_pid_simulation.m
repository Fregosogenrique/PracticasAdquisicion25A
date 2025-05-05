%% DC Motor PID Control Simulation
% This script simulates a DC motor with PID control and analyzes performance
% Features:
% - DC motor system modeling with configurable parameters
% - PID controller implementation in continuous and discrete time
% - Step response simulation and analysis
% - Parameter tuning visualization
% - Performance metrics calculation

close all;
clear all;
clc;

%% Motor Parameters (adjustable)
% Typical values for a small DC motor with encoder
J = 0.01;      % Moment of inertia of the rotor (kg*m^2)
b = 0.1;       % Motor viscous friction constant (N*m*s)
Kt = 0.01;     % Motor torque constant (N*m/A)
Ke = 0.01;     % Back EMF constant (V/rad/s)
R = 1;         % Armature resistance (ohm)
L = 0.5;       % Armature inductance (H)

% Create a state-space model
A = [-b/J   Kt/J
     -Ke/L  -R/L];
B = [0
     1/L];
C = [1 0];  % Measuring angular position
D = 0;

% Convert to transfer function
motor_ss = ss(A, B, C, D);
motor_tf = tf(motor_ss);
disp('DC Motor Transfer Function (Position):');
display(motor_tf);

% Convert to velocity output
motor_vel_ss = ss(A, B, [0 1], D);
motor_vel_tf = tf(motor_vel_ss);
disp('DC Motor Transfer Function (Velocity):');
display(motor_vel_tf);

%% PID Controller Design (Continuous Time)
% Initial PID values (adjustable)
Kp = 100;      % Proportional gain
Ki = 200;      % Integral gain
Kd = 10;       % Derivative gain

% Create PID controller transfer function
s = tf('s');
C_pid = Kp + Ki/s + Kd*s;

% Filter the derivative term to make it proper
N = 100;  % Filter coefficient
C_pid_filtered = Kp + Ki/s + (Kd*s)/(1 + s/N);

% Open loop transfer function
OL_tf = C_pid_filtered * motor_tf;

% Closed loop transfer function
CL_tf = feedback(OL_tf, 1);

% Display controller and closed-loop transfer functions
disp('PID Controller Transfer Function:');
display(C_pid_filtered);
disp('Closed-Loop Transfer Function:');
display(CL_tf);

%% Step Response Simulation
t = 0:0.01:2;  % Time vector (2 seconds)
[y, t] = step(CL_tf, t);
u = step(C_pid_filtered * feedback(1, motor_tf), t);  % Control signal

% Calculate performance metrics
S = stepinfo(y, t);
fprintf('\nPerformance Metrics (Continuous Time):\n');
fprintf('Rise Time: %.4f seconds\n', S.RiseTime);
fprintf('Settling Time: %.4f seconds\n', S.SettingTime);
fprintf('Overshoot: %.2f%%\n', S.Overshoot);
fprintf('Peak: %.4f\n', S.Peak);
fprintf('Peak Time: %.4f seconds\n', S.PeakTime);

% Plot step response
figure;
subplot(2,1,1);
plot(t, y, 'LineWidth', 2);
grid on;
title('Step Response (Continuous Time)');
xlabel('Time (seconds)');
ylabel('Position (rad)');
% Add horizontal lines for rise time and settling
hold on;
plot([0, max(t)], [0.9, 0.9], 'r--');
plot([0, max(t)], [1.02, 1.02], 'g--');
plot([0, max(t)], [0.98, 0.98], 'g--');
legend('Response', '90% Rise', '±2% Settling', 'Location', 'best');

% Plot control signal
subplot(2,1,2);
plot(t, u, 'LineWidth', 2);
grid on;
title('Control Signal');
xlabel('Time (seconds)');
ylabel('Input Voltage (V)');

%% Discrete-Time Implementation
% Sample time (matching the Arduino implementation)
Ts = 0.01;  % 10ms sample time

% Convert continuous system to discrete-time
motor_d = c2d(motor_tf, Ts, 'zoh');
disp('Discrete-Time Motor Transfer Function:');
display(motor_d);

% Discrete PID controller
z = tf('z', Ts);
Kp_d = Kp;
Ki_d = Ki * Ts;
Kd_d = Kd / Ts;

% Discrete PID with filtered derivative
C_pid_d = Kp_d + Ki_d*z/(z-1) + Kd_d*(z-1)/z;

% Apply tustin approximation for derivative filter
alpha = N*Ts/(2+N*Ts);
C_pid_filtered_d = Kp_d + Ki_d*z/(z-1) + Kd_d*(z-1)/(alpha*z + (1-alpha));

% Discrete closed-loop transfer function
OL_tf_d = C_pid_filtered_d * motor_d;
CL_tf_d = feedback(OL_tf_d, 1);

% Display discrete controller and closed-loop transfer functions
disp('Discrete PID Controller Transfer Function:');
display(C_pid_filtered_d);
disp('Discrete Closed-Loop Transfer Function:');
display(CL_tf_d);

% Simulate discrete step response
[y_d, t_d] = step(CL_tf_d, t);
u_d = step(C_pid_filtered_d * feedback(1, motor_d), t);  % Discrete control signal

% Calculate performance metrics
S_d = stepinfo(y_d, t_d);
fprintf('\nPerformance Metrics (Discrete Time):\n');
fprintf('Rise Time: %.4f seconds\n', S_d.RiseTime);
fprintf('Settling Time: %.4f seconds\n', S_d.SettingTime);
fprintf('Overshoot: %.2f%%\n', S_d.Overshoot);
fprintf('Peak: %.4f\n', S_d.Peak);
fprintf('Peak Time: %.4f seconds\n', S_d.PeakTime);

% Plot discrete step response
figure;
subplot(2,1,1);
plot(t_d, y_d, 'LineWidth', 2);
grid on;
title('Step Response (Discrete Time)');
xlabel('Time (seconds)');
ylabel('Position (rad)');
% Add horizontal lines for rise time and settling
hold on;
plot([0, max(t)], [0.9, 0.9], 'r--');
plot([0, max(t)], [1.02, 1.02], 'g--');
plot([0, max(t)], [0.98, 0.98], 'g--');
legend('Response', '90% Rise', '±2% Settling', 'Location', 'best');

% Plot discrete control signal
subplot(2,1,2);
plot(t_d, u_d, 'LineWidth', 2);
grid on;
title('Discrete Control Signal');
xlabel('Time (seconds)');
ylabel('Input Voltage (V)');

%% Parameter Tuning Visualization
% Create a figure to visualize the effect of changing PID parameters
figure;

% Effect of changing Kp
Kp_values = [0.5*Kp, Kp, 2*Kp];
subplot(3,1,1);
hold on;
for i = 1:length(Kp_values)
    C_temp = (Kp_values(i) + Ki/s + (Kd*s)/(1 + s/N)) * motor_tf;
    CL_temp = feedback(C_temp, 1);
    [y_temp, t_temp] = step(CL_temp, t);
    plot(t_temp, y_temp, 'LineWidth', 2);
end
grid on;
title('Effect of Proportional Gain (Kp)');
xlabel('Time (seconds)');
ylabel('Position (rad)');
legend(sprintf('Kp=%.1f', Kp_values(1)), sprintf('Kp=%.1f', Kp_values(2)), sprintf('Kp=%.1f', Kp_values(3)), 'Location', 'best');

% Effect of changing Ki
Ki_values = [0.5*Ki, Ki, 2*Ki];
subplot(3,1,2);
hold on;
for i = 1:length(Ki_values)
    C_temp = (Kp + Ki_values(i)/s + (Kd*s)/(1 + s/N)) * motor_tf;
    CL_temp = feedback(C_temp, 1);
    [y_temp, t_temp] = step(CL_temp, t);
    plot(t_temp, y_temp, 'LineWidth', 2);
end
grid on;
title('Effect of Integral Gain (Ki)');
xlabel('Time (seconds)');
ylabel('Position (rad)');
legend(sprintf('Ki=%.1f', Ki_values(1)), sprintf('Ki=%.1f', Ki_values(2)), sprintf('Ki=%.1f', Ki_values(3)), 'Location', 'best');

% Effect of changing Kd
Kd_values = [0.5*Kd, Kd, 2*Kd];
subplot(3,1,3);
hold on;
for i = 1:length(Kd_values)
    C_temp = (Kp + Ki/s + (Kd_values(i)*s)/(1 + s/N)) * motor_tf;
    CL_temp = feedback(C_temp, 1);
    [y_temp, t_temp] = step(CL_temp, t);
    plot(t_temp, y_temp, 'LineWidth', 2);
end
grid on;
title('Effect of Derivative Gain (Kd)');
xlabel('Time (seconds)');
ylabel('Position (rad)');
legend(sprintf('Kd=%.1f', Kd_values(1)), sprintf('Kd=%.1f', Kd_values(2)), sprintf('Kd=%.1f', Kd_values(3)), 'Location', 'best');

%% Root Locus and Bode Plots for Analysis
figure;
rlocus(OL_tf);
title('Root Locus Plot');
grid on;

figure;
margin(OL_tf);
title('Bode Plot with Stability Margins');
grid on;

%% Interactive PID Tuning Function
% Uncomment to use the interactive tuning tools
% pidTuner(motor_tf, 'pid');

%% Speed Control Analysis
% Create closed-loop system for velocity control
OL_vel_tf = C_pid_filtered * motor_vel_tf;
CL_vel_tf = feedback(OL_vel_tf, 1);

% Simulate step response for velocity control
figure;
step(CL_vel_tf, t);
title('Velocity Control Step Response');
grid on;

%% Save PID Parameters to File for Arduino
% Save the tuned PID parameters to a file that can be used as reference
% for the Arduino implementation
pid_params = [Kp, Ki, Kd];
save('pid_params.mat', 'pid_params');

% Also save as a CSV file for easy import to other systems
csvwrite('pid_params.csv', pid_params);

fprintf('\nTuned PID Parameters:\n');
fprintf('Kp = %.2f\n', Kp);
fprintf('Ki = %.2f\n', Ki);
fprintf('Kd = %.2f\n', Kd);
fprintf('Parameters saved to pid_params.mat and pid_params.csv\n');

%% Additional function to evaluate PID performance with different parameters
% This function can be called separately to test different combinations of parameters
disp('You can call evaluatePID(Kp, Ki, Kd) to test different parameters');

%% Function definition for PID evaluation
function metrics = evaluatePID(Kp, Ki, Kd)
    % Motor parameters (same as above)
    J = 0.01;      % Moment of inertia of the rotor
    b = 0.1;       % Motor viscous friction constant
    Kt = 0.01;     % Motor torque constant
    Ke = 0.01;     % Back EMF constant
    R = 1;         % Armature resistance
    L = 0.5;       % Armature inductance
    
    % Create motor model
    A = [-b/J   Kt/J
         -Ke/L  -R/L];
    B = [0
         1/L];
    C = [1 0];  % Position measurement
    D = 0;
    motor_ss = ss(A, B, C, D);
    motor_tf = tf(motor_ss);
    
    % Create PID controller
    s = tf('s');
    N = 100;  % Filter coefficient
    C_pid = Kp + Ki/s + (Kd*s)/(1 + s/N);
    
    % Create closed-loop system
    CL_tf = feedback(C_pid * motor_tf, 1);
    
    % Simulate step response
    t = 0:0.01:2;
    [y, t] = step(CL_tf, t);
    
    % Calculate metrics
    S = stepinfo(y, t);
    
    % Display results
    fprintf('\nPerformance with Kp=%.2f, Ki=%.2f, Kd=%.2f:\n', Kp, Ki, Kd);
    fprintf('Rise Time: %.4f seconds\n', S.RiseTime);
    fprintf('Settling Time: %.4f seconds\n', S.SettingTime);
    fprintf('Overshoot: %.2f%%\n', S.Overshoot);
    
    % Plot response
    figure;
    plot(t, y, 'LineWidth', 2);
    grid on;
    title(sprintf('Step Response with Kp=%.2f, Ki=%.2f, Kd=%.2f', Kp, Ki, Kd));
    xlabel('Time (seconds)');
    ylabel('Position (rad)');
    
    % Return metrics
    metrics.RiseTime = S.RiseTime;
    metrics.SettlingTime = S.SettingTime;
    metrics.Overshoot = S.Overshoot;
    metrics.Peak = S.Peak;
    metrics.PeakTime = S.PeakTime;
end

