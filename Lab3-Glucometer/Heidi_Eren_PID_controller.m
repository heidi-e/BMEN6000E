%% Heidi Eren 
% Lab 3 - Q10
clear; clc; close all;

%% Simulation Parameters
dt = 0.1;          % time step (min)
t_end = 800;       % total time (min)
t = 0:dt:t_end;
N = numel(t);

% initial conditions
Q = zeros(1,N); G = zeros(1,N); I = zeros(1,N);
Q(1) = 0; 
G(1) = 163;        % mg/dL
I(1) = 11.26;      % µU/mL

%% Model Parameters (Diabetic)
beta = 10;  eta = 4.641;  gam = 25;
R0 = 2.5;   E = 2.5e-3;   S = 1.14e-3;
kq = 0.026; Imax = 0.93;  alpha = 1e4;  ki = 0.06;

%% meal parameters
mealStart = 0;
mealDuration = 15;          % min
mealSize = 15000;           % mg glucose (15 g total)
D = @(time) (mealSize / mealDuration) * (time>=mealStart & time<mealStart+mealDuration);

%% PID Controller
G_set = 120;                % target glucose [mg/dL]
Kp = 0.3;
Ki = 0.005;
Kd = 0.05;

g_prev = 0; sumG = 0;

%% Bolus Control
bolusThreshold = 160;       % mg/dL
bolusDose = 0.5;            % µU/mL/min
bolusDuration = 5;          % min
bolusTimer = 0;             % timer
addIn_bolus = 0;

%% plot
addIn_PID = zeros(1,N);
addIn_total = zeros(1,N);

%% closed loop
for k = 2:N
    time = t(k);

    % meal intake
    Dk = D(time);

    % bolus control
    if (G(k-1) > bolusThreshold && bolusTimer <= 0)
        addIn_bolus = bolusDose;
        bolusTimer = bolusDuration;
    end
    if (bolusTimer > 0)
        bolusTimer = bolusTimer - dt;
    else
        addIn_bolus = 0;
    end
    
    % PID control (use simulated glucose instead of analog sensor)
    g_error = G_set - G(k-1);
    sumG = sumG + g_error * dt;           % integral term
    g_deriv = (g_error - g_prev) / dt;    % derivative term
    g_prev = g_error;

    addIn_PID(k) = Kp * g_error + Ki * sumG + Kd * g_deriv;
    addIn_PID(k) = max(addIn_PID(k), 0);  % no negative insulin

    % total insulin input
    addIn_total(k) = addIn_PID(k) + addIn_bolus;

    % ODE
    dQ = - (beta * Q(k-1) + eta * Dk) / (gam^2 + Q(k-1)^2);
    dG = R0 - (E + S * I(k-1)) * G(k-1) + kq * Q(k-1);
    dI = Imax * (G(k-1)^2 / (alpha + G(k-1)^2)) - ki * I(k-1) + addIn_total(k);

    Q(k) = Q(k-1) + dQ * dt;
    G(k) = G(k-1) + dG * dt;
    I(k) = I(k-1) + dI * dt;
end

overshoot = max(G) - G_set;
steady_state_err = mean(G(end-100:end)) - G_set;
fprintf('Overshoot = %.1f mg/dL\n', overshoot);
fprintf('Steady-state error = %.1f mg/dL\n', steady_state_err);


%% final plots
figure('Color','w');

subplot(4,1,1)
plot(t, Q, 'LineWidth', 1.8);
ylabel('Q [mg]');
title('Discrete Glucose–Insulin Model (PID + Bolus Control)');
grid on;

subplot(4,1,2)
plot(t, G, 'r', 'LineWidth', 1.8); hold on;
yline(G_set, '--k', 'Target');
ylabel('G [mg/dL]');
legend('Glucose','Target');
grid on;

subplot(4,1,3)
plot(t, I, 'b', 'LineWidth', 1.8);
ylabel('I [\muU/mL]');
legend('Insulin');
grid on;

subplot(4,1,4)
plot(t, addIn_PID, 'm', 'LineWidth', 1.5); hold on;
plot(t, addIn_total, 'c--', 'LineWidth', 1.5);
ylabel('Insulin Input [\muU/mL/min]');
xlabel('Time [min]');
legend('PID Output','Total (PID + Bolus)');
grid on;
