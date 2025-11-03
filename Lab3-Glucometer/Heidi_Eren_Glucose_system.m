% Heidi Eren
% Lab 3 - Part 2 Q8
%% Food–Glucose–Insulin Model (non-diabetic vs diabetic)
clear; clc; close all;

%% Define parameters for each condition
params.normal = struct( ...
    'beta_val', 20, 'gamma', 40, 'eta', 4.086, ...
    'R0', 2.1, 'EG0', 1e-3, 'SI', 3.06e-3, ...
    'alpha', 1e4, 'Imax', 0.28, 'kI', 0.01, 'kQ', 0.098);

params.diabetic = struct( ...
    'beta_val', 10, 'gamma', 25, 'eta', 4.641, ...
    'R0', 2.5, 'EG0', 2.5e-3, 'SI', 1.14e-3, ...
    'alpha', 1e4, 'Imax', 0.93, 'kI', 0.06, 'kQ', 0.026);

%% Simulation settings
tspan = [0 800];   % minutes
Q0 = 0;            % mg
G0 = 90;           % mg/dl
I0 = 10;           % µU/ml
y0 = [Q0; G0; I0];
meal = 15000/15; %mg/min

% glucose intake: 1 meal at 0–15min
D = @(t) meal * ((t>=0 & t<=15));

% insulin input: bolus at start of meal
tau = 60; %min
b0 = 30; %µU/mL/min  = 15U over 60min
%B = @(t) b0 * ((t>0 & t<=0+tau));

% insulin input: bolus after meal
%B = @(t) b0 * ((t>75 & t<=75+tau));

% insulin input: bolus before meal and corrective bolus
B = @(t) b0 * ((t>-15 & t<=-15+tau) + (t>90 & t<=90+tau));

% insulin input: bolus before meal and basal rate
%B_pre = @(t) b0 * ((t>-15 & t<=-15+tau) + (t>90 & t<=90+tau));
%B_basal = @(t) 1.0 + 0*t;
%B = @(t) B_pre(t) + B_basal(t);

%% ODEs system function

food_glucose_insulin = @(t, y, p) [
    (-p.beta_val*y(1) + p.eta*D(t)) / (p.gamma^2 + y(1)^2);  % dQ/dt
    p.R0 - (p.EG0 + p.SI*y(3))*y(2) + p.kQ*y(1);             % dG/dt
    p.Imax*(y(2)^2 / (p.alpha + y(2)^2)) - p.kI*y(3) + B(t)  % dI/dt
];

%% Simulate both conditions
conds = fieldnames(params);
results = struct();

for i = 1:numel(conds)
    cond = conds{i};
    p = params.(cond);
    dYdt = @(t, y) food_glucose_insulin(t, y, p);
    [t, Y] = ode45(dYdt, tspan, y0);
    results.(cond).t = t;
    results.(cond).Q = Y(:,1);
    results.(cond).G = Y(:,2);
    results.(cond).I = Y(:,3);
end

%% Plot comparison
figure('Name', 'Glucose–Insulin Dynamics Model', 'Color', 'w');

subplot(3,1,1)
plot(results.normal.t, results.normal.Q, 'b', 'LineWidth', 2); hold on;
plot(results.diabetic.t, results.diabetic.Q, 'r--', 'LineWidth', 2);
ylabel('Q(t) [mg]');
title('Food Absorption and Glucose–Insulin Dynamics Model');
legend('Normal', 'Diabetic'); grid on;

subplot(3,1,2)
plot(results.normal.t, results.normal.G, 'b', 'LineWidth', 2); hold on;
plot(results.diabetic.t, results.diabetic.G, 'r--', 'LineWidth', 2);
ylabel('G(t) [mg/dl]');
legend('Normal', 'Diabetic'); grid on;

subplot(3,1,3)
plot(results.normal.t, results.normal.I, 'b', 'LineWidth', 2); hold on;
plot(results.diabetic.t, results.diabetic.I, 'r--', 'LineWidth', 2);
xlabel('Time (min)');
ylabel('I(t) [\muU/ml]');
legend('Normal', 'Diabetic'); grid on;


