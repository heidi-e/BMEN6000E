%% Heidi Eren — Automatic PID tuning for Glucose–Insulin model
clear; clc; close all;

%% Objective function (nested simulation)
costFun = @(K) glucoseCost(K(1), K(2), K(3));   % [Kp, Ki, Kd]

%% Initial guess
K0 = [0.3, 0.005, 0.05];  % start with stronger action


%% Optimize using Nelder–Mead (simplex)
opts = optimset('Display','iter','TolX',1e-6,'TolFun',1e-6,'MaxIter',200);

[K_opt, J_opt] = fminsearch(costFun, K0, opts);

fprintf('\nOptimal gains:\nKp = %.4f\nKi = %.4f\nKd = %.4f\nCost = %.4f\n', ...
        K_opt(1), K_opt(2), K_opt(3), J_opt);

%% Run final simulation with optimal gains
[~, t, G, I, Q] = glucoseCost(K_opt(1), K_opt(2), K_opt(3));

figure('Color','w');
subplot(3,1,1)
plot(t,G,'r','LineWidth',1.8); hold on
yline(120,'--k','Target');
ylabel('G [mg/dL]'); legend('Glucose','Target'); grid on
title(sprintf('Optimized PID: Kp=%.3f, Ki=%.4f, Kd=%.3f',K_opt))
subplot(3,1,2)
plot(t,I,'b','LineWidth',1.8); ylabel('I [\muU/mL]'); grid on
subplot(3,1,3)
plot(t,Q,'m','LineWidth',1.8); ylabel('Q [mg]'); xlabel('Time [min]'); grid on


%% --- COST FUNCTION DEFINITION ---
function [J, t, G, I, Q] = glucoseCost(Kp, Ki, Kd)

    %% Simulation setup
    dt = 0.1; t_end = 800; t = 0:dt:t_end; N = numel(t);
    Q = zeros(1,N); G = zeros(1,N); I = zeros(1,N);
    Q(1)=0; G(1)=163; I(1)=11.26;
    G_set = 120;

    % model params (diabetic)
    beta=10; eta=4.641; gam=25;
    R0=2.5; E=2.5e-3; S=1.14e-3;
    kq=0.026; Imax=0.93; alpha=1e4; ki=0.06;

    % meal
    mealStart=0; mealDuration=15; mealSize=15000;
    D=@(time) (mealSize/mealDuration)*(time>=mealStart & time<mealStart+mealDuration);

    % bolus
    bolusThreshold=160; bolusDose=0.5; bolusDuration=5;
    bolusTimer=0; addIn_bolus=0;

    % PID states
    g_prev=0; sumG=0;
    addIn_PID=zeros(1,N);

    for k=2:N
        time=t(k);

        % meal
        Dk=D(time);

        % bolus logic
        if (G(k-1)>bolusThreshold && bolusTimer<=0)
            addIn_bolus=bolusDose;
            bolusTimer=bolusDuration;
        end
        if (bolusTimer>0)
            bolusTimer=bolusTimer-dt;
        else
            addIn_bolus=0;
        end

        % PID control
        g_error=G_set-G(k-1);
        sumG=sumG+g_error*dt;
        g_deriv=(g_error-g_prev)/dt;
        g_prev=g_error;
        addIn_PID(k)=Kp*g_error + Ki*sumG + Kd*g_deriv;
        addIn_PID(k)=max(addIn_PID(k),0);
        addIn_total=addIn_PID(k)+addIn_bolus;

        % equations
        dQ=-(beta*Q(k-1)+eta*Dk)/(gam^2+Q(k-1)^2);
        dG=R0-(E+S*I(k-1))*G(k-1)+kq*Q(k-1);
        dI=Imax*(G(k-1)^2/(alpha+G(k-1)^2)) - ki*I(k-1) + addIn_total;

        Q(k)=Q(k-1)+dQ*dt;
        G(k)=G(k-1)+dG*dt;
        I(k)=I(k-1)+dI*dt;
    end

    %% compute metrics
    overshoot = max(G)-G_set;
    ss_err = mean(G(end-100:end))-G_set;
    osc_penalty = std(G(end-300:end)); % small if steady

    % weighted cost (you can tune weights)
    J = 0.4*(overshoot)^2 + 0.4*(ss_err)^2 + 0.2*(osc_penalty)^2;

end
