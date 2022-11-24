%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Salvatore Verso
% Date: 18/01/2021
% Real Time Iteration - Model Predictive Control
% L'obiettivo è quello di sviluppare una legge di controllo real-time
% utilizzando le fasi di preparazione e di feedback
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% TODO list:
% - Pulire il documento e aggiustare i plot dei dati aggiungendo i vincoli
% - Effettuare il plot dei dati nello spazio (X,Y) posizione del veicolo
% - Reference Big L shape ha tutto cio che serve
% - Introdurre nella funzione del modello la parte ralativa ad X,Y e Theta
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all; clear; clc;
format short

disp('Possibili traiettorie di Riferimento:')
disp('1 - UPCDriverless_Data_for_Learning');
disp('2 - References_Driverless_Track');
disp('3 - References_Big_L_Shape_Track')
prompt = 'Scegliere la Traiettoria di Rifeirmento da considerare:\n';
n = input(prompt);


%% Dati di Riferimento:

switch n
    case 1
        load UPCDriverless_Data_for_Learning
        ini_index    = 1000; 
        fin          = 10000;       
        Vx_ref       = vehicle_states(ini_index:fin,1); 
        Vy_ref       = vehicle_states(ini_index:fin,2);
        Omega_ref    = vehicle_states(ini_index:fin,3);
        vx           = Vx_ref(1);
        omega        = Omega_ref(1);
        vy           = Vy_ref(1);
        Steer_ref    = delta_new(ini_index:fin);
        Accel_ref    = accel_new(ini_index:fin);
        steer        = Steer_ref(1);
        accel        = Accel_ref(1);
        R_state      = [vx; vy; omega];  % Real State
        S_state      = [vx; vy; omega];  % Simulation state
        N_element    = length(Vx_ref);
        
    case 2
        load Racing_References
        ini_index    = 800;
        fin          = 10000;
        Vx_ref       = Vx(ini_index:fin);
        Vy_ref       = Vy(ini_index:fin);
        Omega_ref    = Wz(ini_index:fin);
        vx           = Vx_ref(1);
        omega        = Omega_ref(1);
        vy           = Vy_ref(1);
        Steer_ref    = delta(ini_index:fin);
        Accel_ref    = accel(ini_index:fin);
        steer        = Steer_ref(1);
        accel        = Accel_ref(1);
        R_state      = [vx; vy; omega];  % Real State
        S_state      = [vx; vy; omega];  % Simulation state
        N_element    = length(Vx_ref);
        
    case 3
        load References_Big_L_Shape_Track
        ini_index    = 900;
        fin          = 10000;
        Vx_ref       = Vx(ini_index:fin);
        Vy_ref       = Vy(ini_index:fin);
        Omega_ref    = Wz(ini_index:fin);
        vx           = Vx_ref(1);
        omega        = Omega_ref(1);
        vy           = Vy_ref(1);
        Steer_ref    = delta(ini_index:fin);
        Accel_ref    = accel(ini_index:fin);
        steer        = Steer_ref(1);
        accel        = Accel_ref(1);
        R_state      = [vx; vy; omega];  % Real State
        S_state      = [vx; vy; omega];  % Simulation state
        N_element    = length(Vx_ref);
        X_ref        = X(ini_index:fin);
        Y_ref        = Y(ini_index:fin);
        THETA_ref    = Theta(ini_index:fin);
        x_p          = X_ref(1);
        y_p          = Y_ref(1);
        theta_p      = THETA_ref(1);
        load reference.mat
end


% Scelgo il passo di campionamento è l'orizzonte di simulazione
Ts      = 0.033; 
Tc_inl  = 0.001;
N       = Ts/Tc_inl;
Time    = 0:Tc_inl:(N_element-1)*Tc_inl;

%% Disturbance vector definition:

% Slope bounded in [-0.15 0.15]
% road_slope > 0 -> road goes up

DISTURBANCE = zeros(2,fin);
max_lat_wind = 35;
for i = 1:fin
    if i > 100 && i < 150
        road_slope = 0.2; 
        lateral_wind = 0;        
    elseif i > 160 && i < 210
        road_slope = 0.1*sin(i/3.177);
        lateral_wind = 0;        
    elseif i > 210 && i < 310
        road_slope = 0;
        lateral_wind = max_lat_wind*(i-210)/100;   
        %lateral_wind = 0.60*(i-250); 
    elseif i > 340 && i < 600
        road_slope = 0;
        lateral_wind = max_lat_wind;        
    elseif i > 650 && i < 900
        road_slope = 0.2;
        lateral_wind = max_lat_wind;         
    else
        road_slope = 0.0;    
        lateral_wind = 0.0;    
    end
    DISTURBANCE(:,i) = [road_slope lateral_wind]';
end

%% States limits:

SchedVars_Limits = [  3.6  8.0; %3.6 15
                     -0.8  0.8;
                     -1.4  1.4;
                     -0.267 0.267;
                     5.0  11.0    ]; %-2 13
                 
%% RTI-MPC Reference design:

nu = 2;
nx = 3;
Hp = 4;

RTI_MPC_controller = RTI_MPC_fnc( Hp, SchedVars_Limits, nx, nu );

XX          = zeros(nx,1,Hp); % Stato utilizzato per QP
XX(:,:,1)   = S_state;

%% INIZIALIZZAZIONE

if n == 3
    % Costruzione della Strada
    Pose       = zeros(2,length(X_ref));
    NewPose    = zeros(2,length(X_ref));
    RightLine  = zeros(2,length(X_ref));
    LeftLine   = zeros(2,length(X_ref));
    CenterLine = zeros(2,length(X_ref));

    for ii=1:length(X_ref)
        Pose(:,ii)        = [X(ii); Y(ii)];
        R                 = [cos(THETA_ref(ii)) sin(THETA_ref(ii)); -sin(THETA_ref(ii)) cos(THETA_ref(ii))];
        NewPose(:,ii)     = R*Pose(1:2,ii);
        RightLine(:,ii)   = NewPose(1:2,ii) + [0; 4.5];
        LeftLine(:,ii)    = NewPose(1:2,ii) - [0; 1.5];
        CenterLine(:,ii)  = NewPose(1:2,ii) + [0; 1.5];
        RightLine(:,ii)   = inv(R)*RightLine(1:2,ii);
        LeftLine(:,ii)    = inv(R)*LeftLine(1:2,ii);
        CenterLine(:,ii)    = inv(R)*CenterLine(1:2,ii);
    end
end

% Riferimento utilizzato nell'OP
rif_x = zeros(nx,1,Hp+1);
rif_u = zeros(nu,1,Hp);

% Matrici del Sistema e dei Vincoli
A_vec = zeros(nx,nx,Hp);
B_vec = zeros(nx,nu,Hp);
C_vec = zeros(nx*2+nu*2,nx,Hp);
D_vec = zeros(nx*2+nu*2,nu,Hp);

% Condizione Iniziale Ingresso
u_inl           = [steer; accel];
steer_inl       = u_inl(1);

% Variabili x_guess e u_guess
x_guess = zeros(nx,1,Hp+1);
u_guess = zeros(nu,1,Hp);

% Contatore
counter_inl = 1;

% Ciclo Interno
error_State_inl = zeros(fin*N,3);
INPUTS_inl      = zeros(fin*N,2);

% Variabili che mi permettono di  effettuare il plot dei risultati
x_hat = zeros(nx,fin-ini_index);
u_hat = zeros(nu,fin-ini_index);
x_hat(:,1) = S_state;
error_state = zeros(nx,fin-ini_index);
error_state(:,1) = S_state - [Vx_ref(1); Vy_ref(1); Omega_ref(1)];

% Variabili utilizzate all'interno del problema di ottimizzazione
dx = zeros(nx,1,Hp+1);
du = zeros(nu,1,Hp);
r = zeros(nx,1,Hp);
h = zeros(nx*2+nu*2,1);

% Variabili utilizzate per la procedura di shift
x_i_shift = zeros(nx,1,Hp+1);
u_i_shift = zeros(nu,1,Hp);

% Errore Massimo in funzione del disturbo
vx_max_e = 9.8*Ts* (max(abs(DISTURBANCE(1,:))) + max(abs(DISTURBANCE(1,:)))) ;
vy_max_e = Ts * (0.5 * 1.225 * 1.5 * (max(abs(DISTURBANCE(2,:))) + 0*max(abs(DISTURBANCE(2,:))))^2) / 196 ;
vw_max_e = Ts * (0.5 * 1.225 * 2 * (max(abs(DISTURBANCE(2,:))) + 0*max(abs(DISTURBANCE(2,:))))^2) * (0.902-0.638) / 93 ;

if n == 3
    % Variabili per il plot nello spazio
    X = zeros(1,fin-ini_index);
    Y = zeros(1,fin-ini_index);
    THETA = zeros(1,fin-ini_index);
    X(1,1) = x_p;
    Y(1,1) = y_p;
    THETA(1,1) = theta_p;
end

% Per la prima iterazione non avendo ha disposizione il risultato del
% problema di ottimizzazione sfrutto la conoscenza della traiettoria di
% riferimento:

for j = 1:Hp
    %LPV formulation 
    [A_vec(:,:,j), B_vec(:,:,j)] = Discrete_AB_Comp( Steer_ref(j), Vx_ref(j), Vy_ref(j), Ts, 'Varianttt B');
    % compution of tire forces and interactions
    r(:,1,j) = F_model([Vx_ref(j); Vy_ref(j); Omega_ref(j)],[Steer_ref(j); Accel_ref(j)],Ts) - [Vx_ref(j+1); Vy_ref(j+1); Omega_ref(j+1)];
    % initial guess, at it is the first iteration we use the Uref and
    % Xref
    x_guess(:,1,j) = [Vx_ref(j); Vy_ref(j); Omega_ref(j)];
    u_guess(:,1,j) = [Steer_ref(j); Accel_ref(j)];
    % more system matrices 
    C_vec(:,:,j) = [-1 0 0; 1 0 0; 0 -1 0; 0 1 0; 0 0 -1; 0 0 1; 0 0 0; 0 0 0; 0 0 0; 0 0 0];
    D_vec(:,:,j) = [0 0; 0 0; 0 0; 0 0; 0 0; 0 0; -1 0; 1 0; 0 -1; 0 1];
    % update shceduling variables, adjusting them to the guess ?
    h(:,1,j) = H_model(SchedVars_Limits,[Vx_ref(j); Vy_ref(j); Omega_ref(j)],[Steer_ref(j); Accel_ref(j)],nx,nu);                                   
end

x_guess(:,1,Hp+1) = [Vx_ref(Hp+1); Vy_ref(Hp+1); Omega_ref(Hp+1)];

%% ALGORITMO

for i = 1:(fin-ini_index)-Hp-1
    %% FEEDBACK PHASE
    
    % Definisco il riferimento da usare nell'OP
    % Update references
    for j=i:i+Hp-1
        rif_x(:,1,j-i+1) = [Vx_ref(j); Vy_ref(j); Omega_ref(j)];
        rif_u(:,1,j-i+1) = [Steer_ref(j); Accel_ref(j)];
    end
    rif_x(:,1,Hp+1) = [Vx_ref(i+Hp); Vy_ref(i+Hp); Omega_ref(i+Hp)];
    
    % Risolvo il Problema
    inputs = {S_state, x_guess(:,1,1:Hp), u_guess, rif_x(:,1,1:Hp), rif_u, r, h, A_vec, B_vec, C_vec, D_vec};
    tic
    [solutions,diagnostics] = RTI_MPC_controller{inputs};
    ET_MPC(i) = toc;
    
    if diagnostics == -1
        Uopt = [0;0];
    elseif diagnostics == 0
        UU   = solutions{1}; 
        XX   = solutions{2};
        CF(i)= solutions{3};
    else         
        error('Nan result.')
    end
    
    x_i = x_guess + XX;
    u_i = u_guess + UU;
   
    % Aggiornol'ingresso che utilizzero per il plot dei risultati
    u_hat(:,i) = u_i(:,1);
   
    %% SIMULAZIONE DEL SISTEMA NON LINEARE
    
    for k = 0:N-1
        
%       error_State_inl(counter_inl,:) = XX(1:3,1,2) - R_state;
%       vxe_integral = vxe_integral + error_State_inl(counter_inl,1)*Ts_inl;
%       we_integral  = we_integral  + error_State_inl(counter_inl,3)*Ts_inl;
        u_inl(:) = u_i(:,1);
        steer_inl = u_inl(1); 
        accel_inl = u_inl(2);
        INPUTS_inl(counter_inl,:) = [ steer_inl accel_inl ];
        T = Tc_inl*k : Tc_inl/3 : Tc_inl*k+Tc_inl;
        
            %% SISTEMA NON LINEARE (Evoluzione del Sistema sotto l'azione di Controllo)
            
            [ T, x ] = ode45(@( t,x ) nonlinear_DriverlessUPC_model...
                ( t, x, [ steer_inl; accel_inl; DISTURBANCE(1,i); DISTURBANCE(2,i) ]), T, R_state);

            R_state      = [x(end,1); x(end,2); x(end,3)];

            vx           = x(end,1);
            vy           = x(end,2);
            omega        = x(end,3);
             
    end
    
    S_state = R_state ;
    
    % Aggiorno lo stato che utilizzero per il plot dei risultati
    
    x_hat(:,i+1) = S_state;
    error_state(:,i+1) = S_state - [Vx_ref(i+1); Vy_ref(i+1); Omega_ref(i+1)];
    
    %% PREPARATION PHASE
    
    % We initialise the shift as a funtion of the previous guess 
    x_i_shift = x_i(:,1,2:Hp+1);
    u_i_shift = u_i(:,1,2:Hp);
    u_i_shift(:,1,Hp) = u_i(:,1,Hp);
    
    % maps and adds a predition of the state in Hp 
    [x_guess, u_guess] = Shift_Procedure(x_i_shift, u_i_shift, Hp, nx, nu,Ts);

    for j=1:Hp
            [ A_vec(:,:,j), B_vec(:,:,j) ] = Discrete_AB_Comp( u_guess(1,1,j), x_guess(1,1,j), x_guess(2,1,j), Ts, 'Varianttt B' );
            r(:,1,j) = F_model(x_guess(:,1,j),u_guess(:,1,j),Ts) - x_guess(:,1,j+1);
            h(:,1,j) = H_model(SchedVars_Limits,x_guess(:,1,j),u_guess(:,1,j),nx,nu);
    end

    i
end

if n == 3
    load('/Users/salvatore/Library/Mobile Documents/com~apple~CloudDocs/Università/Tesi/Matlab/RTI-MPC/Risultati/reference_track.mat')
    X_ref = X;
    Y_ref = Y;
    VX = [Time(1:i).' x_hat(1,1:i).'];
    VY = [Time(1:i).' x_hat(2,1:i).'];
    OMEGA = [Time(1:i).' x_hat(3,1:i).'];

    VX_ref = [Time(1:i).' Vx_ref(1,1:i).'];
    VY_ref = [Time(1:i).' Vy_ref(1,1:i).'];
    OMEGA_ref = [Time(1:i).' Omega_ref(1,1:i).'];

    result = sim('kinematic_model');
    X = result.X;
    Y = result.Y;
end
%% PLOT DEI RISULTATI

figure(1) % Plot degli Stati
subplot(3,1,1)
plot(Time(1:i),x_hat(1,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),Vx_ref(1:i),'g--','LineWidth',1.5);
xlim([min(Time) max(Time)]);
ylim([SchedVars_Limits(1,1)-1 SchedVars_Limits(1,2)+1]);
plot(Time(1:i),ones(1,i).*SchedVars_Limits(1,1),'r--');
plot(Time(1:i),ones(1,i).*SchedVars_Limits(1,2),'r--');
grid on;
legend('RTI-MPC','Reference','Upper Bound','Lower Bound');
ylabel('V_x [m/s]');
subplot(3,1,2)
plot(Time(1:i),x_hat(2,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),Vy_ref(1:i),'g--','LineWidth',1.5);
plot(Time(1:i),ones(1,i).*SchedVars_Limits(2,1),'r--');
plot(Time(1:i),ones(1,i).*SchedVars_Limits(2,2),'r--');
xlim([min(Time) max(Time)]);
ylim([SchedVars_Limits(2,1)-1 SchedVars_Limits(2,2)+1]);
grid on;
ylabel('V_y [m/s]');
subplot(3,1,3)
plot(Time(1:i),x_hat(3,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),Omega_ref(1:i),'g--','LineWidth',1.5);
plot(Time(1:i),ones(1,i).*SchedVars_Limits(3,1),'r--');
plot(Time(1:i),ones(1,i).*SchedVars_Limits(3,2),'r--');
xlim([min(Time) max(Time)]);
ylim([SchedVars_Limits(3,1)-1 SchedVars_Limits(3,2)+1]);
grid on;
ylabel('\omega [rad/s]');
xlabel('Time [s]');

figure(2) % Plot dell'Ingresso
subplot(2,1,1)
plot(Time(1:i),u_hat(1,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),ones(1,i).*SchedVars_Limits(4,1),'r--');
plot(Time(1:i),ones(1,i).*SchedVars_Limits(4,2),'r--');
legend('RTI-MPC','Lower Bound','Upper Bound');
xlim([min(Time) max(Time)]);
ylim([SchedVars_Limits(4,1)-1 SchedVars_Limits(4,2)+1]);
grid on;
ylabel('Steering [rad]');
subplot(2,1,2)
plot(Time(1:i),u_hat(2,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),ones(1,i).*SchedVars_Limits(5,1),'r--');
plot(Time(1:i),ones(1,i).*SchedVars_Limits(5,2),'r--');
xlim([min(Time) max(Time)]);
ylim([SchedVars_Limits(5,1)-0.5 SchedVars_Limits(5,2)+0.5]);
grid on;
ylabel('Accel. [m/s^2]');
xlabel('Time');

figure(3) % Plot degli errori
subplot(3,1,1)
plot(Time(1:i),error_state(1,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),ones(1,i).*vx_max_e,'r--');
plot(Time(1:i),-ones(1,i).*vx_max_e,'r--');
xlim([min(Time) max(Time)]);
ylim([-vx_max_e-0.08 vx_max_e+0.02]);
grid on;
ylabel('e_{V_x}');
subplot(3,1,2)
plot(Time(1:i),error_state(2,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),ones(1,i).*vy_max_e,'r--');
plot(Time(1:i),-ones(1,i).*vy_max_e,'r--');
xlim([min(Time) max(Time)]);
ylim([-vy_max_e-0.02 vy_max_e+0.02]);
grid on;
ylabel('e_{V_y}');
subplot(3,1,3)
plot(Time(1:i),error_state(3,1:i),'LineWidth',1.5);
hold on
plot(Time(1:i),ones(1,i).*vw_max_e,'r--');
plot(Time(1:i),-ones(1,i).*vw_max_e,'r--');
xlim([min(Time) max(Time)]);
ylim([-vw_max_e-0.05 vw_max_e+0.02]);
grid on;
ylabel('e_\omega');
xlabel('Time [s]');

figure(4) % Plot del tempo computazionale di risoluzione dell'OP
plot(Time(1:i),ET_MPC,'LineWidth',1.5);
grid on
ylabel('Tempo Elaborazione / Iterazioni [s]');
xlabel('Time [s]');
xlim([min(Time) max(Time)]);

if n == 3
    figure(5) % Plot nel piano X e Y
    p1 = plot(LeftLine(1,1:end),LeftLine(2,1:end),'k','LineWidth',1);
    hold on
    p2 = plot(RightLine(1,1:end), RightLine(2,1:end),'k','LineWidth',1);
    p3 = plot(X_ref(1:8800),Y_ref(1:8800), '--r','LineWidth',1);
    p4 = plot(X,Y,'g','LineWidth',1.5);
    grid on
    set(get(get(p1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    set(get(get(p2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    legend('Reference','Trajectory');
    xlabel('X [m]')
    ylabel('Y [m]')
end