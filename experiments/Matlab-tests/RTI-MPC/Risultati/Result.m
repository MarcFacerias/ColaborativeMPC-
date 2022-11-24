close all; clear; clc;

% Plot dei risultati ottenuti

SchedVars_Limits = [  3.6  8.0; %3.6 15
                     -0.8  0.8;
                     -1.4  1.4;
                     -0.267 0.297;
                     5.0  11.0    ]; %-2 13
n = 3;

switch n
    case 1
        load Traiettoria1.mat
        
    case 2
        load Traiettoria2.mat
        
    case 3
        load Traiettoria3.mat
end

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

figure(3) % Plot del tempo computazionale di risoluzione dell'OP
plot(Time(1:i),ET_MPC,'LineWidth',1.5);
grid on
ylabel('Tempo Elaborazione / Iterazioni [s]');
xlabel('Time [s]');
xlim([min(Time) max(Time)]);

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