clc, clear all, close all;

nx = 3; ny = 1; nu = 2;
%-------------------------------------------------------------------------
%% Continuous-time state-space model 

Ac = [-0.02876264232, 0, 0; 
       0.02876264232, -0.03618912464, 0.007426482315; 
       0, 0.007426482315, -0.01228825622];
  
Bc = [64.93506494, 0; 0, 0;0, 0];
Cc = [1, 0, 0;0, 1, 0; 0 0 1];
% Cc = [1, 0, 0];

u = .1e-3*[2 0]';


%% Discrete-time state-space model 
Ts = 1;
A = eye(3,3)+(Ts*Ac);
Bu = Ts*Bc;
C = Cc;

%% Direction of the uncertainties
Ew = [.05 0 0;
      0 .05 0;
      0 0 .05];
Ev = .1*[.08 0 0;
      0 .08 0;
      0 0 .08];
  
%% Initialization
x  = zeros(3,1);

cxio = zeros(3,1);
    Rxio = [1 0 0; 0 1 0;0 0 1];
cxZKF = zeros(3,1);
    RxZKF = [1 0 0; 0 1 0;0 0 1];

cxsm = zeros(3,1);
    Rxsm = [1 0 0; 0 1 0;0 0 1];
cxsmaLQR = zeros(3,1);
    RxsmaLQR = [1 0 0; 0 1 0;0 0 1];


%% LQR computation -> Que estem fent aqui? -> pre-calculem els guanys 

[L] = LQRio(nx,A,C,Ew,Ev);
[lambdaLQR] = LQRsm(nx,A,C,Ew,Ev);
% son la mateix funcio ?
%% estem fent una simulacio entre 1 i 2500
X = [x];
Y = [];
si = 1;
fi = 2500;


for k=si:fi
    k
   %% Uncertainties -> simulo uncertainties
    a = -.5; b = .5;
    w(:,k) = a + (b-a).*rand(3,1);
    v(:,k) = a + (b-a).*rand(3,1);
 
%% SYSTEM SIMULATION IN AcF OPERATION -> simulo com avança el sistema
    xx = A*x + Bu*u + Ew*w(:,k);
    X = [X, x];
    
    y = C*x + Ev*v(:,k);
    Y = [Y, y];


%%
% ------------------ Interval observer ------------------------------------
%% IOA using LQR gain -> IOA amb un guany fix, imagino q es el gunay estacionari del kalman
    % algun tipo de reduccio -> que fem amb aixo? 
    Rxio_red = reduction(Rxio,3);
    
    % cpio i Rpio del paper
    xeio=A*cxio + Bu*u;
    Reio=[A*Rxio, Ew];   
    
    % un cop actualiza cpio i Rpio les fem servir per actualizar cxxio i
    % rxxio
    cxxio = xeio +(L*(y-(C*xeio)));
    I=eye(length(xeio),length(xeio));
    Rxxio=[(I-(L*C))*Reio -L*Ev];
    
        Exio    = envbox(Rxio);
        x1maxio(k) = cxio(1) + abs(Exio (1,1));
        x1minio(k) = cxio(1) - abs(Exio (1,1));
        x2maxio(k) = cxio(2) + abs(Exio (2,2));
        x2minio(k) = cxio(2) - abs(Exio (2,2));
        x3maxio(k) = cxio(3) + abs(Exio (3,3));
        x3minio(k) = cxio(3) - abs(Exio (3,3));
% Output prediction       
    cyio = C*cxio;
    Ryio =[C*Rxio_red, Ev];
    
    Eyio    = envbox(Ryio);
        y1maxio(k) = cyio(1) + abs(Eyio (1,1));
        y1minio(k) = cyio(1) - abs(Eyio (1,1));
        y2maxio(k) = cyio(2) + abs(Eyio (2,2));
        y2minio(k) = cyio(2) - abs(Eyio (2,2));
        y3maxio(k) = cyio(3) + abs(Eyio (3,3));
        y3minio(k) = cyio(3) - abs(Eyio (3,3));
% Residual
    crio = y - cyio;
    Rrio = [-C*Rxio_red, -Ev];
    
    Erio    = envbox(Rrio);
        r1maxio(k) = crio(1) + abs(Erio (1,1));
        r1minio(k) = crio(1) - abs(Erio (1,1));
        r2maxio(k) = crio(2) + abs(Erio (2,2));
        r2minio(k) = crio(2) - abs(Erio (2,2));
        r3maxio(k) = crio(3) + abs(Erio (3,3));
        r3minio(k) = crio(3) - abs(Erio (3,3));  
    
    
%% IOA using ZKF gain -> IOA actualizant el guany segons el kalman filter 
    
  RxZKF_red = reduction(RxZKF,3);
    
    xeZKF=A*cxZKF + Bu*u;
    ReZKF=[A*RxZKF, Ew];
    
   
    L_ZKF= (A*(RxZKF*RxZKF')*C')*inv(C*(RxZKF*RxZKF')*C'+ (Ev*Ev'));
    
 
    cxxZKF = xeZKF +(L_ZKF*(y-(C*xeZKF)));
    I=eye(length(xeZKF),length(xeZKF));
    RxxZKF=[(I-(L_ZKF*C))*ReZKF -L_ZKF*Ev];
    
    
      ExZKF    = envbox(RxZKF);
        x1maxZKF(k) = cxZKF(1) + abs(ExZKF (1,1));
        x1minZKF(k) = cxZKF(1) - abs(ExZKF (1,1));
        x2maxZKF(k) = cxZKF(2) + abs(ExZKF (2,2));
        x2minZKF(k) = cxZKF(2) - abs(ExZKF (2,2));
        x3maxZKF(k) = cxZKF(3) + abs(ExZKF (3,3));
        x3minZKF(k) = cxZKF(3) - abs(ExZKF (3,3));
% Output prediction       
    cyZKF = C*cxZKF;
    RyZKF =[C*RxZKF_red, Ev];
    
    EyZKF    = envbox(RyZKF);
        y1maxZKF(k) = cyZKF(1) + abs(EyZKF (1,1));
        y1minZKF(k) = cyZKF(1) - abs(EyZKF (1,1));
        y2maxZKF(k) = cyZKF(2) + abs(EyZKF (2,2));
        y2minZKF(k) = cyZKF(2) - abs(EyZKF (2,2));
        y3maxZKF(k) = cyZKF(3) + abs(EyZKF (3,3));
        y3minZKF(k) = cyZKF(3) - abs(EyZKF (3,3));
% Residual

    crZKF = y - cyZKF;
    RrZKF = [-C*RxZKF_red, -Ev];
    
    ErZKF    = envbox(RrZKF);

        r1maxZKF(k) = crZKF(1) + abs(ErZKF (1,1));
        r1minZKF(k) = crZKF(1) - abs(ErZKF (1,1));
        r2maxZKF(k) = crZKF(2) + abs(ErZKF (2,2));
        r2minZKF(k) = crZKF(2) - abs(ErZKF (2,2));
        r3maxZKF(k) = crZKF(3) + abs(ErZKF (3,3));
        r3minZKF(k) = crZKF(3) - abs(ErZKF (3,3));
   
    
%%
% ------------------ Set-membership approach ------------------------------
%% SMA using LQR lambda -> SMA amb lambda igual al guany estarionari de kalman     
    
 RxsmaLQR_red = reduction(RxsmaLQR,3);
    
    xesmaLQR=A*cxsmaLQR + Bu*u;
    ResmaLQR=[A*RxsmaLQR, Ew];
   
    
    cxxsmaLQR = xesmaLQR +(lambdaLQR*(y-(C*xesmaLQR)));
    I=eye(length(xesmaLQR),length(xesmaLQR));
    RxxsmaLQR=[(I-(lambdaLQR*C))*ResmaLQR -lambdaLQR*Ev];
    
       
      ExsmaLQR    = envbox(RxsmaLQR);
        x1maxsmaLQR(k) = cxsmaLQR(1) + abs(ExsmaLQR (1,1));
        x1minsmaLQR(k) = cxsmaLQR(1) - abs(ExsmaLQR (1,1));
        x2maxsmaLQR(k) = cxsmaLQR(2) + abs(ExsmaLQR (2,2));
        x2minsmaLQR(k) = cxsmaLQR(2) - abs(ExsmaLQR (2,2));
        x3maxsmaLQR(k) = cxsmaLQR(3) + abs(ExsmaLQR (3,3));
        x3minsmaLQR(k) = cxsmaLQR(3) - abs(ExsmaLQR (3,3));
% Output prediction       
    cysmaLQR = C*cxsmaLQR;
    RysmaLQR =[C*RxsmaLQR_red, Ev];
    
    EysmaLQR    = envbox(RysmaLQR);
        y1maxsmaLQR(k) = cysmaLQR(1) + abs(EysmaLQR (1,1));
        y1minsmaLQR(k) = cysmaLQR(1) - abs(EysmaLQR (1,1));
        y2maxsmaLQR(k) = cysmaLQR(2) + abs(EysmaLQR (2,2));
        y2minsmaLQR(k) = cysmaLQR(2) - abs(EysmaLQR (2,2));
        y3maxsmaLQR(k) = cysmaLQR(3) + abs(EysmaLQR (3,3));
        y3minsmaLQR(k) = cysmaLQR(3) - abs(EysmaLQR (3,3));
% Residual

    crsmaLQR = y - cysmaLQR;
    RrsmaLQR = [-C*RxsmaLQR_red, -Ev];
    
    ErsmaLQR    = envbox(RrsmaLQR);


        r1maxsmaLQR(k) = crsmaLQR(1) + abs(ErsmaLQR (1,1));
        r1minsmaLQR(k) = crsmaLQR(1) - abs(ErsmaLQR (1,1));
        r2maxsmaLQR(k) = crsmaLQR(2) + abs(ErsmaLQR (2,2));
        r2minsmaLQR(k) = crsmaLQR(2) - abs(ErsmaLQR (2,2));
        r3maxsmaLQR(k) = crsmaLQR(3) + abs(ErsmaLQR (3,3));
        r3minsmaLQR(k) = crsmaLQR(3) - abs(ErsmaLQR (3,3));

   
 %% SMA using Alamo lambda -> SMA amb la lambda que ens marca el kalman, alamo lambda ?
 
 Rxsm_red = reduction(Rxsm,3);
    
    xesm=A*cxsm + Bu*u;
    Resm=[A*Rxsm, Ew];
    
    lambda = (A*(Rxsm*Rxsm')*C')*inv(C*(Rxsm*Rxsm')*C'+ (Ev*Ev'));
   
    cxxsm = xesm +(lambda*(y-(C*xesm)));
    I=eye(length(xesm),length(xesm));
    Rxxsm=[(I-(lambda*C))*Resm -lambda*Ev];
    
      Exsm    = envbox(Rxsm);
        x1maxsm(k) = cxsm(1) + abs(Exsm (1,1));
        x1minsm(k) = cxsm(1) - abs(Exsm (1,1));
        x2maxsm(k) = cxsm(2) + abs(Exsm (2,2));
        x2minsm(k) = cxsm(2) - abs(Exsm (2,2));
        x3maxsm(k) = cxsm(3) + abs(Exsm (3,3));
        x3minsm(k) = cxsm(3) - abs(Exsm (3,3));
% Output prediction       
    cysm = C*cxsm;
    Rysm =[C*Rxsm_red, Ev];
    
    Eysm    = envbox(Rysm);
        y1maxsm(k) = cysm(1) + abs(Eysm (1,1));
        y1minsm(k) = cysm(1) - abs(Eysm (1,1));
        y2maxsm(k) = cysm(2) + abs(Eysm (2,2));
        y2minsm(k) = cysm(2) - abs(Eysm (2,2));
        y3maxsm(k) = cysm(3) + abs(Eysm (3,3));
        y3minsm(k) = cysm(3) - abs(Eysm (3,3));
% Residual
    crsm = y - cysm;
    Rrsm = [-C*Rxsm_red, -Ev];
    
    Ersm    = envbox(Rrsm);
        r1maxsm(k) = crsm(1) + abs(Ersm (1,1));
        r1minsm(k) = crsm(1) - abs(Ersm (1,1));
        r2maxsm(k) = crsm(2) + abs(Ersm (2,2));
        r2minsm(k) = crsm(2) - abs(Ersm (2,2));
        r3maxsm(k) = crsm(3) + abs(Ersm (3,3));
        r3minsm(k) = crsm(3) - abs(Ersm (3,3));
           
    
%%
    x  = xx;
    cxio = cxxio;
    Rxio = Rxxio;
    
    cxZKF = cxxZKF;
    RxZKF = RxxZKF;  
    
    cxsm = cxxsm;
    Rxsm = Rxxsm;
    
    cxsmaLQR = cxxsmaLQR;
    RxsmaLQR = RxxsmaLQR;
     
    
end


%% Classical gain
figure, set(gcf,'DefaultLineLineWidth',2.5);

    subplot(3,1,1)
    hold on
    ylabel('x_1')
    
    Nomx = plot(X(1,si:fi),'k');
    
    
    % IOA
 
    
    StESZKF = plot(x1maxZKF(1,si:fi),'r');
    plot(x1minZKF(1,si:fi),'r');

    
    % SMA
    StESsm = plot(x1maxsm(1,si:fi),'b--');
    plot(x1minsm(1,si:fi),'b--');
    

    

    legend([Nomx, StESZKF, StESsm],'x','IOA (Christophe)', 'SMA (Alamo)');
    %
    subplot(3,1,2)
    hold on
    ylabel('x_2')
    
    plot(X(2,si:fi),'k');
    
    % IOA
 
    
    plot(x2maxZKF(1,si:fi),'r');
    plot(x2minZKF(1,si:fi),'r');
    
    % SMA
    plot(x2maxsm(1,si:fi),'b--');
    plot(x2minsm(1,si:fi),'b--');
    

    subplot(3,1,3)
    hold on
    ylabel('x_3')
    xlabel('Time step')
    
    plot(X(3,si:fi),'k');
    
    % IOA
 
    
    plot(x3maxZKF(1,si:fi),'r');
    plot(x3minZKF(1,si:fi),'r');
    
    % SMA
    plot(x3maxsm(1,si:fi),'b--');
    plot(x3minsm(1,si:fi),'b--');
    
 
    

    
    
    
    
    
 %% LQR gain
figure, set(gcf,'DefaultLineLineWidth',2.5);

    subplot(3,1,1)
    hold on
    ylabel('x_1')
    
    Nomx = plot(X(1,si:fi),'k');
    
    
    % IOA
    StESio = plot(x1maxio(1,si:fi),'r');
    plot(x1minio(1,si:fi),'r');
    
   
    % SMA
  
 
    StESsmLQR = plot(x1maxsmaLQR(1,si:fi),'b--');
    plot(x1minsmaLQR(1,si:fi),'b--');
    

    legend([Nomx, StESio, StESsmLQR],'x','IOA (LQR)','SMA (LQR)');
    %
    subplot(3,1,2)
    hold on
    ylabel('x_2')
    
    plot(X(2,si:fi),'k');
    
    % IOA
    plot(x2maxio(1,si:fi),'r');
    plot(x2minio(1,si:fi),'r');
    
    
    % SMA
    
    plot(x2maxsmaLQR(1,si:fi),'b--');
    plot(x2minsmaLQR(1,si:fi),'b--');
    
    
    
    subplot(3,1,3)
    hold on
    ylabel('x_3')
    xlabel('Time step')
    
    plot(X(3,si:fi),'k');
    
    % IOA
    plot(x3maxio(1,si:fi),'r');
    plot(x3minio(1,si:fi),'r');

    
    % SMA

    
    plot(x3maxsmaLQR(1,si:fi),'b--');
    plot(x3minsmaLQR(1,si:fi),'b--');
       
    
    
 
    
    
    
    
%%    
%% All together
figure, set(gcf,'DefaultLineLineWidth',2.5);

    subplot(3,1,1)
    hold on
    ylabel('x_1')
    
    Nomx = plot(X(1,si:fi),'k');
    
 % LQR   
    % IOA
    StESio = plot(x1maxio(1,si:fi),'r');
    plot(x1minio(1,si:fi),'r');
    % SMA
    StESsmLQR = plot(x1maxsmaLQR(1,si:fi),'b--');
    plot(x1minsmaLQR(1,si:fi),'b--');
    
 % Classical   
    % IOA
    StESZKF = plot(x1maxZKF(1,si:fi),'r');
    plot(x1minZKF(1,si:fi),'r');
    % SMA
    StESsm = plot(x1maxsm(1,si:fi),'b--');
    plot(x1minsm(1,si:fi),'b--');  
 
 
 
 
    
    
%    
legend([Nomx, StESio, StESsmLQR, StESZKF, StESsm],'x','IOA (Proposed approach)','SMA (Proposed approach)', 'IOA (Classical approach)','SMA (Classical approach)');
    %%
    subplot(3,1,2)
    hold on
    ylabel('x_2')
    
    plot(X(2,si:fi),'k');
    
 % LQR
    
    % IOA
    plot(x2maxio(1,si:fi),'r');
    plot(x2minio(1,si:fi),'r');
    % SMA
    plot(x2maxsmaLQR(1,si:fi),'b--');
    plot(x2minsmaLQR(1,si:fi),'b--');
    
    
% Classical   
    % IOA
    plot(x2maxZKF(1,si:fi),'r');
    plot(x2minZKF(1,si:fi),'r');
    % SMA
    plot(x2maxsm(1,si:fi),'b--');
    plot(x2minsm(1,si:fi),'b--');


%%
    subplot(3,1,3)
    hold on
    ylabel('x_3')
    xlabel('Time step')
    
    plot(X(3,si:fi),'k');
    
% LQR  
    % IOA
    plot(x3maxio(1,si:fi),'r');
    plot(x3minio(1,si:fi),'r');
    % SMA
    plot(x3maxsmaLQR(1,si:fi),'b--');
    plot(x3minsmaLQR(1,si:fi),'b--');
       
% Classical   
    % IOA
    plot(x3maxZKF(1,si:fi),'r');
    plot(x3minZKF(1,si:fi),'r');
    % SMA
    plot(x3maxsm(1,si:fi),'b--');
    plot(x3minsm(1,si:fi),'b--');
    
    
    
    
    