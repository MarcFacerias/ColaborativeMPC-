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


%% Obsrever gain
% L_poles = [0.3;0.9;0.2];
% L = place(A',C', L_poles).';
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



%%
X = [x];
Y = [];

%% LQR gain for IOA
[L] = LQRio(nx,A,C,Ew,Ev);


si = 1;
fi = 2500;
for k=si:fi
    k
   %% Uncertainties
    a = -.5; b = .5;
    w(:,k) = a + (b-a).*rand(3,1);
    v(:,k) = a + (b-a).*rand(3,1);
 
%% SYSTEM SIMULATION IN AcF OPERATION 
    xx = A*x + Bu*u + Ew*w(:,k);
    X = [X, x];
    
    y = C*x + Ev*v(:,k);
    Y = [Y, y];
%% IOA
 
% State observer
    
    Rxio_red = reduction(Rxio,3);
    
    xeio=A*cxio + Bu*u;
    Reio=[A*Rxio, Ew];
    
 
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
        
        
        

   
    
%% ZKF -> kalman filter aplicat a zonotopes -> Diferencia amb el IOA -> aqui estem actualizant el guany a cada step 

    RxZKF_red = reduction(RxZKF,3); % reduccio, apareix al paper com lo de R fletxa cap a baix bla bla bla 
    % expandim estats anteriors
    xeZKF=A*cxZKF + Bu*u;
    ReZKF=[A*RxZKF, Ew];
    
    %calculem guany optim amb la info d'aquest step
    L_ZKF= ((RxZKF*RxZKF')*C')*inv(C*(RxZKF*RxZKF')*C'+ (Ev*Ev'));
            
    %calcul del x+ tenint en compte el model i l'observador (mateix sistema q el IOA)
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
% Output prediction amb les dades noves     
    cyZKF = C*cxZKF;
    RyZKF =[C*RxZKF_red, Ev];
    
    EyZKF    = envbox(RyZKF);
        y1maxZKF(k) = cyZKF(1) + abs(EyZKF (1,1));
        y1minZKF(k) = cyZKF(1) - abs(EyZKF (1,1));
        y2maxZKF(k) = cyZKF(2) + abs(EyZKF (2,2));
        y2minZKF(k) = cyZKF(2) - abs(EyZKF (2,2));
        y3maxZKF(k) = cyZKF(3) + abs(EyZKF (3,3));
        y3minZKF(k) = cyZKF(3) - abs(EyZKF (3,3));
        
% Residual -> calculem l'error entre la prediccio i la y real

    crZKF = y - cyZKF;
    RrZKF = [-C*RxZKF_red, -Ev]; % canvi de signe pq la R del "zonotope" de la y esta buida -> es un punt!! 
    
    ErZKF    = envbox(RrZKF);


        r1maxZKF(k) = crZKF(1) + abs(ErZKF (1,1));
        r1minZKF(k) = crZKF(1) - abs(ErZKF (1,1));
        r2maxZKF(k) = crZKF(2) + abs(ErZKF (2,2));
        r2minZKF(k) = crZKF(2) - abs(ErZKF (2,2));
        r3maxZKF(k) = crZKF(3) + abs(ErZKF (3,3));
        r3minZKF(k) = crZKF(3) - abs(ErZKF (3,3));



%%
    x  = xx;
    cxio = cxxio;
    Rxio = Rxxio;
    
    cxZKF = cxxZKF;
    RxZKF = RxxZKF;
end
% 
% figure,
% hold on
% drawZonotope(cxio, Rxio_red, 'red')
% drawZonotope(cxsm, Rxsm_red, 'green')

  

%%
figure, set(gcf,'DefaultLineLineWidth',2.5);

    subplot(3,1,1)
    hold on
    ylabel('x_1')
    
    
    Nomx = plot(X(1,si:fi),'k');
    
    StESio = plot(x1maxio(1,si:fi),'r');
    plot(x1minio(1,si:fi),'r');
    
    StESZKF = plot(x1maxZKF(1,si:fi),'b--');
    plot(x1minZKF(1,si:fi),'b--');
    
    
    legend([Nomx, StESio, StESZKF],'x','IOA', 'ZKF');
    %
    subplot(3,1,2)
    hold on
    ylabel('x_2')
    
    plot(X(2,si:fi),'k');
    
    plot(x2maxio(1,si:fi),'r');
    plot(x2minio(1,si:fi),'r');
    
    plot(x2maxZKF(1,si:fi),'b--');
    plot(x2minZKF(1,si:fi),'b--');
    
    subplot(3,1,3)
    hold on
    ylabel('x_3')
    xlabel('Time step')
    
    plot(X(3,si:fi),'k');
    
    plot(x3maxio(1,si:fi),'r');
    plot(x3minio(1,si:fi),'r');
    
    plot(x3maxZKF(1,si:fi),'b--');
    plot(x3minZKF(1,si:fi),'b--');
%%  
% % % % % figure, set(gcf,'DefaultLineLineWidth',2);
% % % % %     subplot(3,1,1)
% % % % %     hold on
% % % % %     ylabel('y_1')
% % % % %     
% % % % %     Nomy = plot(Y(1,si:fi),'k');
% % % % %     
% % % % %     OuPrio = plot(y1maxio(1,si:fi),'r');
% % % % %     plot(y1minio(1,si:fi),'r');
% % % % %     
% % % % %     OuPrsm = plot(y1maxsm(1,si:fi),'b--');
% % % % %     plot(y1minsm(1,si:fi),'b--');
% % % % %     
% % % % %     
% % % % %     legend([Nomy, OuPrio, OuPrsm],'y','IOA', 'SMA');
% % % % %     %
% % % % %     subplot(3,1,2)
% % % % %     hold on
% % % % %     ylabel('y_2')
% % % % %     
% % % % %     plot(Y(2,si:fi),'k');
% % % % %     
% % % % %     plot(y2maxio(1,si:fi),'r');
% % % % %     plot(y2minio(1,si:fi),'r');
% % % % %     
% % % % %   
% % % % %     plot(y2maxsm(1,si:fi),'b--');
% % % % %     plot(y2minsm(1,si:fi),'b--');
% % % % %     
% % % % %     %
% % % % %     subplot(3,1,3)
% % % % %     hold on
% % % % %     ylabel('y_3')
% % % % %     
% % % % %     plot(Y(3,si:fi),'k');
% % % % %     
% % % % %     plot(y3maxio(1,si:fi),'r');
% % % % %     plot(y3minio(1,si:fi),'r');
% % % % %     
% % % % %   
% % % % %     plot(y3maxsm(1,si:fi),'b--');
% % % % %     plot(y3minsm(1,si:fi),'b--');
% % % % %%  
% % % % figure, set(gcf,'DefaultLineLineWidth',2.5);
% % % % %     subplot(3,1,1)
% % % %     hold on
% % % %     ylabel('r_1')
% % % %     
% % % %     Resio = plot(r1maxio(1,si:fi),'r');
% % % %     plot(r1minio(1,si:fi),'r');
% % % %     
% % % %     Ressm = plot(r1maxsm(1,si:SiFault+2),'b--');
% % % %     plot(r1minsm(1,si:SiFault+2),'b--');
% % % %     
% % % %     
% % % %     ThrSh = plot (zeros(1,k),'k');
% % % %     
% % % %     legend([ThrSh, Resio, Ressm],'Threshold','IOA', 'SMA');
% % % %     %
% % % %     figure, set(gcf,'DefaultLineLineWidth',2.5);
% % % % %     subplot(3,1,2)
% % % %     hold on
% % % %     ylabel('r_2')
% % % %     
% % % %     plot(r2maxio(1,si:fi),'r');
% % % %     plot(r2minio(1,si:fi),'r');
% % % %     
% % % %     plot(r2maxsm(1,si:SiFault+2),'b--');
% % % %     plot(r2minsm(1,si:SiFault+2),'b--');
% % % % 
% % % %     
% % % %     ThrSh = plot (zeros(1,k),'k');
% % % %    % 
% % % %    figure, set(gcf,'DefaultLineLineWidth',2.5);
% % % % %      subplot(3,1,3)
% % % %     hold on
% % % %     ylabel('r')
% % % %     
% % % %    Resio3 = plot(r3maxio(1,si:fi),'r');
% % % %     plot(r3minio(1,si:fi),'r');
% % % %     
% % % % 
% % % %     
% % % %    Ressm3 = plot(r3maxsm(1,si:SiFault+2),'b--');
% % % %     plot(r3minsm(1,si:SiFault+2),'b--');
% % % %     
% % % %     ThrSh3 = plot (zeros(1,k),'k');
% % % %     xlabel('Time step')
% % % %        legend([ThrSh3, Resio3, Ressm3],'Threshold','IOA', 'SMA');
% % % %     
% % % % 
% % % %  %%
% % % %  
% % % %  
% % % % figure, set(gcf,'DefaultLineLineWidth',2.5);
% % % % % subplot(3,1,1)
% % % % hold on
% % % % ylabel('fy_1')
% % % %     
% % % %     FDTestio = stairs(DetFy1io(1,si:fi),'r');   
% % % %     FDTestsm = stairs(DetFy1sm(1,si:SiFault+2),'b--');   
% % % %     legend([FDTestio, FDTestsm],'IOA', 'SMA');
% % % %  %
% % % %  figure, set(gcf,'DefaultLineLineWidth',2.5);
% % % % % subplot(3,1,2)
% % % % hold on
% % % % ylabel('fy_2')
% % % %     
% % % %     stairs(DetFy2io(1,si:fi),'r'); 
% % % %     stairs(DetFy2sm(1,si:SiFault+2),'b--'); 
% % % %     
% % % %     
% % % % figure, set(gcf,'DefaultLineLineWidth',2.5); 
% % % % % subplot(3,1,3)
% % % % hold on
% % % % ylabel('fy')
% % % %     
% % % %     FDTestio3 = stairs(DetFy3io(1,si:fi),'r');  
% % % %     FDTestsm3 = stairs(DetFy3sm(1,si:SiFault+2),'b--'); 
% % % %     
% % % %     legend([FDTestio3, FDTestsm3],'IOA', 'SMA');
% % % % xlabel('Time step')    