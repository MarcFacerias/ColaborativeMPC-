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
Ev = [.08 0 0;
      0 .08 0;
      0 0 .08];

  
%% Initialization

x  = zeros(3,1);
cxio = zeros(3,1);
Rxio = [1 0 0; 0 1 0;0 0 1];
cxsm = zeros(3,1);
Rxsm = [1 0 0; 0 1 0;0 0 1];



%%
X = [];
Y = [];


[L] = LQRGainObsv (nx,ny,A);
L = [L(1,1) 0 0;0 L(2,1) 0;0 0 L(3,1)];

lambda=LQRLambda(nx,ny,A);
lambda = [lambda(1,1) 0 0;0 lambda(2,1) 0;0 0 lambda(3,1)];

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
%     cxxio = (A-L*C)*cxio + Bu*u + L*y;
%     Rxio_red = reduction(Rxio,3);
%     Rxxio =[(A-L*C)*Rxio_red, Ew, -L*Ev];
    
    
    Rxio_red = reduction(Rxio,3);
    
    xeio=A*cxio + Bu*u;
    Reio=[A*Rxio, Ew];
    
%     L = (A*(Rxio*Rxio')*C')*inv(C*(Rxio*Rxio')*C'+(Ev*Ev'));
    
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
        
        
        

   
    
%% SMA

    Rxsm_red = reduction(Rxsm,3);
    
    xesm=A*cxsm + Bu*u;
    Resm=[A*Rxsm, Ew];
    

%     lambda= (A*(Rxsm*Rxsm')*C')*inv(C*(Rxsm*Rxsm')*C'+ (Ev*Ev'));
    
 
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
    
    cxsm = cxxsm;
    Rxsm = Rxxsm;
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
    
    StESsm = plot(x1maxsm(1,si:fi),'b--');
    plot(x1minsm(1,si:fi),'b--');
    
    
    legend([Nomx, StESio, StESsm],'x','IOA', 'SMA');
    %
    subplot(3,1,2)
    hold on
    ylabel('x_2')
    
    plot(X(2,si:fi),'k');
    
    plot(x2maxio(1,si:fi),'r');
    plot(x2minio(1,si:fi),'r');
    
    plot(x2maxsm(1,si:fi),'b--');
    plot(x2minsm(1,si:fi),'b--');
    
    subplot(3,1,3)
    hold on
    ylabel('x_3')
    xlabel('Time step')
    
    plot(X(3,si:fi),'k');
    
    plot(x3maxio(1,si:fi),'r');
    plot(x3minio(1,si:fi),'r');
    
    plot(x3maxsm(1,si:fi),'b--');
    plot(x3minsm(1,si:fi),'b--');
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