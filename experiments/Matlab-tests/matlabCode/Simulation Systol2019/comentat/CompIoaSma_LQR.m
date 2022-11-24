clc, clear all, close all;

nx = 3; ny = 1; nu = 2; %mal, no tindrien que ser ny = 3 ?
%-------------------------------------------------------------------------
%% Continuous-time state-space model 

% Definim el sis
Ac = [-0.02876264232, 0, 0; 
       0.02876264232, -0.03618912464, 0.007426482315; 
       0, 0.007426482315, -0.01228825622];
  
Bc = [64.93506494, 0; 0, 0;0, 0];
Cc = [1, 0, 0;0, 1, 0; 0 0 1];
% Cc = [1, 0, 0];

u = .1e-3*[2 0]';


%% Discrete-time state-space model 
%Versio discreta
Ts = 1;
A = eye(3,3)+(Ts*Ac);
Bu = Ts*Bc;
C = Cc;

%% Direction of the uncertainties

% definim les uncertanties com distribution matrices del noise
Ew = [.05 0 0;
      0 .05 0;
      0 0 .05];
Ev = .1*[.08 0 0;
      0 .08 0;
      0 0 .08];
  
%% Initialization
x  = zeros(3,1);

cxio = zeros(3,1);
    Rxio = [1 0 0; 0 1 0;0 0 1]; % Zonotope unitari ?
cxZKF = zeros(3,1);
    RxZKF = [1 0 0; 0 1 0;0 0 1];

cxsm = zeros(3,1);
    Rxsm = [1 0 0; 0 1 0;0 0 1];
cxsmaLQR = zeros(3,1);
    RxsmaLQR = [1 0 0; 0 1 0;0 0 1];


%% LQR computation
% calcula dos L amb la "mateix" funcio i que donen el mateix? -> No ! volem
% veure que pasa si fiquem com a lambda del SMA la gain L del IOA
[L] = LQRio(nx,A,C,Ew,Ev); % representa la gain L del IOA
[lambdaLQR] = LQRsm(nx,A,C,Ew,Ev); % representa la lambda del obs SMA

%%
n_red = 3;
X = [x];
Y = [];
si = 1;
fi = 2500;

%bucle de simulació 
for k=si:fi
    k;
   %% Uncertainties
   %simulem el soroll, sempre entre 0 i 1 !! 
    a = -.5; b = .5;
    w(:,k) = a + (b-a).*rand(3,1);
    v(:,k) = a + (b-a).*rand(3,1);
 
%% SYSTEM SIMULATION IN AcF OPERATION 
    %simulem estats i derivades [eq 4a i 4b]
    xx = A*x + Bu*u + Ew*w(:,k);
    X = [X, x];
    
    y = C*x + Ev*v(:,k);
    Y = [Y, y];


%%
% ------------------ Interval observer ------------------------------------
%% IOA using LQR gain    
    %Rxio es va fent gran i amb la reduccio la mantenim com 3x3
    Rxio_red = reduction(Rxio,n_red); % versio simplificada a 3D d Rxio
    
    %propagacio del zonotopo -> possibles states
    %propag. del centre i R -> apliquem la eq 2.7a en dos troços
    xeio=A*cxio + Bu*u;
    Reio=[A*Rxio, Ew];   
    
    cxxio = xeio +(L*(y-(C*xeio))); % 2.7a
    I=eye(length(xeio),length(xeio));
    
    Rxxio=[(I-(L*C))*Reio - L*Ev];  % tindria que ser el 2.7b
    % agafem una box del tamany de R i la fem servir per calcular bounds 
    % al voltatn del centre, la funcio treu la distancia del centre a cada
    % cara, una per cada component del espai en el que estem propagant
    % l'estimacio
    
        Exio    = envbox(Rxio); 
        x1maxio(k) = cxio(1) + abs(Exio (1,1));
        x1minio(k) = cxio(1) - abs(Exio (1,1));
        x2maxio(k) = cxio(2) + abs(Exio (2,2));
        x2minio(k) = cxio(2) - abs(Exio (2,2));
        x3maxio(k) = cxio(3) + abs(Exio (3,3));
        x3minio(k) = cxio(3) - abs(Exio (3,3));
        
% Output prediction       
    cyio = C*cxio; % -> predim un conjunt de possibles outputs amb un
    %un set de possibles estats i la C 
    
    Ryio =[C*Rxio_red, Ev]; % definim el bound del zonotope amb el boud 
    %anterior als estats i la matriu que projecta l'error de sortida
    
    %com antes, ens calculem els extrems de cada sortida
    Eyio = envbox(Ryio);
        y1maxio(k) = cyio(1) + abs(Eyio (1,1));
        y1minio(k) = cyio(1) - abs(Eyio (1,1));
        y2maxio(k) = cyio(2) + abs(Eyio (2,2));
        y2minio(k) = cyio(2) - abs(Eyio (2,2));
        y3maxio(k) = cyio(3) + abs(Eyio (3,3));
        y3minio(k) = cyio(3) - abs(Eyio (3,3));
        
% Residual
    crio = y - cyio; % ens generem el error del conjunt de possibles estats
    
    Rrio = [-C*Rxio_red, -Ev]; % ni puta pq te aquesta matriu de propoag
    
    Erio    = envbox(Rrio); % calcul limits dels estats
        r1maxio(k) = crio(1) + abs(Erio (1,1));
        r1minio(k) = crio(1) - abs(Erio (1,1));
        r2maxio(k) = crio(2) + abs(Erio (2,2));
        r2minio(k) = crio(2) - abs(Erio (2,2));
        r3maxio(k) = crio(3) + abs(Erio (3,3));
        r3minio(k) = crio(3) - abs(Erio (3,3));  
     
%%
% ------------------ Set-membership approach ------------------------------
%% SMA using LQR lambda      
    
    RxsmaLQR_red = reduction(RxsmaLQR,n_red); % preguntar pq fem la reduccio
    
    xesmaLQR=A*cxsmaLQR + Bu*u; % eq 2.20.a -> predim estats amb el model 
    %del sistema 
    ResmaLQR=[A*RxsmaLQR, Ew]; % eq 2.20b -> actualizem la matriu que expnadeix l'error 
    % del sistema amb el R anterior
    
    %correcio de la prediccio del model utilitzant la nova entrada + el
    %gain de lambda eq 2.19 -> vec 0 d'on surten les eq
    cxxsmaLQR = xesmaLQR +(lambdaLQR*(y-(C*xesmaLQR)));
    I=eye(length(xesmaLQR),length(xesmaLQR));
    RxxsmaLQR=[(I-(lambdaLQR*C))*ResmaLQR -lambdaLQR*Ev];
    
    % un cop definit el zonotope -> calculem limits en les x 
      ExsmaLQR    = envbox(RxsmaLQR);
        x1maxsmaLQR(k) = cxsmaLQR(1) + abs(ExsmaLQR (1,1));
        x1minsmaLQR(k) = cxsmaLQR(1) - abs(ExsmaLQR (1,1));
        x2maxsmaLQR(k) = cxsmaLQR(2) + abs(ExsmaLQR (2,2));
        x2minsmaLQR(k) = cxsmaLQR(2) - abs(ExsmaLQR (2,2));
        x3maxsmaLQR(k) = cxsmaLQR(3) + abs(ExsmaLQR (3,3));
        x3minsmaLQR(k) = cxsmaLQR(3) - abs(ExsmaLQR (3,3));
        
% a partir d'aqui anem fent igual que en IOA, pero tecnicament el SMA faria
% servir un strip que intersectaria amb els possibles outputs per saber com
% anem. 

% Output prediction -> igual que amb el anterior expandim les possibles
% sortides del sistema
    cysmaLQR = C*cxsmaLQR;
    RysmaLQR =[C*RxsmaLQR_red, Ev];

% ens quedem amb els valors extrems
    EysmaLQR    = envbox(RysmaLQR);
        y1maxsmaLQR(k) = cysmaLQR(1) + abs(EysmaLQR (1,1));
        y1minsmaLQR(k) = cysmaLQR(1) - abs(EysmaLQR (1,1));
        y2maxsmaLQR(k) = cysmaLQR(2) + abs(EysmaLQR (2,2));
        y2minsmaLQR(k) = cysmaLQR(2) - abs(EysmaLQR (2,2));
        y3maxsmaLQR(k) = cysmaLQR(3) + abs(EysmaLQR (3,3));
        y3minsmaLQR(k) = cysmaLQR(3) - abs(EysmaLQR (3,3));
% Residual -> Seria el calcul del strip no? 

    crsmaLQR = y - cysmaLQR;
    RrsmaLQR = [-C*RxsmaLQR_red, -Ev]; % no acabo de veure el canvi de signe 
    
    ErsmaLQR    = envbox(RrsmaLQR);


        r1maxsmaLQR(k) = crsmaLQR(1) + abs(ErsmaLQR (1,1));
        r1minsmaLQR(k) = crsmaLQR(1) - abs(ErsmaLQR (1,1));
        r2maxsmaLQR(k) = crsmaLQR(2) + abs(ErsmaLQR (2,2));
        r2minsmaLQR(k) = crsmaLQR(2) - abs(ErsmaLQR (2,2));
        r3maxsmaLQR(k) = crsmaLQR(3) + abs(ErsmaLQR (3,3));
        r3minsmaLQR(k) = crsmaLQR(3) - abs(ErsmaLQR (3,3));

   if  k > 2000
       
        figure, 
        hold on
        rg = [y(3,1) v(3,1)]'/v(3,1);
        [d,ro,qo] = strip(y(3),v(3,k),rg,1);
              
        ResmLQR_red = reduction(ResmaLQR, n_red);
        drawZonotope(xesmaLQR, ResmLQR_red, 'green')        
        xlabel('x_1')
        ylabel('x_2')
        zlabel('x_3')
        
%         figure, 
%         hold on
%         drawZonotope(cxsmaLQR, RxsmaLQR_red, 'blue')
%         xlabel('x_1')
%         ylabel('x_2')
%         zlabel('x_3') 
       
       
       
   end

           
    
%%
    x  = xx;
    cxio = cxxio;
    Rxio = Rxxio;
      
    cxsmaLQR = cxxsmaLQR;
    RxsmaLQR = RxxsmaLQR;
     
    
end






%% IOA
figure,
hold on
drawZonotope(cxio, Rxio_red, 'red')
        xlabel('x_1')
        ylabel('x_2')
        zlabel('x_3')
%% SMA

       figure, 
        hold on
        rg = [y(3,1) v(3,1)]'/v(3,1);
        [d,ro,qo] = strip(y(3),v(3,k),rg,1);
              
        ResmLQR_red = reduction(ResmaLQR, n_red);
        drawZonotope(xesmaLQR, ResmLQR_red, 'green')        
        xlabel('x_1')
        ylabel('x_2')
        zlabel('x_3')
        
        figure, 
        hold on
        drawZonotope(cxsmaLQR, RxsmaLQR_red, 'blue')
        xlabel('x_1')
        ylabel('x_2')
        zlabel('x_3')




    
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
       
    
    
    
    
    
    