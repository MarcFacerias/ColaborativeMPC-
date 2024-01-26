function [x] = F_model(state,input,Tc)

% Parametri del Modello

lf          = 0.902;
lr          = 0.638;
m           = 196;
I           = 93;
Cf          = 17974;
Cr          = 24181;

% Stato

vx      = state(1);
vy      = state(2);
w       = state(3);

% Ingresso

delta   = input(1);
a       = input(2);

a_F = 0.0; % Front slip angle
a_R = 0.0; % Rear slip angle

if abs(vx) > 0.1
    % Front and rear slip angles:
    a_F = atan((vy + lf*w)/vx) - delta;
    a_R = atan((vy - lr*w)/vx);
end

FyF = -Cf * a_F;
FyR = -Cr * a_R;

F_drag = 1.4*vx;

% Rispeto al modello TC utilizzo il modello TD con l'ausilio di una
% discretizzazione tramite l'approssimazione di eulero in avanti

x(1,1) = a*Tc - F_drag*Tc  -  FyF*sin(delta)*Tc/m  +  w*vy*Tc + vx;
x(2,1) = (FyF*cos(delta) + FyR )*Tc/m  -  w*vx*Tc + vy;
x(3,1) = (FyF*lf*cos(delta) - FyR*lr)*Tc/I + w;
% x(4,1) = vx;
% x(5,1) = w;

end