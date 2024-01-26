% function [Rr, NCR] = reduction(R, nc)
%
% R�duction de complexit� du zonotope Z(R):
% R      : matrice de dimension (n,p) d�crivant un zonotope centr�. 
% nc     : nombre maximum de colonnes de Rr (param�tre de r�glage de la complexit�)
% Rr     : matrice avec au plus nc colonnes (nc>=n) telle que Z(R) est inclu dans Z(Rr)
% NCR    : norme (euclidienne) des vecteurs colonnes de R (crit�re de tri)
%
% C. Combastel (ECS-ENSEA), 2008

function [Zr,NCZ]=reduction(Z,nc)

% Initialisation et v�rification des param�tres
% ---------------------------------------------
n=size(Z,1);   % Dimension de l'espace
p=size(Z,2);   % Nombre de vecteurs (ou segments) engendrant le zonotope Z
if nc<n
   disp('ERREUR dans reduction: il faut nc>=n');
end   

% Partie principale
% -----------------
% Norme des vecteurs colonne de M
NCZ=sqrt(sum(Z.^2,1));

% R�duction
if p<=nc
   % R�duction inutile (limite de complexit� non atteinte)
   Zr=Z;
else
   % S�lection des vecteurs � r�duire
   nvr=p-nc+n;          % Nombre de vecteurs � r�duire
   [temp,I]=sort(NCZ);  % Tri
   Is  = I(1:nvr);      % Indices dans Z des nvr vecteurs s�lectionn�s
   Ins = I((nvr+1):p);  % Indices dans Z des vecteurs non s�lectionn�s
   Ms  = Z(:,Is);       % Vecteurs colonne de Z s�lectionn�s
   Mns = Z(:,Ins);      % Vecteurs colonne de Z non s�lectionn�s
   % Majoration de l'influence des petits segments par une bo�te
   Msr = box(Ms);
   % Construction du zonotope r�duit
   Zr  = [Mns Msr];      
end   
   
return
