function [d,ro,qo]=strip(y,v1,c,gr)
%STRIP   Computes the strip of parameters consistent with the data.
%   [ro,qo]=strip(y,v1,c,gr)
%
%     
%   d: 
%
%   ro:Max. value of the strip
%
%   qo: Min value of the strip
% 
%   v1: Noise value in the instant k.  
%
%   y: Output data in the instant k.
%
%   c: Regression vector divided by noise value (i.e.    rg=[y(k-1) u(k-2) u(k-1)]'/v1 ).  
%
%   gr: Value 1 to draw the strip only in 2D. 
%   
%
%   Pedro Guerra B. 10-03-2005
%   Copyright 2005 Universitat Politecnica de Catalunya.
%   $Revision: 1.0 $  $Date: 2005/01/15 17:53:38 $

d=y/v1;

ro=1+d;
qo=d-1;



if gr==1     %%%% Draw the srip
 hold on
 ax=-.5:0.1:5;
%  ax=-1.74:0.01:2.543;
 b1=(ro-ax*c(1))/c(2);
 a1=(ro-ax*c(2))/c(1);
 



 plot(ax,b1,'b')

 plot(a1,ax,'b')
 
 a2=(qo-ax*c(2))/c(1);
 b2=(qo-ax*c(1))/c(2);
 


 plot(ax,b2,'b')
 plot(a2,ax,'b')




 
 
end