function [R]=envbox(H)
%ENVBOX   Computes the intervall hull of a zonotope.
%   R=envbox(H)
%
%   R: Intervall hull matrix.
%   
%   H: zonotope matrix.
% 
%   
%
%   Pedro Guerra B. 15-01-2005
%   Copyright 2005 Universitat Politecnica de Catalunya.
%   $Revision: 1.0 $  $Date: 2005/01/15 17:53:38 $

for i=1:size(H,1)
    R(i,i)=sum(abs(H(i,:)));
end