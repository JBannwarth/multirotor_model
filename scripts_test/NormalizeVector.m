function [ qO ] = NormalizeVector( qI )
%NORMALIZEVECTOR Return normalized vector 
%   Written by: J.X.J. Bannwarth, 28/03/2017

    qO = qI ./ norm(qI);
    
end

