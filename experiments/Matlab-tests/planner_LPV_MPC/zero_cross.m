function [ cross ] = zero_cross( tThetaref, tXref, tYref, velocity, Ts )

% If cross == 1 means the vehicle has crossed the initial line

    factor = 2;
%     if (tThetaref(1) == 0) && (tXref(1) < 0) && (tXref(1) > -factor*velocity*Ts) && (abs(tYref(1)) < 0.5 )

    if (tXref(1) < 0) && (tXref(1) > -factor*velocity*Ts) && (abs(tYref(1)) < 0.5 )
        cross = 1;
    else
        cross = 0;
    end
    

end

