function [ w_angle ] = wrap( angle )

    if angle < -pi
        w_angle = 2 * pi + angle;
    elseif angle > pi
        w_angle = angle - 2 * pi;
    else
        w_angle = angle;
    end

end

