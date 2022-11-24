function [ angle ] = computeAngle( point1, origin, point2 )

    % The orientation of this angle matches that of the coordinate system. Tha is why a minus sign is needed
    v1 = point1 - origin;
    v2 = point2 - origin;
    %
    % cosang = np.dot(v1, v2)
    % sinang = la.norm(np.cross(v1, v2))
    %
    % dp = np.dot(v1, v2)
    % laa = la.norm(v1)
    % lba = la.norm(v2)
    % costheta = dp / (laa * lba)

    dot = v1(1) * v2(1) + v1(2) * v2(2);    % dot product between [x1, y1] and [x2, y2]
    det = v1(1) * v2(2) - v1(2) * v2(1);    % determinant
    angle = atan2(det, dot);                % atan2(y, x) or atan2(sin, cos)

end

