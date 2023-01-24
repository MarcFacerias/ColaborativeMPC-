function [ curvature ] = Curvature( s, PointAndTangent, side )

    PointAndTangent = PointAndTangent(:,:,side);
    % Compute the curvature
    TrackLength = PointAndTangent(end-1,4) + PointAndTangent(end-1,5);

    % # In case on a lap after the first one regularise
    while (s > TrackLength)
        s = s - TrackLength;
    end

    % # Given s \in [0, TrackLength] compute the curvature
    % # Compute the segment in which system is evolving
    
%     index = all( [ [s >= PointAndTangent(:, 3)], [s < PointAndTangent(:, 3) + PointAndTangent(:, 4)]] );
%     indx = int(where(squeeze(index))[0]);  % EA: this works 
    % map is build by describing segments with the same curvature and we obtain the curvature by evaluating 
    % the arc length at which we are
    for i=1:size(PointAndTangent,1)-1
        if (s >= PointAndTangent(i, 4)) && (s <= PointAndTangent(i,4)+PointAndTangent(i,5))
            indx = i;
        end
    end

    curvature = PointAndTangent(indx, 6);

end