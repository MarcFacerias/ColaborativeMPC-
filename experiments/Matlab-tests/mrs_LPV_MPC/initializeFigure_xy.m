function [track, points] = initializeFigure_xy( map )

%    xdata = []; ydata = [];
%    fig = plt.figure(figsize=(8,6))
%    plt.ion()
%    axtr = plt.axes()
%     figure(111)
%     hold on
track = []; 
points = []; 

for k = 1:size(map.PointAndTangent,3)
    n_Points = floor(10 * (map.PointAndTangent(end, 3,k) + map.PointAndTangent(end, 4,k)));
    points(k) = n_Points-1; 
    [x0(1),y0(1)] = map.getGlobalPosition(0, 0.0, k);        
    [x1(1),y1(1)] = map.getGlobalPosition(0, map.halfWidth,k);
    [x2(1),y2(1)] = map.getGlobalPosition(0, -map.halfWidth,k);
    for i = 2:n_Points-1
        [x0(i),y0(i)] = map.getGlobalPosition(i * 0.1, 0.0, k);        
        [x1(i),y1(i)] = map.getGlobalPosition(i * 0.1, map.halfWidth,k);
        [x2(i),y2(i)] = map.getGlobalPosition(i * 0.1, -map.halfWidth,k);
    end

%     plot(map.PointAndTangent(:, 1, k), map.PointAndTangent(:, 2, k ), 'o')
%     hold on, plot( x0(1:n_Points-1), y0(1:n_Points-1), '--r')
%     hold on, plot( x1(1:n_Points-1), y1(1:n_Points-1), '-b')
%     hold on, plot( x2(1:n_Points-1), y2(1:n_Points-1), '-b')
%     grid on

    track(:,:,k) = [x0', y0',x1', y1',x2', y2']; 

end
end

