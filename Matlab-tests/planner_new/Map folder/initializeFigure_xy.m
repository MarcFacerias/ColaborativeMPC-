function [x1, y1, x2, y2] = initializeFigure_xy( map, option )

%     xdata = []; ydata = [];
%     fig = plt.figure(figsize=(8,6))
%     plt.ion()
%     axtr = plt.axes()
% %     figure(111)
% %     hold on

if option==1
    n_Points = floor(10 * (map.PointAndTangent(end, 3) + map.PointAndTangent(end, 4)));
    x0(1) = 0; y0(1) = 0;
    x1(1) = 0; y1(1) = map.halfWidth;
    x2(1) = 0; y2(1) = -map.halfWidth;
    for i = 2:n_Points-1
        [x0(i),y0(i)] = map.getGlobalPosition(i * 0.1, 0.0);        
        [x1(i),y1(i)] = map.getGlobalPosition(i * 0.1, map.halfWidth);
        [x2(i),y2(i)] = map.getGlobalPosition(i * 0.1, -map.halfWidth);
    end
else
    n_Points = floor(10 * (map.PointAndTangent(end, 3) + map.PointAndTangent(end, 4)));
    x0(1) = 0; y0(1) = 0;
    x1(1) = 0; y1(1) = 0.24;
    x2(1) = 0; y2(1) = -0.24;
    for i = 2:n_Points-1
        [x0(i),y0(i)] = map.getGlobalPosition(i * 0.1, 0.0);        
        [x1(i),y1(i)] = map.getGlobalPosition(i * 0.1, 0.24);
        [x2(i),y2(i)] = map.getGlobalPosition(i * 0.1, -0.24);
    end
end

%     plot(map.PointAndTangent(:, 1), map.PointAndTangent(:, 2), 'o')
%     hold on, plot( x0, y0, '--r')
%     hold on, plot( x1, y1, '-b')
%     hold on, plot( x2, y2, '-b')
%     grid on

end

