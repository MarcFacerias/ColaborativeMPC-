classdef Map
    
    properties
        PointAndTangent;
        TrackLength;
        halfWidth;
        slack;
    end
    
    methods
        %% Constructor:
        function self = Map(selectedTrack)
            
            if selectedTrack == "3110"
                self.halfWidth = 0.5;
                self.slack     = 0.15;
                spec = [ 60 * 0.03, 0;
                         80 * 0.03, +80 * 0.03 * 2 / pi;
                         20 * 0.03, 0;
                         80 * 0.03, +80 * 0.03 * 2 / pi;
                         40 * 0.03, -40 * 0.03 * 10 / pi;
                         60 * 0.03, +60 * 0.03 * 5 / pi;
                         40 * 0.03, -40 * 0.03 * 10 / pi;
                         80 * 0.03, +80 * 0.03 * 2 / pi;
                         20 * 0.03, 0;                    
                         80 * 0.03, +80 * 0.03 * 2 / pi;
                         80 * 0.03, 0];
                     
            elseif selectedTrack == "Verschueren2016"
                self.halfWidth = 0.5;
                radius = 0.25;
                lengthCurve    = pi*radius;
                spec = 3*[ 1.0, 0;
                         lengthCurve,           -lengthCurve / pi;
                         lengthCurve,           lengthCurve / pi;
                         lengthCurve,           -lengthCurve / pi;
                         0.5,    0;
                         lengthCurve/2,         -lengthCurve / pi;
                         0.5,                   0;
                         lengthCurve,           lengthCurve / pi;
                         0.5,                   0;
                         lengthCurve,           -lengthCurve / pi;
                         1.0,    0;
                         lengthCurve/2,         -lengthCurve / pi;
                         0.6,    0 ];

            elseif selectedTrack == "Verschueren2016_2"
                self.halfWidth = 0.53;
                radius = 0.25;
                lengthCurve    = pi*radius;
                spec = 3*[ 1.0, 0;
                         lengthCurve,           -lengthCurve / pi;
                         lengthCurve,           lengthCurve / pi;
                         lengthCurve,           -lengthCurve / pi;
                         0.5,    0;
                         lengthCurve/2,         -lengthCurve / pi;
                         0.5,                   0;
                         lengthCurve,           lengthCurve / pi;
                         0.5,                   0;
                         lengthCurve,           -lengthCurve / pi;
                         1.0,    0;
                         lengthCurve/2,         -lengthCurve / pi;
                         0.6,    0 ];

            elseif selectedTrack == "oval"
                self.halfWidth = 0.5;
                self.slack     = 0.15;
                spec =  [ 1.0, 0;
                         4.5, 4.5 / pi;
                         2.0, 0;
                         4.5, 4.5 / pi;
                         1.0, 0 ];
                     
            elseif selectedTrack == "oval2"
                self.halfWidth = 1;
                self.slack     = 0.15;
                %original 2 
                spec =  2*[ 1.0, 0;
                         4.5, 4.5 / pi;
                         2.0, 0;
                         4.5, 4.5 / pi;
                         1.0, 0 ]; 

            elseif selectedTrack == "oval3"
                self.halfWidth = 1;
                self.slack     = 0.15;
                %original 2 
                spec =  [2.0, 0;
                         5.85, 5.85 / pi;
                         4.0, 0;
                         5.85, 5.85 / pi;
                         2.0, 0 ]; 

            elseif selectedTrack == "L_shape"
                self.halfWidth = 0.5;
                self.slack     = 0.45;
                lengthCurve    = 4.5;
                spec = [ 1.0, 0;
                         lengthCurve, lengthCurve / pi;
                         lengthCurve/2,-lengthCurve / pi;
                         lengthCurve, lengthCurve / pi;
                         lengthCurve / pi *2, 0;
                         lengthCurve/2, lengthCurve / pi;
                         2.0, 0];
              

            elseif selectedTrack == "L_shape_2"
                self.halfWidth = 0.53;
                self.slack     = 0.45;
                lengthCurve    = 4.5;
                spec = [ 1.0, 0;
                         lengthCurve, lengthCurve / pi;
                         lengthCurve/2,-lengthCurve / pi;
                         lengthCurve, lengthCurve / pi;
                         lengthCurve / pi *2, 0;
                         lengthCurve/2, lengthCurve / pi;
                         2.0, 0];



            elseif selectedTrack == "Learning_Track_Widther"
                self.halfWidth = 3;
                self.slack     = 0.45;
                spec = [ 20 , 0;
                         20 , 20 * 1 / pi;
                         20 , 0;
                         10 , -10 * 4 / pi;
                         10 , 20 * 3.5 / pi;
                         20 , 0;
                         20 , 20 * 1.7 / pi;
                         5 , 0;
                         10 , -10 * 6 / pi;
                         5 , 0;
                         10 , -10 * 6 / pi;
                         5 , 0;
                         20 , 20 * 1.03 / pi;
                         10 , 0;
                         15 , 15 * 5 / pi;
                         6 , 0;
                         20 , -10 * 2.3 / pi;
                         20 , 10 * 2 / pi;                       
                         2.5 , 0;
                         13 , -7* 4 / pi;
                         4 , 0;
                        ];                     
            end

            
            % # Now given the above segments we compute the (x, y) points of the track and the angle of the tangent vector (psi) at
            % # these points. For each segment we compute the (x, y, psi) coordinate at the last point of the segment. Furthermore,
            % # we compute also the cumulative s at the starting point of the segment at signed curvature
            % # PointAndTangent = [x, y, psi, cumulative s, segment length, signed curvature]
            
            PointAndTangent = zeros(size(spec,1) + 1, 6);
            for i = 1:size(spec,1)
                if spec(i, 2) == 0.0                % If the current segment is a straight line
                    l = spec(i, 1);                 % Length of the segments
                    % computem el nou punt inicial partint del anterior i
                    % projectan la longitud del segnment
                    if i == 1
                        ang = 0;                          % Angle of the tangent vector at the starting point of the segment
                        x = 0 + l * cos(ang);             % x coordinate of the last point of the segment
                        y = 0 + l * sin(ang);             % y coordinate of the last point of the segment
                    else
                        ang = PointAndTangent(i-1, 3);                % Angle of the tangent vector at the starting point of the segment
                        x = PointAndTangent(i-1, 1) + l * cos(ang);  % x coordinate of the last point of the segment
                        y = PointAndTangent(i-1, 2) + l * sin(ang);  % y coordinate of the last point of the segment
                    end
                    
                    psi = ang;  % Angle of the tangent vector at the last point of the segment

                    % Buffer per nova linea [x, y, psi, cumulative s, segment length, signed curvature]
                    if i == 1
                        NewLine = [x, y, psi, PointAndTangent(i, 4), l, 0];
                    else
                        % la suma es la anterior mes la actual (de segment)
                        NewLine = [x, y, psi, PointAndTangent(i-1,4) + PointAndTangent(i-1,5), l, 0];
                    end
                    
                    PointAndTangent(i,:) = NewLine;  % Write the new info

                % En el cas de que hi hagi curvatura 
                else
                    l = spec(i,1);                 % Length of the segment
                    r = spec(i,2);                 % Radius of curvature

                    %Turn sign
                    if r >= 0
                        direction = 1;
                    else
                        direction = -1;
                    end
                    
                    % If we turn to the right 
                    if i == 1
                        ang = 0;  
                        % Angle of the tangent vector at the starting point of the segment
                                                               
                        CenterX = abs(r) * cos(ang + direction * pi / 2);  % x coordinate center of circle
                        CenterY = abs(r) * sin(ang + direction * pi / 2);  % y coordinate center of circle
                    else
                        % Angle of the tangent vector at the starting point of the segment
                        ang = PointAndTangent(i - 1, 3);                              
                        % Don't really understand this pi/2 and direction
                        CenterX = PointAndTangent(i-1, 1) ...
                                  + abs(r) * cos(ang + direction * pi / 2);  % x coordinate center of circle
                        CenterY = PointAndTangent(i-1, 2) ...
                                  + abs(r) * sin(ang + direction * pi / 2);  % y coordinate center of circle
                    end
                    % a partir d'aqui no entenc gran cosa 

                    spanAng = l / abs(r);                    % Angle spanned by the circle
                    psi = wrap(ang + spanAng * sign(r));     % Angle of the tangent vector at the last point of the segment

                    angleNormal = wrap((direction * pi / 2 + ang));
                    
                    angle = -(pi - abs(angleNormal)) * (sign(angleNormal+0.001));
                    x = CenterX + abs(r) * cos(angle + direction * spanAng);  % x coordinate of the last point of the segment
                    y = CenterY + abs(r) * sin(angle + direction * spanAng);  % y coordinate of the last point of the segment

                    if i == 1
                        NewLine = [x, y, psi, PointAndTangent(i, 4), l, 1 / r];
                    else
                        NewLine = [x, y, psi, PointAndTangent(i-1, 4) + PointAndTangent(i-1, 5), l, 1 / r];
                    end
                    
                    PointAndTangent(i, :) = NewLine;  % Write the new info
                % plt.plot(x, y, 'or')
                
                end
            end

            % Segment final que tanca el circuit 
            xs = PointAndTangent(end-1, 1);
            ys = PointAndTangent(end-1, 2);
            xf = 0;
            yf = 0;
            psif = 0;

            % plt.plot(xf, yf, 'or')
            % plt.show()
            l = sqrt((xf - xs)^2 + (yf - ys)^2);

            NewLine = [xf, yf, psif, PointAndTangent(end-1, 4) + PointAndTangent(end-1, 5), l, 0];
            PointAndTangent(end, :) = NewLine;

            self.PointAndTangent = PointAndTangent;
             self.TrackLength = PointAndTangent(end, 4) + PointAndTangent(end, 5);            
%             self.TrackLength = PointAndTangent(end, 4);  
        end
        
        
        
        
        
        
        %% getGlobalPosition:
        
        function [ x, y, theta ] = getGlobalPosition( self, s, ey,k )
            % wrap s along the track
            while (s > self.TrackLength)
                s = s - self.TrackLength;
            end

            % Compute the segment in which system is evolving
            PointAndTangent = self.PointAndTangent;

            for i=1:size(PointAndTangent,1)
                if (s >= PointAndTangent(i, 4)) && (s < PointAndTangent(i,4)+PointAndTangent(i,5)) 
                    indx = i;
                end
            end
            i = indx;

            if PointAndTangent(i, 6) == 0.0   % If segment is a straight line
                % Extract the first final and initial point of the segment
                
                if i == 1
                    xf = PointAndTangent(i, 1);
                    yf = PointAndTangent(i, 2);
                    xs = PointAndTangent(end, 1);
                    ys = PointAndTangent(end, 2);
                    psi = PointAndTangent(i, 3);
                else
                    xf = PointAndTangent(i, 1);
                    yf = PointAndTangent(i, 2);
                    xs = PointAndTangent(i-1, 1);
                    ys = PointAndTangent(i-1, 2);
                    psi = PointAndTangent(i, 3);                    
                end

                % Compute the segment length
                deltaL = PointAndTangent(i, 5);
                reltaL = s - PointAndTangent(i, 4);

                % Do the linear combination
                x = (1 - reltaL / deltaL) * xs + reltaL / deltaL * xf + ey * cos(psi + pi / 2);
                y = (1 - reltaL / deltaL) * ys + reltaL / deltaL * yf + ey * sin(psi + pi / 2);
                theta = psi;
            else
                r = 1 / PointAndTangent(i, 6);      % Extract curvature
                ang = PointAndTangent(i-1, 3);      % Extract angle of the tangent at the initial point (i-1)
                % Compute the center of the arc:
                if r >= 0
                    direction = 1;
                else
                    direction = -1;
                end

                CenterX = PointAndTangent(i - 1, 1) ...
                          + abs(r) * cos(ang + direction * pi / 2);  % x coordinate center of circle
                CenterY = PointAndTangent(i - 1, 2) ...
                          + abs(r) * sin(ang + direction * pi / 2);  % y coordinate center of circle

                spanAng = ((s - PointAndTangent(i, 4)) / (pi * abs(r))) * pi;

                angleNormal = wrap(((direction * pi / 2) + ang));
                angle = -(pi - abs(angleNormal)) * (sign(angleNormal+0.001));

                x = CenterX + (abs(r) - direction * ey) * cos(angle + direction * spanAng);  % x coordinate of the last point of the segment
                y = CenterY + (abs(r) - direction * ey) * sin(angle + direction * spanAng);  % y coordinate of the last point of the segment
%                 theta = spanAng; % Current circuit orientation
%                 theta = angle + direction * spanAng;
                theta = wrapToPi(ang + direction * spanAng);
            end
        
        end
        
        
        
        
        
        
        
        
        
        %% getLoalPosition:
        % coordinate transformation from inertial reference frame (X, Y) to curvilinear reference frame (s, ey)
        % (X, Y): position in the inertial reference frame

        function [ s, ey, epsi ] = getLocalPosition( self, x, y, psi,k )
            
            PointAndTangent = self.PointAndTangent;
            CompletedFlag = 0;

            %for i in range(0, PointAndTangent.shape[0]):
            for i = 1:size(PointAndTangent,1)
                if CompletedFlag == 1
                    break
                end

                if PointAndTangent(i, 6) == 0.0  % If segment is a straight line
                    % Extract the first final and initial point of the segment

                    if i == 1
                        xf = PointAndTangent(i, 1);
                        yf = PointAndTangent(i, 2);
                        xs = PointAndTangent(end, 1);
                        ys = PointAndTangent(end, 2);
                    else
                        xf = PointAndTangent(i, 1);
                        yf = PointAndTangent(i, 2);
                        xs = PointAndTangent(i-1, 1);
                        ys = PointAndTangent(i-1, 2);                  
                    end

                    psi_unwrap = wrap(psi); %%%%%%% en un principio era unwrap y otra funcion
                    
                    epsi = psi_unwrap - PointAndTangent(end, 3); % etheta = real - desired
                    
                    % Check if on the segment using angles
                    if norm([xs, ys] - [x, y]) == 0
                        s  = PointAndTangent(i, 4);
                        ey = 0;
                        CompletedFlag = 1;

                    elseif norm([xf, yf] - [x, y]) == 0
                        s = PointAndTangent(i, 4)+ PointAndTangent(i, 5);
                        ey = 0;
                        CompletedFlag = 1;
                    else
                        if abs(computeAngle( [x,y] , [xs, ys], [xf, yf])) <= pi/2 && abs(computeAngle( [x,y] , [xf, yf], [xs, ys])) <= pi/2
                            v1 = [x,y] - [xs, ys];
                            angle = computeAngle( [xf,yf] , [xs, ys], [x, y]);
                            s_local = norm(v1) * cos(angle);
                            s       = s_local + PointAndTangent(i, 4);
                            ey      = norm(v1) * sin(angle);

                            if abs(ey)<= self.halfWidth + self.slack
                                CompletedFlag = 1;
                            end
                        end
                    end

                else
                    if i == 1
                        xf = PointAndTangent(i, 1);
                        yf = PointAndTangent(i, 2);
                        xs = PointAndTangent(end, 1);
                        ys = PointAndTangent(end, 2);
                    else
                        xf = PointAndTangent(i, 1);
                        yf = PointAndTangent(i, 2);
                        xs = PointAndTangent(i-1, 1);
                        ys = PointAndTangent(i-1, 2);                  
                    end 

                    r = 1 / PointAndTangent(i, 6);  % Extract curvature
                    if r >= 0
                        direction = 1;
                    else
                        direction = -1;
                    end

                    ang = PointAndTangent(i-1, 3);      % Extract angle of the tangent at the initial point (i-1)

                    % Compute the center of the arc
                    CenterX = xs + abs(r) * cos(ang + direction * pi / 2);  % x coordinate center of circle
                    CenterY = ys + abs(r) * sin(ang + direction * pi / 2);  % y coordinate center of circle

                    % Check if on the segment using angles
                    if norm([xs, ys] - [x, y]) == 0
                        ey = 0;
                        %psi_unwrap = unwrap([ang, psi]);
                        psi_unwrap = wrap(psi);
                        epsi = psi_unwrap - ang;
                        s = PointAndTangent(i, 4);
                        CompletedFlag = 1;
                    elseif norm([xf, yf] - [x, y]) == 0
                        s = PointAndTangent(i, 4) + PointAndTangent(i, 5);
                        ey = 0;
                        %psi_unwrap = unwrap([PointAndTangent(i, 3], psi]))[1];
                        psi_unwrap = wrap(psi);
                        epsi = psi_unwrap - PointAndTangent(i, 3);
                        CompletedFlag = 1;
                    else
                        arc1 = PointAndTangent(i, 5) * PointAndTangent(i, 6);
                        arc2 = computeAngle([xs, ys], [CenterX, CenterY], [x, y]);
                        if sign(arc1) == sign(arc2) && abs(arc1) >= abs(arc2)
                            v = [x, y] - [CenterX, CenterY];
                            s_local = abs(arc2)*abs(r);
                            s    = s_local + PointAndTangent(i, 4);
                            ey   = -sign(direction) * (norm(v) - abs(r));
                            psi_unwrap = wrap(psi);             %% No estoy seguro de que esto este bien, pero el angulo ahora no nos hace falta.
                            epsi = psi_unwrap - (ang + arc2);

%                             if abs(ey) <= self.halfWidth + self.slack % OUT OF TRACK!!
%                                 CompletedFlag = 1;
%                             end
                        end
                    end
                end
            end % END FOR

        
            if CompletedFlag == 0
                s    = 10000;
                ey   = 10000;
                epsi = 10000;
            end
            
        end % END FUNCTION

    end % END METHODS
    
end % END CLASS



            