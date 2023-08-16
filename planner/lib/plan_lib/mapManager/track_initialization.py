#!/usr/bin/env python

import warnings
from math import floor

import numpy as np
import numpy.linalg as la


class Map():

    def __init__(self, selectedTrack = None):
        """Initialization
        Modify the vector spec to change the geometry of the track
        """

        """ Nos interesa que el planner tenga una pista algo mas reducida de la real
        para conservar algo de robustez y no salirnos de la pista en el primer segundo. """
        HW            = 0.5

        self.lane = 0

        if selectedTrack == "3110":

            self.slack     = 0.15
            spec = np.array([[60 * 0.03, 0],
                             [80 * 0.03, +80 * 0.03 * 2 / np.pi],
                             [20 * 0.03, 0],
                             [80 * 0.03, +80 * 0.03 * 2 / np.pi],
                             [40 * 0.03, -40 * 0.03 * 10 / np.pi],
                             [60 * 0.03, +60 * 0.03 * 5 / np.pi],
                             [40 * 0.03, -40 * 0.03 * 10 / np.pi],
                             [80 * 0.03, +80 * 0.03 * 2 / np.pi],
                             [20 * 0.03, 0],
                             [80 * 0.03, +80 * 0.03 * 2 / np.pi],
                             [80 * 0.03, 0]])

            self.halfWidth = 0.6 * np.ones(spec.shape[0])
            self.open = False

        elif selectedTrack == "oval":
            self.slack      = 0.15
            spec = np.empty((5,2,1))
            spec[:,:,0] = np.array([[2.0, 0],
                             [5.85, 5.85 / np.pi],
                             [4.0, 0],
                             [5.85, 5.85 / np.pi],
                             [2.0, 0]])
            self.halfWidth = np.array([0.55,0.55,0.55,0.55,0.55,0.55])
            self.open = False

        elif selectedTrack == "oval_mt":
            self.slack      = 0.15
            spec = np.empty((5,2,1))
            spec[:,:,0] = np.array([[1.0, 0],
                             [4.5, 4.5 / np.pi],
                             [2.0, 0],
                             [4.5, 4.5 / np.pi],
                             [1.0, 0]])
            self.halfWidth = 0.5 * np.ones(spec.shape[0])
            self.open = False

        elif selectedTrack == "Oval2":
            self.slack      = 0.15
            scale = 2
            spec = np.empty((5,2,2))
            spec[:,:,0] = scale * np.array([[1.0, 0],
                             [4.5, 4.5 / np.pi],
                             [2.0, 0],
                             [4.5, 4.5 / np.pi],
                             [1.0, 0]])

            spec[:,:,1] = np.array([[2.0, 0],
                             [5.85, 5.85 / np.pi],
                             [4.0, 0],
                             [5.85, 5.85 / np.pi],
                             [2.0, 0]])
            self.halfWidth = HW * np.ones(spec.shape[0])
            self.open = False

        elif selectedTrack == "TestOpenMap":
            self.slack      = 0.15
            scale = 2
            spec = np.empty((5,2,2))
            # Add two 0 segments to prevent weird jumps
            spec[:,:,0] = scale * np.array([[0.0, 0],[1.0, 0],
                             [4.5, 4.5 / np.pi],
                             [2.0, 0],
                             [0.0, 0]])

            spec[:,:,1] = np.array([[0.0, 0],[2.0, 0],
                             [5.85, 5.85 / np.pi],
                             [4.0, 0],
                             [0.0, 0]])
            self.halfWidth = HW * np.ones(spec.shape[0])
            self.open = True

        elif selectedTrack == "Highway":
            self.slack      = 0.15
            scale = 2
            spec = np.empty((10,2,1))
            spec[:,:,0] = scale * np.array([[0.0, 0],[1.0, 0],
                             [4.5, 4.5 / (0.5*np.pi)],
                             [2.0, 0],
                             [2.5, -2.5 / (0.5 * np.pi)],
                             [2.0, 0],
                             [4.5, 4.5 / np.pi],
                             [2.0, 0],
                             [5.0, 0],
                             [0.0, 0]])

            self.halfWidth = 0.75 * np.ones(spec.shape[0])
            self.open = True

        elif selectedTrack == "SL":
            self.slack = 0.15
            scale = 2
            spec = np.empty((12, 2, 1))
            spec[:, :, 0] = scale * np.array([[0.0, 0],
                                              [3.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [1.0, 0],
                                              [2.0, 0],
                                              [3.0, 0]])

            self.halfWidth = np.asarray([0.75,0.75,0.65,0.65,0.55,0.35,0.35,0.55,0.65,0.65,0.75,0.75])
            self.open = True


        elif selectedTrack == "L_shape":
            self.slack      = 0.45
            lengthCurve     = 4.5
            spec = np.array([[1.0, 0],
                             [lengthCurve, lengthCurve / np.pi],
                             # Note s = 1 * np.pi / 2 and r = -1 ---> Angle spanned = np.pi / 2
                             [lengthCurve/2,-lengthCurve / np.pi ],
                             [lengthCurve, lengthCurve / np.pi],
                             [lengthCurve / np.pi *2, 0],
                             [lengthCurve/2, lengthCurve / np.pi]])
            self.halfWidth = HW * np.ones(spec.shape[0])
            self.open = False

        elif selectedTrack == "L_shape_IDIADA":
            self.slack      = 6*0.45
            lengthCurve     = 10*4.5
            spec = np.array([[1.0, 0],
                             [lengthCurve, lengthCurve / np.pi],
                             # Note s = 1 * np.pi / 2 and r = -1 ---> Angle spanned = np.pi / 2
                             [lengthCurve/2,-lengthCurve / np.pi ],
                             [lengthCurve, lengthCurve / np.pi],
                             [lengthCurve / np.pi *2, 0],
                             [lengthCurve/2, lengthCurve / np.pi]])
            self.halfWidth = HW * np.ones(spec.shape[0])
            self.open = False


        elif selectedTrack == "SLAM_shape1":
            self.slack     = 0.15
            lengthCurve    = 1.5*(np.pi/2)
            spec = np.array([[2.5,0],
                             [2*lengthCurve,(lengthCurve*2)/np.pi],
                             [lengthCurve,-(lengthCurve*2) / np.pi],
                             [1.0,0],
                             [lengthCurve,lengthCurve*2/np.pi],
                             [2.0,0],
                             [lengthCurve,(lengthCurve*2)/np.pi],
                             [4.0,0],
                             [lengthCurve,(lengthCurve*2)/np.pi],
                             [2.6,0]])
            self.halfWidth = 0.4 * np.ones(spec.shape[0])
            self.open = False

        elif selectedTrack == "8_track":
            self.slack     = 0.15
            lengthCurve    = 1.5*(np.pi/2)
            spec = np.array([[0.5,0],
                             [lengthCurve,(lengthCurve*2)/np.pi],
                             [1.0,0],
                             [lengthCurve,-(lengthCurve*2) / np.pi],
                             [lengthCurve,lengthCurve*2/np.pi],
                             [lengthCurve,lengthCurve*2/np.pi],
                             [1.0,0],
                             [lengthCurve,(lengthCurve*2)/np.pi],
                             [lengthCurve,-(lengthCurve*2)/np.pi],
                             [lengthCurve,(lengthCurve*2)/np.pi],
                             [1.0,0],
                             [lengthCurve,lengthCurve*2/np.pi]])
            self.halfWidth = 0.4 * np.ones(spec.shape[0])
            self.open = False

        else:
            warnings.warn("Track not found! defaulting to Oval2")
            self.slack      = 0.15
            scale = 2
            spec = np.empty((5,2,2))
            spec[:,:,0] = scale * np.array([[1.0, 0],
                             [4.5, 4.5 / np.pi],
                             [2.0, 0],
                             [4.5, 4.5 / np.pi],
                             [1.0, 0]])

            spec[:,:,1] = np.array([[2.0, 0],
                             [5.85, 5.85 / np.pi],
                             [4.0, 0],
                             [5.85, 5.85 / np.pi],
                             [2.0, 0]])
            self.halfWidth = HW * np.ones(spec.shape[0])
            self.open = False


        # Now given the above segments we compute the (x, y) points of the track and the angle of the tangent vector (psi) at
        # these points. For each segment we compute the (x, y, psi) coordinate at the last point of the segment. Furthermore,
        # we compute also the cumulative s at the starting point of the segment at signed curvature
        # PointAndTangent = [x, y, psi, cumulative s, segment length, signed curvature]
        roads = spec.shape[2]
        if self.open:
            PointAndTangent = np.zeros((spec.shape[0] , 6,roads))
        else:
            PointAndTangent = np.zeros((spec.shape[0]+1, 6, roads))

        y_ini = [2 * self.halfWidth[0], 2 * 2 * self.halfWidth[0]]
        self.TrackLength = np.zeros(roads)
        for k in range(0, roads):
            for i in range(0, spec.shape[0]):
                if spec[i, 1, k] == 0.0:              # If the current segment is a straight line
                    l = spec[i, 0, k]                 # Length of the segments
                    if i == 0:
                        ang = 0                          # Angle of the tangent vector at the starting point of the segment
                        x = 0 + l * np.cos(ang)          # x coordinate of the last point of the segment
                        y = y_ini[k] + l * np.sin(ang)          # y coordinate of the last point of the segment
                    else:
                        ang = PointAndTangent[i - 1, 2, k]                 # Angle of the tangent vector at the starting point of the segment
                        x = PointAndTangent[i-1, 0, k] + l * np.cos(ang)  # x coordinate of the last point of the segment
                        y = PointAndTangent[i-1, 1, k] + l * np.sin(ang)  # y coordinate of the last point of the segment
                    psi = ang  # Angle of the tangent vector at the last point of the segment

                    if i == 0:
                        NewLine = np.array([x, y, psi, PointAndTangent[i, 3, k], l, 0])
                    else:
                        NewLine = np.array([x, y, psi, PointAndTangent[i-1, 3, k] + PointAndTangent[i-1, 4, k], l, 0])

                    PointAndTangent[i, :, k] = NewLine  # Write the new info
                else:
                    l = spec[i, 0, k]                 # Length of the segment
                    r = spec[i, 1, k]                 # Radius of curvature


                    if r >= 0:
                        direction = 1
                    else:
                        direction = -1

                    if i == 0:
                        ang = 0                                                      # Angle of the tangent vector at the
                                                                                     # starting point of the segment
                        CenterX = 0 + np.abs(r) * np.cos(ang + direction * np.pi / 2)  # x coordinate center of circle
                        CenterY = 0 + np.abs(r) * np.sin(ang + direction * np.pi / 2)  # y coordinate center of circle
                    else:
                        ang = PointAndTangent[i - 1, 2, k]                              # Angle of the tangent vector at the
                                                                                     # starting point of the segment
                        CenterX = PointAndTangent[i-1, 0, k] + np.abs(r) * np.cos(ang + direction * np.pi / 2)  # x coordinate center of circle
                        CenterY = PointAndTangent[i-1, 1, k] + np.abs(r) * np.sin(ang + direction * np.pi / 2)  # y coordinate center of circle

                    spanAng = l / np.abs(r)  # Angle spanned by the circle
                    psi = wrap(ang + spanAng * np.sign(r))  # Angle of the tangent vector at the last point of the segment

                    angleNormal = wrap((direction * np.pi / 2 + ang))
                    angle = -(np.pi - np.abs(angleNormal)) * (sign(angleNormal))
                    x = CenterX + np.abs(r) * np.cos(
                        angle + direction * spanAng)  # x coordinate of the last point of the segment
                    y = CenterY + np.abs(r) * np.sin(
                        angle + direction * spanAng)  # y coordinate of the last point of the segment

                    if i == 0:
                        NewLine = np.array([x, y, psi, PointAndTangent[i, 3, k], l, 1 / r])
                    else:
                        NewLine = np.array([x, y, psi, PointAndTangent[i-1, 3, k] + PointAndTangent[i-1, 4, k], l, 1 / r])

                    PointAndTangent[i, :, k] = NewLine  # Write the new info

            if not self.open:
                xs = PointAndTangent[-2, 0, k]
                ys = PointAndTangent[-2, 1, k]
                xf = 0
                yf = y_ini[k]
                psif = 0

                l = np.sqrt((xf - xs) ** 2 + (yf - ys) ** 2)

                NewLine = np.array([xf, yf, psif, PointAndTangent[-2, 3, k] + PointAndTangent[-2, 4, k], l, 0])
                PointAndTangent[-1, :, k] = NewLine

            self.TrackLength[k] = PointAndTangent[-1, 3, k] + PointAndTangent[-1, 4, k]
        self.PointAndTangent = PointAndTangent
        
    def set_lane(self,lane):
        self.lane = lane

    def wrap_s(self,s, lane = None):

        if not self.open:
            while (s >= self.TrackLength[lane]):
                s = s - self.TrackLength[lane]

        elif s >= self.TrackLength[self.lane]:
            s = s - self.TrackLength[self.lane]

        if s < 0:
            s = 0

        return s

    def check_lap(self,s, lane = None):

        lap = floor(s/self.TrackLength[lane])

        return lap

    def getGlobalPosition(self, s, ey, lane = None, plotting = False):
        """coordinate transformation from curvilinear reference frame (e, ey) to inertial reference frame (X, Y)
        (s, ey): position in the curvilinear reference frame
        """

        if lane is None:
            lane = self.lane

        # Code to treat the case were we deal with the map, ugly fix

        try:
            if not ey.shape[0] == self.PointAndTangent[:,:,lane].shape[0]:
                ey = ey[0] * np.ones(self.PointAndTangent[:, :, lane].shape[0])
        except:
            ey = ey * np.ones(self.PointAndTangent[:, :, lane].shape[0])

        s = self.wrap_s(s)

        # Compute the segment in which system is evolving
        PointAndTangent = self.PointAndTangent[:,:,lane]

        index = np.all([[s >= PointAndTangent[:, 3]], [s < PointAndTangent[:, 3] + PointAndTangent[:, 4]]], axis=0)
        i = np.where(np.squeeze(index))[0]

        if PointAndTangent[i, 5] == 0.0:  # If segment is a straight line
            # Extract the first final and initial point of the segment
            xf = PointAndTangent[i, 0]
            yf = PointAndTangent[i, 1]
            xs = PointAndTangent[i - 1, 0]
            ys = PointAndTangent[i - 1, 1]
            psi = PointAndTangent[i, 2]

            # Compute the segment length
            deltaL = PointAndTangent[i, 4]
            reltaL = s - PointAndTangent[i, 3]

            if plotting and i < self.PointAndTangent[:, :, lane].shape[0]-1 and PointAndTangent[i, 4] != 0:
                dey = (ey[i+1] - ey[i]) * (s - PointAndTangent[i, 3]) /  PointAndTangent[i, 4] + ey[i]
            else:
                dey = ey[i]
            # Do the linear combination
            x = (1 - reltaL / deltaL) * xs + reltaL / deltaL * xf + dey * np.cos(psi + np.pi / 2)
            y = (1 - reltaL / deltaL) * ys + reltaL / deltaL * yf + dey * np.sin(psi + np.pi / 2)
            theta = psi
        else:
            r = 1 / PointAndTangent[i, 5]  # Extract curvature
            ang = PointAndTangent[i - 1, 2]  # Extract angle of the tangent at the initial point (i-1)
            # Compute the center of the arc
            if r >= 0:
                direction = 1
            else:
                direction = -1

            CenterX = PointAndTangent[i - 1, 0] \
                      + np.abs(r) * np.cos(ang + direction * np.pi / 2)  # x coordinate center of circle
            CenterY = PointAndTangent[i - 1, 1] \
                      + np.abs(r) * np.sin(ang + direction * np.pi / 2)  # y coordinate center of circle

            spanAng = (s - PointAndTangent[i, 3]) / (np.pi * np.abs(r)) * np.pi

            angleNormal = wrap((direction * np.pi / 2 + ang))
            angle = -(np.pi - np.abs(angleNormal)) * (sign(angleNormal))

            if plotting and i < self.PointAndTangent[:, :, lane].shape[0] and PointAndTangent[i, 4] != 0:
                dey = (ey[i+1] - ey[i]) * (s - PointAndTangent[i, 3]) /  PointAndTangent[i, 4] + ey[i]
            else:
                dey = ey[i]

            x = CenterX + (np.abs(r) - direction * dey) * np.cos(
                angle + direction * spanAng)  # x coordinate of the last point of the segment
            y = CenterY + (np.abs(r) - direction * dey) * np.sin(
                angle + direction * spanAng)  # y coordinate of the last point of the segment
            theta = ang + direction * spanAng

        return x, y, theta



    def getGlobalPosition_Racing(self, ex, ey, xd, yd, psid):
        'DUBTO DE AQUESTA FUNCION PRO JA VEUREM'
        """coordinate transformation from curvilinear reference frame (ex, ey) to inertial reference frame (X, Y)
        based on inverse of error computation for racing:
            ex      = +(x-xd)*np.cos(psid) + (y-yd)*np.sin(psid)
            ey      = -(x-xd)*np.sin(psid) + (y-yd)*np.cos(psid)
            epsi    = wrap(psi-psid)
        """

        # x = ex*np.cos(psid) - ey*np.sin(psid) + xd
        x = xd
        y = (ey - xd*np.sin(psid) + yd*np.cos(psid) + x*np.sin(psid)) / np.cos(psid)

        return x, y

    def checkLane(self, x, y, psi ):

        ey_target = 10000
        lane = 0
        r_s = 10000
        r_ey = 10000
        r_epsi = 10000
        r_CompletedFlag = 0

        for k in range(0, self.PointAndTangent.shape[-1]):
            s, ey, epsi, CompletedFlag = self.getLocalPosition( x, y, psi, k)
            if abs(ey) < abs(ey_target):
                lane = k
                r_s = s
                r_ey = ey
                r_epsi = epsi
                r_CompletedFlag = CompletedFlag

        return r_s, r_ey, r_epsi, r_CompletedFlag, lane

    def getLocalPosition(self, x, y, psi, lane = None):
        """coordinate transformation from inertial reference frame (X, Y) to curvilinear reference frame (s, ey)
        (X, Y): position in the inertial reference frame
        """

        if lane is None:
            lane = self.lane

        PointAndTangent = self.PointAndTangent[:,:,lane]
        CompletedFlag = 0

        for i in range(0, PointAndTangent.shape[0]):
            if CompletedFlag == 1:
                break

            if PointAndTangent[i, 5] == 0.0:  # If segment is a straight line
                # Extract the first final and initial point of the segment
                xf = PointAndTangent[i, 0]
                yf = PointAndTangent[i, 1]
                xs = PointAndTangent[i - 1, 0]
                ys = PointAndTangent[i - 1, 1]

                psi_unwrap = np.unwrap([PointAndTangent[i - 1, 2], psi])[1]
                epsi = psi_unwrap - PointAndTangent[i - 1, 2]

                # Check if on the segment using angles
                if (la.norm(np.array([xs, ys]) - np.array([x, y]))) == 0:
                    s  = PointAndTangent[i, 3]
                    ey = 0
                    CompletedFlag = 1

                elif (la.norm(np.array([xf, yf]) - np.array([x, y]))) == 0:
                    s = PointAndTangent[i, 3] + PointAndTangent[i, 4]
                    ey = 0
                    CompletedFlag = 1
                else:
                    if np.abs(computeAngle( [x,y] , [xs, ys], [xf, yf])) <= np.pi/2 and np.abs(computeAngle( [x,y] , [xf, yf], [xs, ys])) <= np.pi/2:
                        v1 = np.array([x,y]) - np.array([xs, ys])
                        angle = computeAngle( [xf,yf] , [xs, ys], [x, y])
                        s_local = la.norm(v1) * np.cos(angle)
                        s       = s_local + PointAndTangent[i, 3]
                        ey      = la.norm(v1) * np.sin(angle)

                        if np.abs(ey)<= self.halfWidth[i] + self.slack:
                            CompletedFlag = 1

            else:
                xf = PointAndTangent[i, 0]
                yf = PointAndTangent[i, 1]
                xs = PointAndTangent[i - 1, 0]
                ys = PointAndTangent[i - 1, 1]

                r = 1 / PointAndTangent[i, 5]  # Extract curvature
                if r >= 0:
                    direction = 1
                else:
                    direction = -1

                ang = PointAndTangent[i - 1, 2]  # Extract angle of the tangent at the initial point (i-1)

                # Compute the center of the arc
                CenterX = xs + np.abs(r) * np.cos(ang + direction * np.pi / 2)  # x coordinate center of circle
                CenterY = ys + np.abs(r) * np.sin(ang + direction * np.pi / 2)  # y coordinate center of circle

                # Check if on the segment using angles
                if (la.norm(np.array([xs, ys]) - np.array([x, y]))) == 0:
                    ey = 0
                    psi_unwrap = np.unwrap([ang, psi])[1]
                    epsi = psi_unwrap - ang
                    s = PointAndTangent[i, 3]
                    CompletedFlag = 1
                elif (la.norm(np.array([xf, yf]) - np.array([x, y]))) == 0:
                    s = PointAndTangent[i, 3] + PointAndTangent[i, 4]
                    ey = 0
                    psi_unwrap = np.unwrap([PointAndTangent[i, 2], psi])[1]
                    epsi = psi_unwrap - PointAndTangent[i, 2]
                    CompletedFlag = 1
                else:
                    arc1 = PointAndTangent[i, 4] * PointAndTangent[i, 5]
                    arc2 = computeAngle([xs, ys], [CenterX, CenterY], [x, y])
                    if np.sign(arc1) == np.sign(arc2) and np.abs(arc1) >= np.abs(arc2):
                        v = np.array([x, y]) - np.array([CenterX, CenterY])
                        s_local = np.abs(arc2)*np.abs(r)
                        s    = s_local + PointAndTangent[i, 3]
                        ey   = -np.sign(direction) * (la.norm(v) - np.abs(r))
                        psi_unwrap = np.unwrap([ang + arc2, psi])[1]
                        epsi = psi_unwrap - (ang + arc2)

                        if np.abs(ey) <= 3*self.halfWidth[i] + self.slack: # OUT OF TRACK!!
                            CompletedFlag = 1

        if CompletedFlag == 0:
            s    = 10000
            ey   = 10000
            epsi = 10000


        return s, ey, epsi, CompletedFlag



# ======================================================================================================================
# ======================================================================================================================
# ====================================== Internal utilities functions ==================================================
# ======================================================================================================================
# ======================================================================================================================

def computeAngle(point1, origin, point2):
    # The orientation of this angle matches that of the coordinate system. Tha is why a minus sign is needed
    v1 = np.array(point1) - np.array(origin)
    v2 = np.array(point2) - np.array(origin)
    #
    # cosang = np.dot(v1, v2)
    # sinang = la.norm(np.cross(v1, v2))
    #
    # dp = np.dot(v1, v2)
    # laa = la.norm(v1)
    # lba = la.norm(v2)
    # costheta = dp / (laa * lba)

    dot = v1[0] * v2[0] + v1[1] * v2[1]  # dot product between [x1, y1] and [x2, y2]
    det = v1[0] * v2[1] - v1[1] * v2[0]  # determinant
    angle = np.arctan2(det, dot)  # atan2(y, x) or atan2(sin, cos)

    return angle # np.arctan2(sinang, cosang)


def wrap(angle):
    if angle < -np.pi:
        w_angle = 2 * np.pi + angle
    elif angle > np.pi:
        w_angle = angle - 2 * np.pi
    else:
        w_angle = angle

    return w_angle


def sign(a):
    if a >= 0:
        res = 1
    else:
        res = -1

    return res
