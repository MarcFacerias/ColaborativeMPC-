import numpy as np

def checkEnd(x, maps):

    status = False
    for i,agent in enumerate(x):

        if not agent is None:
            if np.isclose(agent[0,-3],maps[i].TrackLength[maps[i].lane],atol=0.15) or (agent[0,-3] > maps[i].TrackLength[maps[i].lane]) :
                status = True
                return status
        else:
            return False

    return status

def Regression(x, u, lamb):
    """Estimates linear system dynamics
    x, u: date used in the regression
    lamb: regularization coefficient
    """

    # Want to solve W^* = argmin sum_i ||W^T z_i - y_i ||_2^2 + lamb ||W||_F,
    # with z_i = [x_i u_i] and W \in R^{n + d} x n
    Y = x[2:x.shape[0], :]
    X = np.hstack((x[1:(x.shape[0] - 1), :], u[1:(x.shape[0] - 1), :]))

    Q = np.linalg.inv(np.dot(X.T, X) + lamb * np.eye(X.shape[1]))
    b = np.dot(X.T, Y)
    W = np.dot(Q, b)

    A = W.T[:, 0:6]
    B = W.T[:, 6:8]

    ErrorMatrix = np.dot(X, W) - Y
    ErrorMax = np.max(ErrorMatrix, axis=0)
    ErrorMin = np.min(ErrorMatrix, axis=0)
    Error = np.vstack((ErrorMax, ErrorMin))

    return A, B, Error



# EA: Modified for taking also the desired velocity
def Curvature(s, map):
    """curvature and desired velocity computation
    s: curvilinear abscissa at which the curvature has to be evaluated
    PointAndTangent: points and tangent vectors defining the map (these quantities are initialized in the map object)
    """

    PointAndTangent = map.PointAndTangent[:,:,map.lane]
    TrackLength = PointAndTangent[-1,3]+PointAndTangent[-1,4]

    # In case on a lap after the first one
    while (s > TrackLength):
        s = s - TrackLength

    if s < 0: s = 0

    # Given s \in [0, TrackLength] compute the curvature
    # Compute the segment in which system is evolving
    index = np.all([[s >= PointAndTangent[:, 3]], [s < PointAndTangent[:, 3] + PointAndTangent[:, 4]]], axis=0)

    i = int(np.where(np.squeeze(index))[0]) #EA: this works
    #i = np.where(np.squeeze(index))[0]     #EA: this does not work

    curvature = PointAndTangent[i, 5]
    return curvature
    # return 0

def get_ey(s_local, map, sm = 1.0):
    """curvature and desired velocity computation
    s: curvilinear abscissa at which the curvature has to be evaluated
    PointAndTangent: points and tangent vectors defining the map (these quantities are initialized in the map object)
    """
    HW = np.zeros(s_local.shape[0])
    for idx,s in enumerate(s_local):
        PointAndTangent = map.PointAndTangent[:,:,map.lane]
        TrackLength = PointAndTangent[-1,3]+PointAndTangent[-1,4]

        # In case on a lap after the first one
        while (s > TrackLength):
            s = s - TrackLength

        if s < 0: s = 0

        index = np.all([[s >= PointAndTangent[:, 3]], [s < PointAndTangent[:, 3] + PointAndTangent[:, 4]]], axis=0)

        i = int(np.where(np.squeeze(index))[0]) #EA: this works

        HW[idx] = map.halfWidth[i]*sm
    return HW


def GBELLMF(x, a, b, c):

    tmp = ((x - c)/a)**2

    if (tmp == 0) and (b == 0):
        y = 0.5
    elif (tmp == 0) and (b < 0):
        y = 0
    else:
        #tmp = tmp**b
        tmp = np.power(tmp,b)
        y = 1.0/(1 + tmp)
    
    return y

def wrap(angle):
    if angle < -np.pi:
        w_angle = 2 * np.pi + angle
    elif angle > np.pi:
        w_angle = angle - 2 * np.pi
    else:
        w_angle = angle

    return w_angle

def initialise_agents(data,Hp,dt,map, accel_rate=0):
    n_agents = len(data)
    agents = np.zeros((Hp+1,n_agents,2))
    u_pred  = [None] * n_agents
    x_pred  = [None] * n_agents

    for id, el in enumerate(data):
        x_pred[id],u_pred[id] = predicted_vectors_generation(Hp, el, dt, map[id], accel_rate)
        agents[:,id,:] = x_pred[id][:,-2:]

    return agents,x_pred,u_pred

def predicted_vectors_generation(Hp, x0, dt, map, accel_rate = 0):
    # We need a prediction of the states for the start-up proces of the controller (To instantiate the LPV variables)
    # [vx vy psidot y_e thetae theta s x y ]

    Vx      = np.zeros((Hp+1, 1))
    Vx[0]   = x0[0]
    S       = np.zeros((Hp+1, 1))
    S[0]    = 0
    Vy      = np.zeros((Hp+1, 1))
    Vy[0]   = x0[1]
    W       = np.zeros((Hp+1, 1))
    W[0]    = x0[2]
    Ey      = np.zeros((Hp+1, 1))
    Ey[0]   = x0[3]
    Epsi    = np.zeros((Hp+1, 1))
    Epsi[0] = x0[4]

    aux = map.getGlobalPosition(S[0], Ey[0])
    Theta = np.zeros((Hp+1, 1))
    Theta[0] = aux[2]
    X = np.zeros((Hp+1, 1))
    X[0] = aux[0]
    Y = np.zeros((Hp+1, 1))
    Y[0] = aux[1]

    Accel   = 1.0

    for i in range(0, Hp):
        Vy[i+1]      = x0[1]
        W[i+1]       = x0[2]
        Ey[i+1]      = x0[3]
        Epsi[i+1]    = x0[4]

    Accel   = Accel + np.array([ (accel_rate * i) for i in range(0, Hp)])

    for i in range(0, Hp):
        Vx[i+1]    = Vx[i] + Accel[i] * dt
        S[i+1]      = S[i] + Vx[i] * dt
        X[i+1], Y[i+1], Theta[i+1] = map.getGlobalPosition(S[i], Ey[i])

    xx  = np.hstack([ Vx, Vy, W,Ey, Epsi, Theta ,S ,X,Y]) # [vx vy psidot y_e thetae theta s x y slack1 slack2]
    uu = np.zeros(( Hp, 2 ))
    return xx, uu


