#!/usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import osqp
import pandas as pd
import numpy as np
from scipy import sparse
import rospkg
import rospy
import math

class controller: 

  def __init__(self):

    rospy.init_node('mpc_node')
    self.rate = rospy.Rate(6) # 10hz
    self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10) # cmd
    self.sub = rospy.Subscriber("puzzlebot/odom", Odometry, self.callback) # cmd
    self.dt = 0.15

    self.Ad = np.eye(3)
    self.Bd = np.zeros((3,2))

    self.Q = sparse.diags([0.1, 0.1, 0.1])
    self.QN = self.Q
    self.R = 0.05*sparse.eye(2)
    self.start = False

    self.N = 10
    self.track = "trajectory_line"
    self.it = 0

    self.nx = 3
    self.nu = 2

    self.pose_hist = np.asarray([0,0,0])[np.newaxis,:]

    self.get_ref()

    xr = self.compute_ref()
    x0 = np.asarray([0.0,0,0])
    self.xmax = np.asarray([np.inf, np.inf, np.inf])
    self.xmin = np.asarray([-np.inf, -np.inf, -np.inf])
    self.umax = np.asarray([10, 4])
    self.umin = np.asarray([-10.0, -4.0])

    self.initi_optimizer(self.Q, self.QN, self.R, self.N, xr, x0, self.nx, self.nu, self.xmax, self.xmin, self.umax, self.umin)


  def quaternion_to_euler(self,data):

    x = data.x
    y = data.y
    z = data.z
    w = data.w

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return yaw_z

  def callback(self,data):
    self.start = True
    theta = self.quaternion_to_euler(data.pose.pose.orientation)
    self.xo = np.asarray([data.pose.pose.position.x, data.pose.pose.position.y,theta])

  def get_ref(self):

    rospack = rospkg.RosPack()
    self.path = rospack.get_path('mpc_controller')

    data = pd.read_csv(self.path + "/references/trajectory_s.csv")
    x = data.loc[:, "x"].to_numpy(copy=True)[:, np.newaxis]
    y = data.loc[:, "y"].to_numpy(copy=True)[:, np.newaxis]
    th = data.loc[:, "th"].to_numpy(copy=True)[:, np.newaxis]

    self.refs = np.hstack((x, y, th))
    self.nsim = self.refs.shape[0]
    aux_rows = np.repeat(self.refs[-1,:][np.newaxis,:],self.N,axis=0)

    self.refs = np.append(self.refs,aux_rows, axis=0)


  def compute_ref(self):
    return self.refs[self.it:self.it + self.N,:]

  def UpdateABC(self):
    self.Bd[0,0] = np.cos(self.xo[2])*self.dt
    self.Bd[1,0] = np.sin(self.xo[2])*self.dt
    self.Bd[2,1] = self.dt
    # self.Bd[0,0] = 1*self.dt
    # self.Bd[1,0] = 0*self.dt
    # self.Bd[2,1] = self.dt

  def update_optimizer(self, xr, x0):

    Q  = self.Q
    QN = self.QN
    N  = self.N
    nx = self.nx
    nu = self.nu

    # - linear objective
    q = []
    for i in range(0,N):
      q = np.hstack([q, -Q.dot(xr[i,:])])

    q = np.hstack([q, -QN.dot(xr[N-1,:]),np.zeros(N*nu)])

    # - linear dynamics
    Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-x0, np.zeros(N*nx)])
    ueq = leq

    # - OSQP constraints
    A = sparse.vstack([Aeq, self.Aineq], format='csc')
    l = np.hstack([leq, self.lineq])
    u = np.hstack([ueq, self.uineq])

    print("-------------------------------------------------")
    # print(l)
    # print(type(A))
    print("-------------------------------------------------")

    self.opt.update(q=q, Ax=A, l=l, u=u)
    # self.opt.update(q=q, l=l, u=u)

  def initi_optimizer(self, Q, QN, R, N, xr, x0, nx, nu, xmax, xmin, umax, umin):

    # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
    # - quadratic objective
    P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                          sparse.kron(sparse.eye(N), R)], format='csc')

    # - linear objective
    q = []
    for i in range(0,N):
      q = np.hstack([q, -Q.dot(xr[i,:])])

    q = np.hstack([q, -QN.dot(xr[N-1,:]),np.zeros(N*nu)])

    # - linear dynamics
    Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), self.Ad)
    Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), self.Bd)
    Aeq = sparse.hstack([Ax, Bu])
    leq = np.hstack([-x0, np.zeros(N*nx)])
    ueq = leq

    # - input and state constraints
    self.Aineq = sparse.eye((N+1)*nx + N*nu)
    self.lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
    self.uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

    # - OSQP constraints
    A = sparse.vstack([Aeq, self.Aineq], format='csc')
    l = np.hstack([leq, self.lineq])
    u = np.hstack([ueq, self.uineq])

    # Create an OSQP object
    self.opt = osqp.OSQP()
    # print(q.size)
    # print(l.size)
    print(type(A))

    # Setup workspace
    self.opt.setup(P, q, A, l, u, warm_start=True)

  def run(self):

    # Simulate in closed loop
    while not rospy.is_shutdown() and self.it < self.nsim:

        if self.start:
          # Solve
          self.UpdateABC()
          ref = self.compute_ref()
          # self.update_optimizer(ref,self.xo)
          self.initi_optimizer(self.Q, self.QN, self.R, self.N, ref,self.xo, self.nx, self.nu, self.xmax, self.xmin, self.umax, self.umin)
          res = self.opt.solve()

          # Check solver status
          if res.info.status != 'solved':
              raise ValueError('OSQP did not solve the problem!')

          # Apply first control input to the plant
          ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]

          print(ref[0,:])
          print(self.xo)

          self.pose_hist = np.append(self.pose_hist, self.xo[np.newaxis,:], axis=0)



          msg = Twist()
          msg.linear.x = ctrl[0]
          msg.angular.z = ctrl[1]

          self.pub.publish(msg)

          self.it += 1

          self.rate.sleep()

    pd.DataFrame(self.pose_hist).to_csv(self.path + '/data/sample.csv')

if __name__ == "__main__":
    aux = controller()
    aux.run()
