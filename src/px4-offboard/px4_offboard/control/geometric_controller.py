"""Geometric flight controller for the multilift quadrotors."""

import numpy as np
from numpy import linalg as LA

try:
    from px4_offboard.control.l1_adaptive import L1AdaptiveLaw
except ImportError:
    from .l1_adaptive import L1AdaptiveLaw


class Controller(L1AdaptiveLaw):
    """
    Geometric flight controller on SE(3) [1][2]
    [1] Lee, T., Leok, M. and McClamroch, N.H., 2010.
        Control of complex maneuvers for a quadrotor UAV using geometric methods on SE (3).
        arXiv preprint arXiv:1003.2005.
    [2] Lee, T., Leok, M. and McClamroch, N.H., 2010, December.
        Geometric tracking control of a quadrotor UAV on SE (3).
        In 49th IEEE conference on decision and control (CDC) (pp. 5420-5425). IEEE.
    """
    def __init__(self, uav_para, dt_sample):
        # Quadrotor's inertial parameters (mass, rotational inertia)
        self.m    = uav_para[0]
        self.J    = np.diag([uav_para[1], uav_para[2], uav_para[3]])
        self.nq   = int(uav_para[4]) # number of quadrotors
        # Unit direction vector free of coordinate
        self.ex   = np.array([[1, 0, 0]]).T
        self.ey   = np.array([[0, 1, 0]]).T
        self.ez   = np.array([[0, 0, 1]]).T
        # Gravitational acceleration
        self.g    = 9.81
        self.dt   = dt_sample

    def skew_sym(self, v):
        v_cross = np.array([
            [0, -v[2, 0], v[1, 0]],
            [v[2, 0], 0, -v[0, 0]],
            [-v[1, 0], v[0, 0], 0]]
        )
        return v_cross

    def vee_map(self, v):
        vect = np.array([[v[2, 1], v[0, 2], v[1, 0]]]).T
        return vect

    def trace(self, v):
        v_trace = v[0, 0] + v[1, 1] + v[2, 2]
        return v_trace

    def lowpass_filter(self,time_const,curr_i,prev_i):
        alpha       = self.dt/(self.dt+time_const)
        y_filter    = (1-alpha)*prev_i + alpha*curr_i
        return y_filter

    def q_2_rotation(self, q): # from body frame to inertial frame
        q = q/LA.norm(q) # normalization
        q0, q1, q2, q3 = q[0,0], q[1,0], q[2,0], q[3,0] # q0 denotes a scalar while q1, q2, and q3 represent rotational axes x, y, and z, respectively
        R = np.array([
                      [2 * (q0 ** 2 + q1 ** 2) - 1, 2 * q1 * q2 - 2 * q0 * q3,   2 * q0 * q2 + 2 * q1 * q3],
                      [2 * q0 * q3 + 2 * q1 * q2,   2 * (q0 ** 2 + q2 ** 2) - 1, 2 * q2 * q3 - 2 * q0 * q1],
                      [2 * q1 * q3 - 2 * q0 * q2,   2 * q0 * q1 + 2 * q2 * q3,   2 * (q0 ** 2 + q3 ** 2) - 1]
                     ])
        return R

    def geometric_ctrl(self,x,v_prev,a_lpf_prev,j_lpf_prev,ref_p,ref_v,ref_a,ref_j,ref_s,b1_d,sum_e,ctrl_gain):
        # Control gain variables
        self.kp   = np.diag([ctrl_gain[0,0], ctrl_gain[0,1], ctrl_gain[0,2]])
        self.kv   = np.diag([ctrl_gain[0,3], ctrl_gain[0,4], ctrl_gain[0,5]])
        self.ki   = np.diag([ctrl_gain[0,6], ctrl_gain[0,7], ctrl_gain[0,8]])
        self.kr   = np.diag([ctrl_gain[0,9], ctrl_gain[0,10], ctrl_gain[0,11]]) # control gain for attitude tracking error
        self.kw   = np.diag([ctrl_gain[0,12], ctrl_gain[0,13], ctrl_gain[0,14]])
        # Get the system state from the feedback
        p  = np.array([[x[0,0], x[1,0], x[2,0]]]).T
        v  = np.array([[x[3,0], x[4,0], x[5,0]]]).T
        q  = np.array([[x[6,0], x[7,0], x[8,0], x[9,0]]]).T
        Rb = self.q_2_rotation(q) # rotation matrix from body frame to inertial frame
        """
        Position controller
        """
        # Trajectory tracking errors
        ep = p - ref_p
        ev = v - ref_v
        sum_e += self.dt*ep # approximation of the error integration
        # Desired force in inertial frame for the norminal dynamics
        Fd = -np.matmul(self.kp, ep) - np.matmul(self.kv, ev) -self.ki@sum_e + self.m*self.g*self.ez + self.m*ref_a
        # Desired total thruster force fd
        fd = np.inner(Fd.T, np.transpose(np.matmul(Rb, self.ez))) # norminal total thrust projected into the current body z axis
        """
        Attitude controller
        """
        # Construct the desired rotation matrix (from body frame to inertial frame)
        b3c = Fd/LA.norm(Fd) # b3c = -A/norm(A), so A = -Fd
        b2c = np.matmul(self.skew_sym(b3c), b1_d)/LA.norm(np.matmul(self.skew_sym(b3c), b1_d)) # b2c = -C/norm(C), so C = skew_sym(b1_d)@b3c
        b1c = np.matmul(self.skew_sym(b2c), b3c)
        Rbd = np.hstack((b1c, b2c, b3c))
        # Compute the desired angular velocity and angular acceleration，see Appendix F in the 2nd version of [1] for details
        a      = (v-v_prev)/self.dt # acclearation based on 1st-order backward differentiation
        time_const  = 0.025 # used in the low-pass filter for the acceleration
        a_lpf  = self.lowpass_filter(time_const,a,a_lpf_prev)
        j      = (a_lpf-a_lpf_prev)/self.dt # jerk based on 1st-order backward differentiation
        time_const  = 0.03 # used in the low-pass filter for the jerk
        j_lpf  = self.lowpass_filter(time_const,j,j_lpf_prev)
        A      = -Fd
        dA     = np.matmul(self.kp, ev) + np.matmul(self.kv, (a_lpf-ref_a)) - self.m*ref_j
        ddA    = np.matmul(self.kp, (a_lpf-ref_a)) + np.matmul(self.kv, (j_lpf-ref_j)) - self.m*ref_s
        db3c   = -dA/LA.norm(A) + np.inner(A.T,dA.T)*A/(LA.norm(A)**3)
        ddb3c  = -ddA/LA.norm(A) + 2*np.inner(A.T,dA.T)*dA/(LA.norm(A)**3) + (LA.norm(dA)**2+np.inner(A.T,ddA.T))*A/(LA.norm(A)**3) - 3*np.inner(A.T,dA.T)**2*A/(LA.norm(A)**5)
        C      = np.matmul(self.skew_sym(b1_d),b3c)
        dC     = np.matmul(self.skew_sym(b1_d),db3c)
        ddC    = np.matmul(self.skew_sym(b1_d),ddb3c)
        db2c   = -dC/LA.norm(C) + np.inner(C.T,dC.T)*C/(LA.norm(C)**3)
        ddb2c  = -ddC/LA.norm(C) + 2*np.inner(C.T,dC.T)*dC/(LA.norm(C)**3) + (LA.norm(dC)**2+np.inner(C.T,ddC.T))*C/(LA.norm(C)**3) - 3*np.inner(C.T,dC.T)**2*C/(LA.norm(C)**5)
        db1c   = np.matmul(self.skew_sym(db2c),b3c) + np.matmul(self.skew_sym(b2c),db3c)
        ddb1c  = np.matmul(self.skew_sym(ddb2c),b3c) + 2*np.matmul(self.skew_sym(db2c),db3c) + np.matmul(self.skew_sym(b2c),ddb3c)
        dRbd   = np.hstack((db1c, db2c, db3c))
        ddRbd  = np.hstack((ddb1c, ddb2c, ddb3c))
        omegad = self.vee_map(np.matmul(Rbd.T,dRbd))
        # Bound the desired angular rate for stability concern
        if LA.norm(omegad)>=10:
            omegad = omegad/LA.norm(omegad)*10
        domegad= self.vee_map(np.matmul(Rbd.T,ddRbd)-LA.matrix_power(self.skew_sym(omegad),2))
        omega  = np.array([[x[10,0], x[11,0], x[12,0]]]).T
        # attitude tracking errors
        er  = 1/2*self.vee_map(np.matmul(Rbd.T, Rb) - np.matmul(Rb.T, Rbd))
        ew  = omega - np.matmul(Rb.T, np.matmul(Rbd, omegad))
        # desired control torque
        tau = -np.matmul(self.kr, er) - np.matmul(self.kw, ew) \
            + np.matmul(self.skew_sym(omega), np.matmul(self.J, omega)) \
            - np.matmul(self.J, (np.matmul(np.matmul(self.skew_sym(omega), Rb.T), np.matmul(Rbd, omegad)) \
                - np.matmul(Rb.T, np.matmul(Rbd, domegad))))
        # control input
        u   = np.vstack((fd,tau))

        return u, a_lpf, j_lpf, sum_e

    def L1_adaptive_law(self,xi,z_hat):
        return self.l1_adaptive_law(xi, z_hat)

    def system_ref(self, ref_a, ml, ref_al):
        # generate the reference state and control trajectories for tracking
        Fl_ref = ml*self.g*self.ez + ml*ref_al # 3-by-1 vector of the payload in {I}
        fl_ref = LA.norm(Fl_ref)
        F_ref  = self.m*self.g*self.ez + self.m*ref_a + Fl_ref/self.nq # 3-by-1 vector in {I}. Compared to F_d, F_ref does not require any feedback information
        f_ref  = LA.norm(F_ref)  # magnitude of F_ref in ideal case when R = R_d
        # reset the desired attitude to identity matrix and the desired angular velocity to zeros (since the attitude only experiences small changes in flight)
        qd     = np.array([[1,0,0,0]]).T
        omegad = np.zeros((3,1))
        M_ref  = np.zeros((3,1))

        return qd, omegad, f_ref, fl_ref, M_ref
