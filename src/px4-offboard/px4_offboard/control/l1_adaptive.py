"""L1 adaptive compensation helpers."""

import numpy as np
from numpy import linalg as LA
from scipy import linalg as sLA


class L1AdaptiveLaw:
    """Piecewise-constant L1 adaptive law shared by controllers."""

    def l1_adaptive_law(self, xi, z_hat):
        z = np.array([[xi[3, 0], xi[4, 0], xi[5, 0]]]).T
        q = np.array([[xi[6, 0], xi[7, 0], xi[8, 0], xi[9, 0]]]).T
        rb = self.q_2_rotation(q)
        b = 1 / self.m * rb @ self.ez
        brp = np.hstack((1 / self.m * rb @ self.ex, 1 / self.m * rb @ self.ey))
        bbar = np.hstack((b, brp))
        a_s = np.array([[-5, -5, -5]])
        a_s = np.diag(a_s[0])
        phi = LA.inv(a_s) @ (sLA.expm(self.dt * a_s) - np.identity(3))
        mu = sLA.expm(self.dt * a_s) @ (z_hat - z)
        d_hat = -LA.inv(bbar) @ LA.inv(phi) @ mu
        dm_hat = np.reshape(d_hat[0, 0], (1, 1))
        dum_hat = np.reshape(d_hat[1:3, 0], (2, 1))
        return dm_hat, dum_hat, a_s
