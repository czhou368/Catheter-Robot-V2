import numpy as np
import math
from scipy.integrate import solve_ivp
from scipy.optimize import fsolve


def hat(input):

    input = np.squeeze(input)

    return np.array(
        [[0, -input[2], input[1]], [input[2], 0, -input[0]], [-input[1], input[0], 0]]
    )


def getShape(y_cell):
    # y is nx12, get the shape
    segs = len(y_cell)

    p = []
    R = []
    g = []

    for i in range(segs):
        y = y_cell[i]
        ny = y.shape[0]

        p.append(y[:, 0:3].transpose())
        # print(p)
        R.append(np.reshape(y[:, 3:12].transpose(), (3, 3, ny)))
        # print(np.reshape(y[:, 3:12].transpose(), (3, 3, ny)))
        pp = np.reshape(p[i], (3, 1, ny))
        base = np.array([[0, 0, 0, 1]])  # Shape (1,4)
        base = np.repeat(
            base[:, :, np.newaxis], ny, axis=2
        )  # Repeat along the 3rd dimension

        # print(np.concatenate((np.concatenate((R[i], pp), axis=1), base), axis=0)[:, :, 0])
        g.append(np.concatenate((np.concatenate((R[i], pp), axis=1), base), axis=0))

    return p, R, g


# Create a catheter robot v1 object


class CatheterRobotV1:
    def __init__(self):

        # Unit: N, mm
        self.p0 = np.array([0, 0, 0])
        self.R0 = np.eye(3)

        # Catheter parameters
        self.dc_out = 5.3
        self.dc_in = 3
        self.l1 = 170  # Catheter channel part (partially rigid)
        self.l2 = 50  # Bending part (flexible)
        self.l3 = 5  # Straight part (rigid)

        # Nitinol tube parameters
        self.dn_out = 2.8
        self.dn_in = 2.5
        self.l4 = 40  # Nitinol tube part (flexible)
        self.l5 = 5  # Straight end (rigid)

        self.total_length = self.l1 + self.l2 + self.l3 + self.l4 + self.l5

        # Young's modules & torsional stiffness (N/mm^2)
        self.E1 = 30 * 1e3
        self.E2 = 2 * 1e3
        self.E3 = 60 * 1e3
        self.E4 = 2 * 1e3
        self.E5 = 60 * 1e3

        self.G1 = 20 * 1e3
        self.G2 = 1 * 1e3
        self.G3 = 25 * 1e3
        self.G4 = 1 * 1e3
        self.G5 = 25 * 1e3

        self.I1 = math.pi / 64.0 * (self.dc_out**4 - self.dc_in**4)
        self.J1 = 2 * self.I1
        self.I4 = math.pi / 64.0 * (self.dn_out**4 - self.dn_in**4)
        self.J4 = 2 * self.I4

        # Robot bending stiffness
        self.K1 = np.diag([self.E1 * self.I1, self.E1 * self.I1, self.G1 * self.J1])
        self.K2 = np.diag([self.E2 * self.I1, self.E2 * self.I1, self.G2 * self.J1])
        self.K3 = np.diag([self.E3 * self.I1, self.E3 * self.I1, self.G3 * self.J1])
        self.K4 = np.diag([self.E4 * self.I4, self.E4 * self.I4, self.G4 * self.J4])
        self.K5 = np.diag([self.E5 * self.I4, self.E5 * self.I4, self.G5 * self.J4])

        self.r1 = np.array([[1, 0, 0]]).transpose()
        self.r2 = np.array([[-1, 0, 0]]).transpose()

        self.radii1 = 0.25 * (self.dc_out + self.dc_in)  # Radii of cather wire
        self.radii4 = 0.25 * (self.dn_out + self.dn_in)  # Radii of nitinol tube wire

        self.ri1 = [self.radii1 * self.r1, self.radii1 * self.r2]
        self.ri4 = [self.radii4 * self.r1, self.radii4 * self.r2]

        self.u_star = np.array([[0, 0, 0]]).transpose()

        self.loads = np.zeros((3, 3))  # f1_body, f4_body, F_endpoint

        self.ini_guess = np.zeros((10, 1))
        self.ini_sol = self.ini_guess

        self.state_traj = []

        self.actuation = np.array([0, 0, 0, 0])

    def fk_shooting(self, actuation=None, loads=None, ini_guess=None):
        if actuation is not None:
            self.actuation = actuation
        if loads is not None:
            self.loads = loads
        if ini_guess is not None:
            self.ini_guess = ini_guess

        # Solve the forward kinematics using the shooting method
        # self.state_traj = []
        # print(self.ini_guess.shape)

        ini10_sol, info, ier, msg = fsolve(
            self.residual_disp,
            self.ini_guess.flatten(),
            full_output=True,
            # xtol=1e-12,
            maxfev=500,
        )

        if ier == 1:
            print("Solution found:", ini10_sol)
        else:
            print("Solution not found:", msg)

        res, shape, results = self.residual_disp_last(ini10_sol)

        return shape, ini10_sol, res, results

    def ode_T(self, s, y, tension, f_body, ri, Ki):
        # print(y.shape)
        y = np.reshape(y, (18, 1))
        R = np.reshape(y[3:12], (3, 3))
        u = np.reshape(y[12:15], (3, 1))
        n = np.reshape(y[15:18], (3, 1))
        # tension = y[18:22]
        v = np.array([[0, 0, 1]]).transpose()

        A = 0
        B = 0
        G = 0
        H = 0
        a = 0
        b = 0
        Ai = []
        Bi = []
        ai = []
        bi = []
        dpi_b = []

        for i in range(2):
            # print(np.matmul(hat(u), np.reshape(ri[i], (3, 1))))
            dpi_b.append(hat(u) @ np.reshape(ri[i], (3, 1)) + v)
            # print(dpi_b[i])
            # print(np.linalg.norm(dpi_b[i]) ** 3)
            Ai.append(
                -tension[i]
                * (hat(dpi_b[i]) @ hat(dpi_b[i]))
                / (np.linalg.norm(dpi_b[i]) ** 3)
            )

            Bi.append(hat(ri[i]) @ Ai[i])
            A = A + Ai[i]
            B = B + Bi[i]
            G = G - Ai[i] @ hat(ri[i])
            H = H - Bi[i] @ hat(ri[i])
            ai.append(Ai[i] @ (hat(u) @ dpi_b[i]))
            bi.append(hat(ri[i]) @ np.reshape(ai[i], (3, 1)))
            a = a + ai[i]
            b = b + bi[i]

        # formulate the diff equation
        le = np.zeros((3, 1))
        # print(u)
        # print(self.u_star)
        du = np.linalg.inv(H + Ki) @ (
            -hat(u) @ Ki @ np.reshape((u - self.u_star), (3, 1))
            - hat(v) @ R.transpose() @ np.reshape(n, (3, 1))
            - R.transpose() @ le
            - b
        )
        # print(du)
        dn = -R @ (a + G @ du) - np.reshape(f_body, (3, 1))
        # print(dn)

        ys = np.vstack((R @ v, np.reshape(R @ hat(u), (9, 1)), du, dn))
        # print(ys.shape)
        ys = ys.flatten()
        return ys

    def state_trans(self, y1, tension, ri, K1, K2):
        # print(y1.shape)
        # everything below is at s=L (end of the rod)
        p1 = y1[0:3, :]

        R1 = np.reshape(y1[3:12, :], (3, 3))
        u1 = y1[12:15, :]

        n1 = y1[15:18, :]
        v = np.array([[0, 0, 1]]).transpose()

        m1 = K1 @ R1 @ (u1 - self.u_star)
        Ft = np.zeros((3, 1))
        Mt = np.zeros((3, 1))
        dpi = []

        for i in range(2):

            dpi.append(R1 @ (hat(u1) @ ri[i] + v))
            # print(tension[i] * dpi[i])
            norm = np.linalg.norm(dpi[i])
            if norm == 0:
                norm = 1
            Ft = Ft + tension[i] * dpi[i] / norm
            Mt = Mt + tension[i] * hat(R1 @ ri[i]) @ dpi[i] / norm

        p2 = p1
        R2 = R1
        n2 = n1 + Ft
        m2 = m1 + Mt
        u2 = R2.T @ np.linalg.inv(K2) @ m2 + self.u_star
        y2 = np.vstack((p2, np.reshape(R2, (9, 1)), u2, n2))
        # print(m2.shape)
        return y2, m2, n2

    def residual_disp(self, ini10):
        # print(time.time())
        ini_18 = np.vstack(
            (
                self.p0.reshape(3, 1),
                np.reshape(self.R0, (9, 1)),
                np.reshape(ini10[0:6], (6, 1)),
            )
        )
        # print(ini_18)
        tension1 = ini10[6:8]
        tension2 = ini10[8:10]
        # print(tension1)
        # print(tension2)
        # define different parts
        # print(tension1 + tension2)
        ode_disp1 = lambda s, y: self.ode_T(
            s, y, tension1 + tension2, self.loads[0], self.ri1, self.K1
        )
        ode_disp2 = lambda s, y: self.ode_T(
            s, y, tension1 + tension2, self.loads[0], self.ri1, self.K2
        )
        ode_disp3 = lambda s, y: self.ode_T(
            s, y, tension1 + tension2, self.loads[0], self.ri1, self.K3
        )
        ode_disp4 = lambda s, y: self.ode_T(
            s, y, tension2, self.loads[1], self.ri4, self.K4
        )
        ode_disp5 = lambda s, y: self.ode_T(
            s, y, tension2, self.loads[1], self.ri4, self.K5
        )

        # solve ivps
        s1_end = self.l1
        s2_end = s1_end + self.l2
        s3_end = s2_end + self.l3
        s4_end = s3_end + self.l4
        s5_end = s4_end + self.l5

        solution1 = solve_ivp(ode_disp1, [0, s1_end], ini_18.squeeze(), method="DOP853")
        _, y1_sol = solution1.t, solution1.y

        y2_ini, _, _ = self.state_trans(
            np.expand_dims(y1_sol[:, -1], axis=1), [0, 0], self.ri1, self.K1, self.K2
        )

        solution2 = solve_ivp(
            ode_disp2, [s1_end, s2_end], y2_ini.squeeze(), method="DOP853"
        )
        _, y2_sol = solution2.t, solution2.y

        y3_ini, _, _ = self.state_trans(
            np.expand_dims(y2_sol[:, -1], axis=1), [0, 0], self.ri1, self.K2, self.K3
        )
        solution3 = solve_ivp(
            ode_disp3, [s2_end, s3_end], y3_ini.squeeze(), method="DOP853"
        )
        _, y3_sol = solution3.t, solution3.y

        y4_ini, _, _ = self.state_trans(
            np.expand_dims(y3_sol[:, -1], axis=1), tension1, self.ri1, self.K3, self.K4
        )
        solution4 = solve_ivp(
            ode_disp4, [s3_end, s4_end], y4_ini.squeeze(), method="DOP853"
        )
        _, y4_sol = solution4.t, solution4.y

        y5_ini, _, _ = self.state_trans(
            np.expand_dims(y4_sol[:, -1], axis=1), [0, 0], self.ri4, self.K4, self.K5
        )
        solution5 = solve_ivp(
            ode_disp5, [s4_end, s5_end], y5_ini.squeeze(), method="DOP853"
        )
        _, y5_sol = solution5.t, solution5.y

        # force/moment at end of the rod
        _, m_end, n_end = self.state_trans(
            np.expand_dims(y5_sol[:, -1], axis=1), tension2, self.ri4, self.K5, self.K5
        )

        # length agree
        pc_total = np.zeros((2, 1))
        pn_total = np.zeros((2, 1))

        y_sol_c = np.vstack(
            (y1_sol.transpose(), y2_sol.transpose(), y3_sol.transpose())
        )
        y_sol_n = np.vstack(
            (
                y1_sol.transpose(),
                y2_sol.transpose(),
                y3_sol.transpose(),
                y4_sol.transpose(),
                y5_sol.transpose(),
            )
        )

        n_c = y_sol_c.shape[0]
        n_n = y_sol_n.shape[0]
        pc_c = y_sol_c[:, 0:3].transpose()
        pc_n = y_sol_n[:, 0:3].transpose()
        Rc_c = np.reshape(y_sol_c[:, 3:12].transpose(), (3, 3, n_c))
        Rc_n = np.reshape(y_sol_n[:, 3:12].transpose(), (3, 3, n_n))

        # original length
        Lc = self.l1 + self.l2 + self.l3
        Ln = self.l1 + self.l2 + self.l3 + self.l4 + self.l5

        for j in range(2):
            # print(self.ri1[j].shape)

            pc_i = pc_c + np.reshape(
                np.matmul(Rc_c.transpose(2, 0, 1), self.ri1[j]).transpose(1, 2, 0),
                (3, n_c),
            )
            pn_i = pc_n + np.reshape(
                np.matmul(Rc_n.transpose(2, 0, 1), self.ri4[j]).transpose(1, 2, 0),
                (3, n_n),
            )

            # print(pc_i.shape)
            pc_total[j] = np.sum(
                np.linalg.norm(np.diff(pc_i, n=1, axis=1), ord=2, axis=0)
            )
            pn_total[j] = np.sum(
                np.linalg.norm(np.diff(pn_i, n=1, axis=1), ord=2, axis=0)
            )

        res = np.vstack(
            (
                n_end - np.reshape(self.loads[2], (3, 1)),
                m_end - 0,
                pc_total - (Lc - self.actuation[0:2].reshape(2, 1)),
                pn_total - (Ln - self.actuation[2:].reshape(2, 1)),
            )
        )

        return res.flatten()

    def residual_disp_last(self, ini10):
        # print("----------------")
        ini_18 = np.vstack(
            (
                self.p0.reshape(3, 1),
                np.reshape(self.R0, (9, 1)),
                np.reshape(ini10[0:6], (6, 1)),
            )
        )
        # print(ini_18)
        tension1 = ini10[6:8]
        tension2 = ini10[8:10]
        # print(tension1)
        # print(tension2)
        # define different parts
        # print(tension1 + tension2)
        ode_disp1 = lambda s, y: self.ode_T(
            s, y, tension1 + tension2, self.loads[0], self.ri1, self.K1
        )
        ode_disp2 = lambda s, y: self.ode_T(
            s, y, tension1 + tension2, self.loads[0], self.ri1, self.K2
        )
        ode_disp3 = lambda s, y: self.ode_T(
            s, y, tension1 + tension2, self.loads[0], self.ri1, self.K3
        )
        ode_disp4 = lambda s, y: self.ode_T(
            s, y, tension2, self.loads[1], self.ri4, self.K4
        )
        ode_disp5 = lambda s, y: self.ode_T(
            s, y, tension2, self.loads[1], self.ri4, self.K5
        )

        # solve ivps
        s1_end = self.l1
        s2_end = s1_end + self.l2
        s3_end = s2_end + self.l3
        s4_end = s3_end + self.l4
        s5_end = s4_end + self.l5

        solution1 = solve_ivp(ode_disp1, [0, s1_end], ini_18.squeeze())
        s1_sol, y1_sol = solution1.t, solution1.y

        # print(s1_sol.shape)

        y2_ini, _, _ = self.state_trans(
            np.expand_dims(y1_sol[:, -1], axis=1), [0, 0], self.ri1, self.K1, self.K2
        )

        solution2 = solve_ivp(ode_disp2, [s1_end, s2_end], y2_ini.squeeze())
        s2_sol, y2_sol = solution2.t, solution2.y

        y3_ini, _, _ = self.state_trans(
            np.expand_dims(y2_sol[:, -1], axis=1), [0, 0], self.ri1, self.K2, self.K3
        )
        solution3 = solve_ivp(ode_disp3, [s2_end, s3_end], y3_ini.squeeze())
        s3_sol, y3_sol = solution3.t, solution3.y

        y4_ini, _, _ = self.state_trans(
            np.expand_dims(y3_sol[:, -1], axis=1), tension1, self.ri1, self.K3, self.K4
        )
        solution4 = solve_ivp(ode_disp4, [s3_end, s4_end], y4_ini.squeeze())
        s4_sol, y4_sol = solution4.t, solution4.y

        y5_ini, _, _ = self.state_trans(
            np.expand_dims(y4_sol[:, -1], axis=1), [0, 0], self.ri4, self.K4, self.K5
        )
        solution5 = solve_ivp(ode_disp5, [s4_end, s5_end], y5_ini.squeeze())
        s5_sol, y5_sol = solution5.t, solution5.y

        # force/moment at end of the rod
        _, m_end, n_end = self.state_trans(
            np.expand_dims(y5_sol[:, -1], axis=1), tension2, self.ri4, self.K5, self.K5
        )

        # length agree
        pc_total = np.zeros((2, 1))
        pn_total = np.zeros((2, 1))

        y_sol_c = np.vstack(
            (y1_sol.transpose(), y2_sol.transpose(), y3_sol.transpose())
        )
        y_sol_n = np.vstack(
            (
                y1_sol.transpose(),
                y2_sol.transpose(),
                y3_sol.transpose(),
                y4_sol.transpose(),
                y5_sol.transpose(),
            )
        )

        n_c = y_sol_c.shape[0]
        n_n = y_sol_n.shape[0]
        pc_c = y_sol_c[:, 0:3].transpose()
        pc_n = y_sol_n[:, 0:3].transpose()
        Rc_c = np.reshape(y_sol_c[:, 3:12].transpose(), (3, 3, n_c))
        Rc_n = np.reshape(y_sol_n[:, 3:12].transpose(), (3, 3, n_n))

        # original length
        Lc = self.l1 + self.l2 + self.l3
        Ln = self.l1 + self.l2 + self.l3 + self.l4 + self.l5

        for j in range(2):
            # print(self.ri1[j].shape)

            pc_i = pc_c + np.reshape(
                np.matmul(Rc_c.transpose(2, 0, 1), self.ri1[j]).transpose(1, 2, 0),
                (3, n_c),
            )
            pn_i = pc_n + np.reshape(
                np.matmul(Rc_n.transpose(2, 0, 1), self.ri4[j]).transpose(1, 2, 0),
                (3, n_n),
            )

            # print(pc_i.shape)
            pc_total[j] = np.sum(
                np.linalg.norm(np.diff(pc_i, n=1, axis=1), ord=2, axis=0)
            )
            pn_total[j] = np.sum(
                np.linalg.norm(np.diff(pn_i, n=1, axis=1), ord=2, axis=0)
            )

        res = np.vstack(
            (
                n_end - np.reshape(self.loads[2], (3, 1)),
                m_end - 0,
                pc_total - (Lc - self.actuation[0:2].reshape(2, 1)),
                pn_total - (Ln - self.actuation[2:].reshape(2, 1)),
            )
        )

        cell_all = [
            y1_sol.transpose(),
            y2_sol.transpose(),
            y3_sol.transpose(),
            y4_sol.transpose(),
            y5_sol.transpose(),
        ]

        p, R, g = getShape(cell_all)
        y_sol = np.vstack(
            (
                y1_sol.transpose(),
                y2_sol.transpose(),
                y3_sol.transpose(),
                y4_sol.transpose(),
                y5_sol.transpose(),
            )
        )

        s_sol = np.vstack(
            (
                np.expand_dims(s1_sol, 1),
                np.expand_dims(s2_sol, 1),
                np.expand_dims(s3_sol, 1),
                np.expand_dims(s4_sol, 1),
                np.expand_dims(s5_sol, 1),
            )
        )

        shape = y_sol[:, 0:3].transpose()

        results = {
            "p": p,
            "R": R,
            "g": g,
            "y_sol": y_sol,
            "s_sol": s_sol,
        }

        return res.flatten(), shape, results


if __name__ == "__main__":
    robot = CatheterRobotV1()
    robot.actuation = np.array([3, -3, 0, 0])
    shape, ini10_sol, res, results = robot.fk_shooting()
    # print("Shape:", shape)
    # print("Result:", results['g'])
