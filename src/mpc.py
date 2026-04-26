import os
import numpy as np
import scipy.linalg
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca


class MPC:
    """
    Nieliniowy MIMO Model Predictive Control dla dwukołowego robota.
    Wykorzystuje ciągły model kinematyki i dynamiki z integratorem ERK.
    """

    def __init__(self, N=10, dt=0.1):
        self.N = N
        self.dt = dt

        self.nx = 6  # [X, X_dot, Phi, Phi_dot, Psi, Psi_dot]
        self.nu = 2  # [a_L, a_R]

        # Parametry fizyczne robota (zaktualizowane do Twojego fizycznego modelu na biurku!)
        self.Mb = 1.045
        self.Mw = 0.1
        self.d = 0.1788
        self.R = 0.04
        self.L = 0.2
        self.Ix = 0.0011695
        self.Iz = 0.0027748
        self.Iy = 0.0019317
        self.Iwx = 0.5 * self.Mw * (self.R**2)
        self.Iwz = 0.0001
        self.g = 9.81

        # Ograniczenia przyspieszenia kół (Twarde limity fizyczne)
        self.u_min = -15.0  # Max przyśpieszenie w tył [m/s^2]
        self.u_max = 15.0   # Max przyśpieszenie w przód [m/s^2]

        # Dodatkowe ograniczenia
        # Maksymalna dopuszczalna prędkość liniowa kół [m/s]
        self.v_max = 3.0
        self.phi_max = 0.26  # Max pitch (ok. 15 stopni) [rad]

        # Zmienne na wagi
        self.Q_diag = np.ones(self.nx)
        self.R_diag = np.ones(self.nu) * 0.1

        self.solver = None

    def _setup_acados(self):
        ocp = AcadosOcp()

        # --- DEFINICJA MODELU CASADI ---
        model = AcadosModel()
        model.name = 'segway_nonlinear_mpc'

        # Symbole CasADi dla stanów i wejść
        x = ca.SX.sym('x', self.nx)
        u = ca.SX.sym('u', self.nu)

        model.x = x
        model.u = u

        # Rozpakowanie zmiennych dla czytelności
        X_pos = x[0]
        X_dot = x[1]
        Phi = x[2]
        Phi_dot = x[3]
        Psi = x[4]
        Psi_dot = x[5]

        a_L = u[0]
        a_R = u[1]

        # Zależności kinematyczne dla wejść systemu (przyśpieszenie bazy i przyśpieszenie obrotu)
        xpp_in = 0.5 * a_L + 0.5 * a_R
        psipp_in = - (1.0 / self.L) * a_L + (1.0 / self.L) * a_R

        # Nieliniowe równanie dynamiki wahadła na podstawie wyprowadzeń z eof.py
        den_phipp = 4*self.Iwx*self.R + 2*self.Iy*self.R + 2*self.Mb*self.R**3 + \
            4*self.Mb*self.R**2*self.d * \
            ca.cos(Phi) + 2*self.Mb*self.R*self.d**2 + 4*self.Mw*self.R**3

        phipp = (-4*self.Iwx*xpp_in + self.Ix*self.R*Psi_dot**2*ca.sin(2*Phi) - self.Iz*self.R*Psi_dot**2*ca.sin(2*Phi) +
                 2*self.Mb*self.R**2*self.d*Phi_dot**2*ca.sin(Phi) - 2*self.Mb*self.R**2*xpp_in +
                 2*self.Mb*self.R*self.d*self.g*ca.sin(Phi) - 2*self.Mb*self.R*self.d*xpp_in*ca.cos(Phi) -
                 4*self.Mw*self.R**2*xpp_in) / den_phipp

        # Model w formie różniczkowej ciągłej (x_dot = f(x, u))
        model.f_expl_expr = ca.vertcat(
            X_dot, xpp_in, Phi_dot, phipp, Psi_dot, psipp_in)

        ocp.model = model

        # Horyzont predykcji
        ocp.dims.N = self.N

        # --- KOSZT (LINEAR LEAST SQUARES) ---
        ocp.cost.cost_type = 'LINEAR_LS'
        ocp.cost.cost_type_e = 'LINEAR_LS'

        # Macierze decydujące o mapowaniu (Vx - mapuje x, Vu - mapuje u do funkcji kosztu y = Vx*x + Vu*u)
        ocp.cost.Vx = np.vstack(
            (np.eye(self.nx), np.zeros((self.nu, self.nx))))
        ocp.cost.Vu = np.vstack(
            (np.zeros((self.nx, self.nu)), np.eye(self.nu)))
        ocp.cost.Vx_e = np.eye(self.nx)

        # Wagi dla funkcji kosztu
        W = scipy.linalg.block_diag(np.diag(self.Q_diag), np.diag(self.R_diag))
        ocp.cost.W = W
        ocp.cost.W_e = np.diag(self.Q_diag)

        # Wartości referencyjne (cel: stany = 0, wejścia = 0)
        ocp.cost.yref = np.zeros(self.nx + self.nu)
        ocp.cost.yref_e = np.zeros(self.nx)

        # --- OGRANICZENIA ---
        # Sterowanie (u)
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.lbu = np.array([self.u_min, self.u_min])
        ocp.constraints.ubu = np.array([self.u_max, self.u_max])

        # Ograniczenia Stanów (x) -> Ograniczamy tylko Pitch (Phi)
        ocp.constraints.idxbx = np.array([2])
        ocp.constraints.lbx = np.array([-self.phi_max])
        ocp.constraints.ubx = np.array([self.phi_max])

        ocp.constraints.idxbx_e = np.array([2])
        ocp.constraints.lbx_e = np.array([-self.phi_max])
        ocp.constraints.ubx_e = np.array([self.phi_max])

        # Ograniczenia Prędkości Kół jako General Linear Constraints (C * x <= ug)
        # v_L = X_dot - (L/2)*Psi_dot
        # v_R = X_dot + (L/2)*Psi_dot
        C = np.zeros((2, self.nx))
        C[0, 1] = 1.0
        C[0, 5] = -self.L / 2.0  # Wzór na prędkość lewego koła
        C[1, 1] = 1.0
        C[1, 5] = self.L / 2.0  # Wzór na prędkość prawego koła

        ocp.constraints.C = C
        ocp.constraints.D = np.zeros((2, self.nu))
        ocp.constraints.lg = np.array([-self.v_max, -self.v_max])
        ocp.constraints.ug = np.array([self.v_max, self.v_max])

        ocp.constraints.C_e = C
        ocp.constraints.lg_e = np.array([-self.v_max, -self.v_max])
        ocp.constraints.ug_e = np.array([self.v_max, self.v_max])

        # Warunek początkowy x0 (aktualizowany na żywo podczas działania MPC)
        ocp.constraints.x0 = np.zeros(self.nx)

        # --- OPCJE SOLVERA ---
        ocp.solver_options.tf = self.N * self.dt
        ocp.solver_options.integrator_type = 'ERK'
        ocp.solver_options.nlp_solver_type = 'SQP_RTI'
        ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
        ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
        ocp.solver_options.print_level = 0

        # Kompilacja C-code
        self.solver = AcadosOcpSolver(ocp, json_file='acados_ocp.json')

    def setWeights(self, Q_diag, R_diag):
        """ Aktualizuje wagi Q i R. Setup z generowaniem C wykonywany jest dopiero po tym etapie. """
        self.Q_diag = np.array(Q_diag)
        self.R_diag = np.array(R_diag)

        if self.solver is None:
            self._setup_acados()
        else:
            # Optymalna aktualizacja na działającym solverze bez rekompilacji
            W = scipy.linalg.block_diag(
                np.diag(self.Q_diag), np.diag(self.R_diag))
            for i in range(self.N):
                self.solver.cost_set(i, 'W', W)
            self.solver.cost_set(self.N, 'W', np.diag(self.Q_diag))

    def update(self, current_state):
        if self.solver is None:
            return np.zeros(self.nu)

        x0 = np.array(current_state)

        # Przypisujemy aktualny stan z robota do zera jako punkt startowy
        self.solver.set(0, 'lbx', x0)
        self.solver.set(0, 'ubx', x0)

        # Rozwiązanie problemu
        status = self.solver.solve()

        if status == 0:
            # Bierzemy optymalny wektor z pierwszego kroku Horyzontu predykcji
            return self.solver.get(0, 'u')
        else:
            # Ratunkowe zwolnienie blokad gdy model wyjdzie poza constraints
            print(
                f"[MPC] Solver failed with status {status}. Reverting to zero command.")
            return np.zeros(self.nu)
