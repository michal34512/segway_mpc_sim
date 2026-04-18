import numpy as np
import scipy.sparse as sparse
import osqp


class MPC:
    """
    Pełny MIMO Model Predictive Control dla dwukołowego robota
    Zoptymalizowany pod solver OSQP (Embedded / Real-Time)
    """

    def __init__(self, A, B, N=10, dt=0.01):
        self.A = np.array(A)
        self.B = np.array(B)

        self.N = N
        self.dt = dt

        self.nx = self.A.shape[0]  # Liczba stanów (6)
        self.nu = self.B.shape[1]  # Liczba wejść (2)

        # Inicjalizacja wag w formacie macierzy rzadkich (sparse)
        self.Q = sparse.diags(np.ones(self.nx))
        self.R = sparse.diags(np.ones(self.nu) * 0.1)

        # Ograniczenia przyspieszenia kół (Twarde limity fizyczne)
        self.u_min = -15.0  # Max przyśpieszenie w tył [m/s^2]
        self.u_max = 15.0   # Max przyśpieszenie w przód [m/s^2]

        # Instancja solvera
        self.prob = osqp.OSQP()

        # Flaga sprawdzająca czy setup został już wykonany
        self._setup_done = False

        # Inicjalizacja układu macierzowego (setup solvera)
        self._setup_qp()

    def _setup_qp(self):
        """ Buduje duże macierze dla problemu QP z ograniczeniami na sterowanie i STANY """

        INF = np.inf  # Wartość oznaczająca brak limitu (nieskończoność)

        # ---------------------------------------------------------
        # DEFINICJA BEZPIECZNYCH GRANIC DLA STANÓW ROBOTA
        # Stany to: [X, X_dot, Phi, Phi_dot, Psi, Psi_dot]
        # X_dot odpowiada prędkości w przód (np. 1.5 m/s chroni przed rozkręceniem kół)
        # Phi ograniczamy twardo do +/- 0.26 radiana (ok. 15 stopni)
        # ---------------------------------------------------------
        self.x_min = np.array([-INF, -1.5, -0.26, -INF, -INF, -INF])
        self.x_max = np.array([INF,  1.5,  0.26,  INF,  INF,  INF])

        # --- 1. Koszt: Macierz P i wektor q ---
        P_x = sparse.kron(sparse.eye(self.N + 1), self.Q)
        P_u = sparse.kron(sparse.eye(self.N), self.R)
        self.P = sparse.block_diag([P_x, P_u]).tocsc()
        self.q = np.zeros((self.N + 1) * self.nx + self.N * self.nu)

        # --- 2. Dynamika systemu (Blok 1) ---
        Ax = sparse.kron(sparse.eye(self.N + 1), sparse.eye(self.nx)) - \
            sparse.kron(sparse.eye(self.N + 1, k=-1), self.A)
        Bu = sparse.kron(sparse.vstack(
            [sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]), -self.B)
        A_dyn = sparse.hstack([Ax, Bu])

        self.l_dyn = np.zeros((self.N + 1) * self.nx)
        self.u_dyn = np.zeros((self.N + 1) * self.nx)

        # --- 3. Ograniczenia dla sterowań U (Blok 2) ---
        A_ineq_u = sparse.hstack([
            sparse.csc_matrix((self.N * self.nu, (self.N + 1) * self.nx)),
            sparse.eye(self.N * self.nu)
        ])
        l_ineq_u = np.full(self.N * self.nu, self.u_min)
        u_ineq_u = np.full(self.N * self.nu, self.u_max)

        # --- 4. Ograniczenia dla stanów X (Blok 3 - NOWOŚĆ) ---
        # Wyciągamy stany x_1 do x_N (pomijamy x_0 by uniknąć problemu z infeasibility przy uderzeniu)
        A_ineq_x = sparse.hstack([
            # Zerowy blok dla x_0
            sparse.csc_matrix((self.N * self.nx, self.nx)),
            # Wyciągnięcie x_1 do x_N
            sparse.eye(self.N * self.nx),
            # Zerowy blok dla części U
            sparse.csc_matrix((self.N * self.nx, self.N * self.nu))
        ])

        l_ineq_x = np.tile(self.x_min, self.N)
        u_ineq_x = np.tile(self.x_max, self.N)

        # --- 5. Złączenie w jedną wielką macierz i wektory dla OSQP ---
        self.A_c = sparse.vstack([A_dyn, A_ineq_u, A_ineq_x]).tocsc()
        self.l = np.hstack([self.l_dyn, l_ineq_u, l_ineq_x])
        self.u = np.hstack([self.u_dyn, u_ineq_u, u_ineq_x])

        # Inicjalizacja OSQP
        if not self._setup_done:
            self.prob.setup(self.P, self.q, self.A_c, self.l,
                            self.u, warm_start=True, verbose=False)
            self._setup_done = True
        else:
            self.prob.update(Px=self.P.data, l=self.l, u=self.u)

    def setWeights(self, Q_diag, R_diag):
        """ Aktualizuje wagi Q i R, a następnie odświeża problem QP """
        self.Q = sparse.diags(Q_diag)
        self.R = sparse.diags(R_diag)
        self._setup_qp()  # Przebuduj układ po zmianie wag

    def update(self, current_state):
        x0 = np.array(current_state)

        # Krok 1: Wstrzykujemy aktualny stan robota jako początkowy warunek dynamiki x_0
        self.l[:self.nx] = x0
        self.u[:self.nx] = x0

        # Krok 2: Aktualizujemy limity w OSQP w czasie rzeczywistym
        self.prob.update(l=self.l, u=self.u)

        # Krok 3: Rozwiązanie optymalizacji
        res = self.prob.solve()

        if res.info.status == 'solved':
            # Z wektora rozwiązań z = [X, U] wyciągamy pierwsze u_0
            u_start_idx = (self.N + 1) * self.nx
            optimal_u = res.x[u_start_idx: u_start_idx + self.nu]
            return optimal_u
        else:
            # Gdy układ utraci stabilność, ratujemy się "puszczeniem" kół luzem
            return np.zeros(self.nu)
