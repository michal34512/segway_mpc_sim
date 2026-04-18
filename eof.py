from sympy.printing.pycode import pycode
import sympy as sp
from sympy.physics.mechanics import dynamicsymbols, init_vprinting

# Wymuszenie drukowania wektorowego (ładne kropki zamiast d/dt)
init_vprinting(use_unicode=True)

# ==========================================
# 1. DEFINICJE ZMIENNYCH I PARAMETRÓW
# ==========================================
t = sp.Symbol('t')

# Parametry korpusu (B) i kół (W), momenty bezwładności i wymiary
M_B, R, d, g = sp.symbols('M_B R d g', positive=True)
I_xx, I_yy, I_zz = sp.symbols('I_xx I_yy I_zz', positive=True)
M_W, L = sp.symbols('M_W L', positive=True)
I_Wx, I_Wz = sp.symbols('I_Wx I_Wz', positive=True)

# Współrzędne uogólnione (używamy dynamicsymbols dla ładnego formatowania)
x, phi, yaw = dynamicsymbols('x phi yaw')

# Prędkości uogólnione (pochodne po czasie)
xdot = sp.diff(x, t)
phidot = sp.diff(phi, t)
yawdot = sp.diff(yaw, t)

# ==========================================
# 2. ENERGIA KINETYCZNA TRANSLACJI KORPUSU (T_B^T)
# ==========================================
x_cdot = xdot + d * phidot * sp.cos(phi) + R * phidot
z_cdot = -d * phidot * sp.sin(phi)
v_c_sq = x_cdot**2 + z_cdot**2

T_B_T = sp.Rational(1, 2) * M_B * v_c_sq
T_B_T_simp = sp.trigsimp(sp.expand(T_B_T))

# ==========================================
# 3. ENERGIA KINETYCZNA ROTACJI KORPUSU (T_B^R)
# ==========================================
T_B_R = sp.Rational(1, 2) * I_yy * phidot**2 + sp.Rational(1, 2) * \
    (I_xx * sp.sin(phi)**2 + I_zz * sp.cos(phi)**2) * yawdot**2
T_B_R_simp = sp.trigsimp(sp.expand(T_B_R))

# ==========================================
# 4. KINEMATYKA KÓŁ (prędkości kątowe prawego i lewego koła)
# ==========================================
thetadot_R = (xdot + (L * yawdot) / 2) / R + phidot
thetadot_L = (xdot - (L * yawdot) / 2) / R + phidot

# ==========================================
# 5. ENERGIA KINETYCZNA TRANSLACJI KÓŁ (T_W^T)
# ==========================================
T_W_T = sp.Rational(1, 2) * M_W * R**2 * (thetadot_R**2 + thetadot_L**2)
T_W_T_simp = sp.simplify(sp.expand(T_W_T))

# ==========================================
# 6. ENERGIA KINETYCZNA ROTACJI KÓŁ (T_W^R)
# ==========================================
T_W_R = sp.Rational(1, 2) * I_Wx * (thetadot_R**2 +
                                    thetadot_L**2) + sp.Rational(1, 2) * I_Wz * yawdot**2
T_W_R_simp = sp.simplify(sp.expand(T_W_R))

# ==========================================
# 7. CAŁKOWITA ENERGIA KINETYCZNA
# ==========================================
T_total = T_B_T_simp + T_B_R_simp + T_W_T_simp + T_W_R_simp


# ==========================================
# 8. CAŁKOWITA ENERGIA POTENCJALNA
# ==========================================
V_total = M_B * g * d * sp.cos(phi)

# Lagrangian
L = sp.simplify(sp.expand(T_total - V_total))

# phi
dL_dphidot = sp.diff(L, phidot)
d_dt_dL_dphidot = sp.diff(dL_dphidot, t)
dL_dphi = sp.diff(L, phi)
EL_phi = sp.simplify(d_dt_dL_dphidot - dL_dphi)

# x
dL_dxdot = sp.diff(L, xdot)
d_dt_dL_dxdot = sp.diff(dL_dxdot, t)
dL_dx = sp.diff(L, x)
EL_x = sp.simplify(d_dt_dL_dxdot - dL_dx)

# yaw
dL_dyawdot = sp.diff(L, yawdot)
d_dt_dL_dyawdot = sp.diff(dL_dyawdot, t)
dL_dyaw = sp.diff(L, yaw)
EL_yaw = sp.simplify(d_dt_dL_dyawdot - dL_dyaw)


print("Euler-Lagrange'a dla phi:")
sp.pprint(EL_phi)
print("Euler-Lagrange'a dla x:")
sp.pprint(EL_x)
print("Euler-Lagrange'a dla yaw:")
sp.pprint(EL_yaw)


# # ==========================================
# # 9. SYMULACJA DYNAMIKI I GENERATOR KODU PYTHON
# # ==========================================
# print("\nRozwiązuję układ równań (to może zająć kilkanaście sekund)...")

# # Definiujemy wejścia z silników tak jak w Twoim przykładzie (tau1 - lewy, tau2 - prawy)
# tau1, tau2 = sp.symbols('tau1 tau2')
# F_x = (tau1 + tau2) / R
# tau_yaw = (tau1 - tau2) * L / (2 * R)

# phi_dd = sp.diff(phi, t, 2)
# yaw_dd = sp.diff(yaw, t, 2)
# x_dd = sp.diff(x, t, 2)

# eq_phi = sp.Eq(EL_phi, 0)
# eq_x = sp.Eq(EL_x, F_x)
# eq_yaw = sp.Eq(EL_yaw, tau_yaw)

# solutions = sp.solve((eq_phi, eq_x, eq_yaw), (x_dd, phi_dd, yaw_dd))


# # --- MAGIA GENEROWANIA KODU ---
# Mb, Mw, d_sym, R_sym, L_sym, g_sym = sp.symbols('Mb Mw d R L g')
# Ix, Iy, Iz, Iwx, Iwz = sp.symbols('Ix Iy Iz Iwx Iwz')
# phi_val, phip, psip, xp = sp.symbols('phi phip psip xp')
# subs_dict = {
#     M_B: Mb, M_W: Mw, d: d_sym, R: R_sym, L: L_sym, g: g_sym,
#     I_xx: Ix, I_yy: Iy, I_zz: Iz, I_Wx: Iwx, I_Wz: Iwz,
#     phi: phi_val,
#     phidot: phip,
#     yawdot: psip,
#     xdot: xp
# }
# def generate_python_snippet(var_name, expression):
#     expr_subbed = expression.subs(subs_dict)
#     num, den = sp.fraction(sp.cancel(expr_subbed))
#     num_str = pycode(num).replace('math.', '')
#     den_str = pycode(den).replace('math.', '')
#     print(f"        # computation of {var_name}")
#     print(f"        den_{var_name} = {den_str}")
#     print(f"        {var_name} = ({num_str}) / den_{var_name}\n")
# print("\n" + "# "*25)
# print("# SKOPIUJ PONIŻSZY KOD DO SWOJEJ KLASY:")
# print("# "*25)
# generate_python_snippet('phipp', solutions[phi_dd])
# generate_python_snippet('xpp', solutions[x_dd])
# generate_python_snippet('psipp', solutions[yaw_dd])

# ==========================================
# 9. SYMULACJA DYNAMIKI DLA SILNIKÓW KROKOWYCH
# ==========================================
print("\nRozwiązuję równanie dla silników krokowych...")

# 1. Definiujemy wejścia z silników jako zmienne, którymi sterujemy
# (xpp_in to zadane przyśpieszenie liniowe, psipp_in to zadane przyśpieszenie skręcania)
xpp_in, psipp_in = sp.symbols('xpp_in psipp_in')

phi_dd = sp.diff(phi, t, 2)
yaw_dd = sp.diff(yaw, t, 2)
x_dd = sp.diff(x, t, 2)

# 2. Podstawiamy nasze zadane przyśpieszenia w miejsce x_dd i yaw_dd w równaniu na phi
EL_phi_stepper = EL_phi.subs({x_dd: xpp_in, yaw_dd: psipp_in})
eq_phi = sp.Eq(EL_phi_stepper, 0)

# 3. Rozwiązujemy tylko jedno równanie - opadanie korpusu
solutions = sp.solve(eq_phi, phi_dd)

# --- MAGIA GENEROWANIA KODU ---

# 4. Definiujemy czyste symbole do wyplutego kodu (bez 't')
Mb, Mw, d_sym, R_sym, L_sym, g_sym = sp.symbols('Mb Mw d R L g')
Ix, Iy, Iz, Iwx, Iwz = sp.symbols('Ix Iy Iz Iwx Iwz')
phi_val, phip, psip, xp = sp.symbols('phi phip psip xp')

# Słownik zamian
subs_dict = {
    M_B: Mb, M_W: Mw, d: d_sym, R: R_sym, L: L_sym, g: g_sym,
    I_xx: Ix, I_yy: Iy, I_zz: Iz, I_Wx: Iwx, I_Wz: Iwz,
    phi: phi_val,
    phidot: phip,
    yawdot: psip,
    xdot: xp
}


def generate_stepper_python_snippet(var_name, expression):
    expr_subbed = expression.subs(subs_dict)

    # Rozdzielamy ułamek na licznik i mianownik
    num, den = sp.fraction(sp.cancel(expr_subbed))

    # Przekształcamy na czysty tekst Pythona i wywalamy prefix "math."
    num_str = pycode(num).replace('math.', '')
    den_str = pycode(den).replace('math.', '')

    print(f"        # computation of {var_name}")
    print(f"        den_{var_name} = {den_str}")
    print(f"        {var_name} = ({num_str}) / den_{var_name}\n")


print("\n" + "# "*25)
print("# SKOPIUJ PONIŻSZY KOD DO SWOJEJ KLASY:")
print("# "*25)

# Wypluwamy gotowy kod.
# solutions to w tym wypadku lista jednoelementowa, więc bierzemy solutions[0]
generate_stepper_python_snippet('phipp', solutions[0])
