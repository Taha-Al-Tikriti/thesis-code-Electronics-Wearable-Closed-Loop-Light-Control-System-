import numpy as np
import control as ct

# _______________________________________________________________
# ...............................................................
# 1. System Parameters & Grid Def

Ts = 0.1  # Sampling period (100 ms)
s = ct.TransferFunction.s

# Nominal CIE 1931 color-mixing matrix (3in-3out MIMO plant)
M_xyz_nominal = np.array([
    [0.15, 0.08, 0.05],
    [0.08, 0.22, 0.02],
    [0.00, 0.03, 0.25]
])

# Fixed 1st-order Pade approx (T=100ms delay) + Sensor 1st-order lag (tau=50ms)
# Pade: (1 - 0.05s)/(1 + 0.05s) | Sensor: 1/(0.05s + 1)
g_dyn = ((-0.05 * s + 1) / (0.05 * s + 1)) * (1 / (0.05 * s + 1))
G_dyn_ss = ct.tf2ss(g_dyn) 
G_mimo_dyn = ct.append(G_dyn_ss, G_dyn_ss, G_dyn_ss)

# Weighting filters
# todo: tweak w_s if low-freq tracking is sluggish
w_s = (s / 1.5 + 1) / (s + 0.001)         # Sensitivity filter (tracking)
w_ks = (0.05 * s + 0.1) / (0.01 * s + 1)   # Control effort filter (actuator penalty)
w_t = (s + 1) / (s / 10 + 1)              # complementary sensitivity 

W_S = ct.append(w_s, w_s, w_s)
W_KS = ct.append(w_ks, w_ks, w_ks)
W_T  = ct.append(w_t, w_t, w_t)

# Define ambient scale lookup grid (5 discrete levels)
gain_scales = [0.80, 0.90, 1.00, 1.10, 1.20] # tested 0.7-1.3 but gamma blew up
controller_grid = []


print("="*60)
print(" SYNTHESIZING DISCRETE H-INFINITY LOOKUP TABLE ")
print("="*60)

for idx, scale in enumerate(gain_scales):
    # Scale static color matrix for ambient shift
    M_scaled = M_xyz_nominal * scale
    G_gain = ct.ss([], [], [], M_scaled)
    G = G_gain * G_mimo_dyn

    # Formulate generalized plant and solve H-inf synthesis
    P = ct.augw(G, w1=W_S, w2=W_KS, w3=W_T)
    K_c, _, gamma, _ = ct.hinfsyn(P, nmeas=3, ncon=3)

    # Bilinear / Tustin discretization (Ts = 0.1s)
    K_d = ct.sample_system(K_c, Ts=Ts, method='bilinear')
    controller_grid.append(K_d)

    print(f"Index {idx} | Ambient Scale: {scale:.2f}x | Gamma (γ): {gamma:.4f} | States: {K_d.nstates}")


# -------------------------------------------------------------
# C++ Header Auto Generator


num_states = controller_grid[0].A.shape[0]

print("\n" + "="*60)
print(" C++ HEADER GENERATED OUTPUT ")
print("=" * 60 + "\n")

# Defines
print(f"#define NUM_LOOKUP_MODELS {len(gain_scales)}")
print(f"#define K_STATES {num_states}")
print(f"#define K_INPUTS 3")
print(f"#define K_OUTPUTS 3\n")

# Matrix arrays
print("const float A_GRID[NUM_LOOKUP_MODELS][K_STATES][K_STATES] = {")
for k in controller_grid:
    print("  {")
    for row in k.A:
        print("    {" + ", ".join([f"{val:.6e}f" for val in row]) + "},")
    print("  },")
print("};\n")

print("const float B_GRID[NUM_LOOKUP_MODELS][K_STATES][K_INPUTS] = {")
for k in controller_grid:
    print("  {")
    for row in k.B:
        print("    {" + ", ".join([f"{val:.6e}f" for val in row]) + "},")
    print("  },")
print("};\n")


print("const float C_GRID[NUM_LOOKUP_MODELS][K_OUTPUTS][K_STATES] = {")
for k in controller_grid:
    print("  {")
    for row in k.C:
        print("    {" + ", ".join([f"{val:.6e}f" for val in row]) + "},")
    print("  },")
print("};\n")

print("const float D_GRID[NUM_LOOKUP_MODELS][K_OUTPUTS][K_INPUTS] = {")
for k in controller_grid:
    print("  {")
    for row in k.D:
        print("    {" + ", ".join([f"{val:.6e}f" for val in row]) + "},")
    print("  },")
print("};")
# ## ## ## ## ## ## OUTPUT ## ## ## ## ## ## ## # 
#Index 0 | Ambient Scale: 0.80x | Gamma (γ): 1.8012 | States: 15
#Index 1 | Ambient Scale: 0.90x | Gamma (γ): 1.7131 | States: 15
#Index 2 | Ambient Scale: 1.00x | Gamma (γ): 1.6479 | States: 15
#Index 3 | Ambient Scale: 1.10x | Gamma (γ): 1.5985 | States: 15
#Index 4 | Ambient Scale: 1.20x | Gamma (γ): 1.5602 | States: 15
# promissing gamma values:)
# followed by the matrices
