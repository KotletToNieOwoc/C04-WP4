import numpy as np
import math

# ----- Inputs -----

# Position of each fastener with respect to origin at centroid of backup plate [m]
fastener_pos = np.array([[1, -5.0], # x, z
                            [1, 7.0],
                            [-1, -7.0],
                            [-1, 5.0]])

# Area of each fastener [m^2]
fastener_areas = np.array([10, 
                           10, 
                           10,
                           10])

# Applied forces [N]
F_x = 10
F_z = 10
F_y = 10
M_z = 10

# Backup plate fastener hole diameter [m]
D_2 = 1

# Backup plate thickness [m]
t_2 = 0.5

# Fastener diameters [m]
D_fo = 1
D_fi = 0.5

# Wall thickness [m]
t_3 = 0.3


# Number of fasteners (calculated automatically)
n_f = len(fastener_areas)



# ----- Analysis calculation functions -----

# ----- Bearing check -----
# 4.5 - Calculate fastener cg
def calc_cg():
    x_cg = np.sum(np.multiply(fastener_areas, fastener_pos[:,0])) / np.sum(fastener_areas)
    z_cg = np.sum(np.multiply(fastener_areas, fastener_pos[:,1])) / np.sum(fastener_areas)

    print("CG position of the fasteners is ", x_cg, z_cg)

    return (x_cg, z_cg)


# 4.6 - Calculate resultant moment when force vector on backup plate centroid is moved to fastener cg
def calc_M_y(x_cg, z_cg):
    M_y = (F_x * z_cg) + (F_z * x_cg)

    print("M_y is ", M_y, "Nm")
    return M_y


# 4.6 - Find forces on fasteners from net forces and moment
def calc_in_plane_forces(M_y, x_cg, z_cg):
    # Force from in-plane applied forces
    F_fx = F_x / n_f # incorrect if different areas of fasteners
    F_fz = F_z / n_f

    # Force from moment M_y
    cg_pos = np.array(x_cg, z_cg)
    pos = fastener_pos - cg_pos
    moment_arm = np.linalg.norm(pos, axis=1)
    F_M_magnitude = M_y * ((fastener_areas * moment_arm) / np.sum(fastener_areas * np.square(moment_arm)))
    F_M_unit_vector = np.flip(pos / moment_arm[:, np.newaxis], 1)
    F_M_unit_vector[:, 1] *= -1

    # Combine magnitude and direction of the force vector
    F_M = F_M_magnitude[:, np.newaxis] * F_M_unit_vector

    # Add up the forces
    F_f_tot = F_M.copy()
    F_f_tot[:,0] += F_fx
    F_f_tot[:,1] += F_fz

    print('total force on fasteners:\n', F_f_tot)
    return F_f_tot


# 4.7 - Calculate bearing stress
def calc_bearing_stress(F_f):
    return np.linalg.norm(F_f, axis=1) / (D_2 * t_2)



# ----- Pull through check -----
# 4.8 - Calculate forces on fasteners from net forces and moment
def calc_out_plane_forces(M_z,x_cg):
    F_fy = F_y / n_f # incorrect with different fastener areas
    x_delta_pos = fastener_pos[:,0] - x_cg
    F_M_z = ((M_z * fastener_areas * x_delta_pos) / np.sum(fastener_areas * np.square(x_delta_pos)))
    F_f_tot = F_fy + F_M_z
    return F_f_tot


# 4.9 - Calculate shear stress due to pull forces
def calc_pull_through_stress(F_pull_f):
    #A_contact = (math.pi / 4) * ( (D_fo ** 2) - (D_fi ** 2) )
    #normal_stress = F_f / contact_area
    A_shear_backup = math.pi * D_fi * t_2
    shear_stress_backup = F_pull_f / A_shear_backup

    A_shear_vehicle = math.pi * D_fi * t_3
    shear_stress_vehicle = F_pull_f / A_shear_vehicle

    return shear_stress_backup, shear_stress_vehicle



# ----- Analysis program -----
np.set_printoptions(precision=2)
print("\n")
x_cg, z_cg = calc_cg()
M_y = calc_M_y(x_cg=x_cg, z_cg=z_cg)
F_plane_f = calc_in_plane_forces(M_y, x_cg, z_cg)
s_bear = calc_bearing_stress(F_plane_f)
F_pull_f = calc_out_plane_forces(M_z, x_cg)
s_shear_backup, s_shear_vehicle = calc_pull_through_stress(F_pull_f)

print('----------------------- Analysis results -----------------------')
print('Bearing stresses:\n', s_bear)
print('Pull through shear stresses (on backup wall):\n', s_shear_backup)
print('Pull through shear stresses (on vehicle wall):\n', s_shear_vehicle)
print('----------------------------------------------------------------')