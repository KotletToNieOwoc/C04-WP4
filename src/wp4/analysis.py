import numpy as np
import math

# ----- Inputs -----

fastener_pos = np.array([[1, -5.0], # x, z
                            [1, 7.0],
                            [-1, -7.0],
                            [-1, 5.0]])

fastener_areas = np.array([10, 
                           10, 
                           10,
                           10])

n_f = len(fastener_areas)

# Applied forces
F_x = 10
F_z = 10
F_y = 10
M_z = 10

D_2 = 1

t_2 = 0.5

# Fastener diameters
D_fo = 1
D_fi = 0.5

# Wall thickness
t_3 = 0.3

# BEARING CHECK
# Calculate fastener cg
def calc_cg():
    x_cg = np.sum(np.multiply(fastener_areas, fastener_pos[:,0])) / np.sum(fastener_areas)
    z_cg = np.sum(np.multiply(fastener_areas, fastener_pos[:,1])) / np.sum(fastener_areas)

    print("CG position of the fasteners is ", x_cg, z_cg)

    return (x_cg, z_cg)


# Calculate forces on fasteners from moment
def calc_M_y(x_cg, z_cg):
    M_y = (F_x * z_cg) + (F_z * x_cg)

    print("M_y is ", M_y, "Nm")
    return M_y
    

# Find forces on fasteners from net forces and moment
def calc_in_plane_forces(M_y, x_cg, z_cg):
    F_fx = F_x / n_f # incorrect if different areas for fasteners
    F_fz = F_z / n_f
    cg = np.array(x_cg, z_cg)
    pos = fastener_pos - cg
    r = np.linalg.norm(pos, axis=1)
    F_M_y_mag = M_y * ((fastener_areas * r) / np.sum(fastener_areas * np.square(r)))
    F_M_y_dir = np.flip(pos / r[:, np.newaxis], 1)
    F_M_y_dir[:, 1] *= -1
    print('force from moment direction:\n', F_M_y_dir)

    F_M_y = F_M_y_mag[:, np.newaxis] * F_M_y_dir

    F_f_tot = F_M_y.copy()
    F_f_tot[:,0] += F_fx
    F_f_tot[:,1] += F_fz

    print('total force on fastener:\n', F_f_tot)
    return F_f_tot
    

def calc_bearing_stress(F_f):
    return np.linalg.norm(F_f, axis=1) / (D_2 * t_2)



# PULL THROUGH
# Find forces on fasteners from net forces and moment
def calc_out_plane_forces(M_z,x_cg):
    F_fy = F_y / n_f # incorrect with different fastener areas
    x_delta_pos = fastener_pos[:,0] - x_cg
    F_M_z = ((M_z * fastener_areas * x_delta_pos) / np.sum(fastener_areas * np.square(x_delta_pos)))
    F_f_tot = F_fy + F_M_z
    return F_f_tot


# Calculate shear stress on fasteners
def calc_pull_through_stress(F_pull_f):
    #A_contact = (math.pi / 4) * ( (D_fo ** 2) - (D_fi ** 2) )
    #normal_stress = F_f / contact_area
    A_shear_backup = math.pi * D_fi * t_2
    shear_stress_backup = F_pull_f / A_shear_backup

    A_shear_vehicle = math.pi * D_fi * t_3
    shear_stress_vehicle = F_pull_f / A_shear_vehicle

    return shear_stress_backup, shear_stress_vehicle

    

# Analysis
print("\n")
x_cg, z_cg = calc_cg()
M_y = calc_M_y(x_cg=x_cg, z_cg=z_cg)
F_plane_f = calc_in_plane_forces(M_y, x_cg, z_cg)
s_bear = calc_bearing_stress(F_plane_f)
F_pull_f = calc_out_plane_forces(M_z, x_cg)
s_shear_backup, s_shear_vehicle = calc_pull_through_stress(F_pull_f)

print('bearing stresses:\n', s_bear)
print('pull through shear stresses (on backup wall):\n', s_shear_backup)
print('pull through shear stresses (on vehicle wall):\n', s_shear_vehicle)
