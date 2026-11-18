import numpy as np

# ----- Inputs -----

fastener_pos = np.array([[-5.0, -5.0], # x, z
                            [-5.0, 5.0],
                            [5.0, -5.0],
                            [5.0, -5.0]])

fastener_areas = np.array([10, 
                           10, 
                           10,
                           10])

n_f = len(fastener_areas)

F_x = 10
F_z = 10

D_2 = np.array([1,
                1,
                1,
                1])

t_2 = np.array([0.5,
                0.5,
                0.5,
                0.5])

#Find fastener cg
def calc_cg():
    x_cg = np.sum(np.multiply(fastener_areas, fastener_pos[:,0])) / np.sum(fastener_areas)

    z_cg = np.sum(np.multiply(fastener_areas, fastener_pos[:,1])) / np.sum(fastener_areas)

    print("CG position of the fasteners is ", x_cg, z_cg)

    return (x_cg, z_cg)


# Find forces on fasteners from net forces and moment
def calc_M_y(x_cg, z_cg):
    M_y = (F_x * z_cg) + (F_z * x_cg )

    print("M_y is ", M_y, "Nm")
    return M_y
    

# Find forces on fasteners from net forces and moment
def calc_forces(M_y, x_cg, z_cg):
    F_fx = F_x / n_f # incorrect if different areas for fasteners
    F_fz = F_z / n_f
    cg = np.array(x_cg, z_cg)
    pos = fastener_pos - cg
    r = np.linalg.norm(pos, axis=1)
    F_M_y_mag = M_y * ((fastener_areas * r) / np.sum(fastener_areas * np.square(r)))
    F_M_y_dir =  np.flip(pos / r[:, np.newaxis], 1)
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


# Program
x_cg, z_cg = calc_cg()
M_y = calc_M_y(x_cg=x_cg, z_cg=z_cg)
F_f = calc_forces(M_y, x_cg, z_cg)
s_f = calc_bearing_stress(F_f)
print('bearing stresses:\n', s_f)