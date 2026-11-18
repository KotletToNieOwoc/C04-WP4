import numpy as np

fastener_positions = np.array([[-5.0, -5.0], # x, z
                              [-5.0, 0.0],
                              [0.0, -5.0],
                              [0.0, 0.0]])

fastener_areas = np.array([10, 
                           10, 
                           10,
                           10])

x_cg = np.sum(np.multiply(fastener_areas, fastener_positions[:,0])) / np.sum(fastener_areas)

z_cg = np.sum(np.multiply(fastener_areas, fastener_positions[:,1])) / np.sum(fastener_areas)

print(x_cg, z_cg)