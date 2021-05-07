import sim
import math
import time

SERVER_ADDRESS = '127.0.0.1'
PORT = 10009
# PORT = 19999

# Cerrar todas las conecciones por si acaso
sim.simxFinish(-1) 

# Conectarse a Coppelia Sim
client_id = sim.simxStart(SERVER_ADDRESS, PORT, True, True, 5000, 5)

if client_id == -1:
    print('No se pudo conectar')

# Obtener el axis
AXIS_NAME = 'MTB_axis1#1'
ret,axis=sim.simxGetObjectHandle(client_id, AXIS_NAME, sim.simx_opmode_blocking)

# Mover el axis
value_degree = 180
value_radians = math.radians(value_degree)
sim.simxSetJointTargetPosition(client_id, axis, value_radians, sim.simx_opmode_oneshot)
time.sleep(1)
