import sim
import math

PORT = 19999
HANDLERS = (
    'MTB_joint1',
    'MTB_joint2',
    'MTB_joint3',
    'Dummy',
)

def connect(port):
    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if client_id == 0:
        print('Conectado a', port)
    else:
        print('No se pudo conectar')
    return client_id


def get_object_handler(client_id, handler_name):
    ret, object_handle = sim.simxGetObjectHandle(client_id, handler_name, sim.simx_opmode_blocking)
    return object_handle

def set_joint_target_position(client_id, joint_id, value):
    sim.simxSetJointTargetPosition(client_id, joint_id, value, sim.simx_opmode_oneshot)

def mover(client_id, handler, degree):
    radian = math.radians(degree)
    obj = get_object_handler(client_id, handler)
    set_joint_target_position(client_id, obj, radian)

