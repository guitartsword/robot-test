import sim

JOINT = (
    'MTB_joint1',
    'MTB_joint2',
)


def connect(port):
    sim.simxFinish(-1)
    client_id = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if client_id == 0:
        print('Conectado a', port)
    else:
        print('No se pudo conectar')
    return client_id

def move(client_id, joint_id, valor):
    #Definir
    print(type(client_id), type(valor))
    ret,joint2=sim.simxGetObjectHandle(client_id, JOINT[joint_id],sim.simx_opmode_blocking)
    sim.simxSetJointTargetPosition(client_id, joint2, valor, sim.simx_opmode_oneshot)
    print(client_id, valor, joint2)
