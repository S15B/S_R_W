import pybullet as p
import time
import pybullet_data
import numpy as np
import matplotlib.pyplot as plt


# Инициализация PyBullet
# p.connect(p.GUI)
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Создание плоскости
p.loadURDF("plane.urdf")

# Устанавливаем положение и ориентацию камеры
camera_distance = 2.5  # Расстояние от камеры до целевой точки
camera_pitch = 0  # Угол наклона камеры (в градусах)
camera_yaw = 0     # Угол поворота камеры (в градусах)
position = [1, 0, 1.4]  # Точка, на которую направлена камера

p.resetDebugVisualizerCamera(camera_distance, camera_yaw, camera_pitch, position)

dt = 1/240      # pybullet simulation step 

# Размерности 
radius, height = 0.04, 0.06                         # Цилиндр
width_leg, depth_leg, length_leg = 0.07, 0.06, 0.5  # Плечо
length_platform = 0.25                              # Bi
a = np.array([[0.7, 2.5], [2, 1.1], [0, 1]]) # Координаты неподвижных платформ

cyl_rot = p.getQuaternionFromEuler([np.pi/2, 0, 0])     # Поворот цилиндра, чтобы выглядил красивым кругом
zero_rot = p.getQuaternionFromEuler([0, 0, 0])
base_position = [1, 0, 1.4]

# Создание визуальной и коллизионной форм для 6 ног поршней
leg_visual_shape, leg_collision_shape = [], []
for i in range(6):
    leg_visual_shape.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_leg/2], rgbaColor=[0.53, 0.8, 0.92, 1]))
    leg_collision_shape.append(p.createCollisionShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_leg/2]))

# Создание визуальной и коллизионной форм для 3 внутренних ног платформы
plat_leg_visual_shape, plat_leg_collision_shape = [], []
for i in range(3):
    plat_leg_visual_shape.append(p.createVisualShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_platform/2], rgbaColor=[0.24, 0.7, 0.53, 1]))
    plat_leg_collision_shape.append(p.createCollisionShape(p.GEOM_BOX, halfExtents=[width_leg/2, depth_leg/2, length_platform/2]))

# Создание визуальной и коллизионной форм для 6 вращетельных кругов
joint_collision_shape, joint_visual_shape = [], []
for i in range(7):
    joint_visual_shape.append(p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height, rgbaColor=[0,93, 0.93, 0.83, 1]))
    joint_collision_shape.append(p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height))

# Расчет для начального положения
h1 = ((base_position[0]- a[0][0])**2 + (base_position[2] + length_platform - a[0][1])**2)**(1/2)
g1 = (base_position[0] - a[0][0])

h2 = ((base_position[0] + 3**(1/2)/2*length_platform - a[1][0])**2 + (base_position[2] -1/2*length_platform - a[1][1])**2)**(1/2)
g2 = (-base_position[0] - 3**(1/2)/2*length_platform + a[1][0])

h3 = ((base_position[0] - 3**(1/2)/2*length_platform - a[2][0])**2 + (base_position[2] -1/2*length_platform - a[2][1])**2)**(1/2)
g3 = (base_position[0] - 3**(1/2)/2*length_platform - a[2][0])

# Создание трёх неподвижных тел
# База - крутящийся шарнир. Соединения - 2 плеча, связанных призматическим соединением

anchor_id_1 = p.createMultiBody(
    baseMass=1,                          
    basePosition=[a[0][0], 0, a[0][1]],      
    baseCollisionShapeIndex=joint_collision_shape[0],          
    baseVisualShapeIndex=joint_visual_shape[0],
    baseOrientation=cyl_rot,
    linkMasses=[1]*2,
    linkCollisionShapeIndices=[-1]*2,
    linkVisualShapeIndices=leg_visual_shape[0:2],
    linkPositions=[[0, 0, 0], [0, 0, -length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, np.pi + np.arcsin(g1/h1)])] + [zero_rot],
    linkInertialFramePositions=[[0, 0, 0]] * 2,
    linkInertialFrameOrientations=[[zero_rot]] * 2,
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_FIXED, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, 1, 0], [0, 0, 1]] 
)
anchor_id_2 = p.createMultiBody(
    baseMass=1,                          
    basePosition=[a[1][0], 0, a[1][1]],      
    baseCollisionShapeIndex=joint_collision_shape[1],          
    baseVisualShapeIndex=joint_visual_shape[1],
    baseOrientation=cyl_rot,
    linkMasses=[1]*2,
    linkCollisionShapeIndices=[-1]*2,
    linkVisualShapeIndices=leg_visual_shape[2:4],
    linkPositions=[[0, 0, 0], [0, 0, -length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, np.pi/2 - np.arccos(g2/h2)])] + [zero_rot] ,
    linkInertialFramePositions=[[0, 0, 0]] * 2,
    linkInertialFrameOrientations=[[zero_rot]] * 2,
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_FIXED, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, 1, 0], [0, 0, 1]] 
)
anchor_id_3 = p.createMultiBody(
    baseMass=1,                          
    basePosition=[a[2][0], 0, a[2][1]],      
    baseCollisionShapeIndex=joint_collision_shape[2],          
    baseVisualShapeIndex=joint_visual_shape[2],
    baseOrientation=cyl_rot,
    linkMasses=[1]*2,
    linkCollisionShapeIndices=[-1]*2,
    linkVisualShapeIndices=leg_visual_shape[4:6],
    linkPositions=[[0, 0, 0], [0, 0, -length_leg/2]],
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, -np.pi/2 + np.arccos(g3/h3)])] + [zero_rot] ,
    linkInertialFramePositions=[[0, 0, 0]] * 2,
    linkInertialFrameOrientations=[[zero_rot]] * 2,
    linkParentIndices=[0, 1],
    linkJointTypes=[p.JOINT_FIXED, p.JOINT_PRISMATIC],
    linkJointAxis=[[0, 1, 0], [0, 0, 1]] 
)

# Присоединение приводов к неподвижной системе координат
constraint_id_1 = p.createConstraint(
    parentBodyUniqueId=anchor_id_1, 
    parentLinkIndex=-1,
    childBodyUniqueId=-1,
    childLinkIndex=-1,
    childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    jointType=p.JOINT_POINT2POINT,
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, 0],  
    childFramePosition=[a[0][0], 0, a[0][1]]
)
constraint_id_2 = p.createConstraint(
    parentBodyUniqueId=anchor_id_2, 
    parentLinkIndex=-1,
    childBodyUniqueId=-1,
    childLinkIndex=-1,
    childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    jointType=p.JOINT_POINT2POINT,
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, 0],  
    childFramePosition=[a[1][0], 0, a[1][1]]
)
constraint_id_3 = p.createConstraint(
    parentBodyUniqueId=anchor_id_3, 
    parentLinkIndex=-1,
    childBodyUniqueId=-1,
    childLinkIndex=-1,
    childFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
    jointType=p.JOINT_POINT2POINT,
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, 0],  
    childFramePosition=[a[2][0], 0, a[2][1]]
)

# Создание платформы
platform_id = p.createMultiBody(
    baseMass=0.01,
    baseCollisionShapeIndex=joint_collision_shape[3],
    baseVisualShapeIndex=joint_visual_shape[3],
    basePosition=base_position,
    baseOrientation=cyl_rot,
    linkMasses=[0.01]*6,
    linkCollisionShapeIndices=[-1]*3 + joint_collision_shape[4:7],
    linkVisualShapeIndices=plat_leg_visual_shape + joint_visual_shape[4:7],
    linkPositions=[[0, length_platform/2, 0], 
                   [length_platform*np.sqrt(3)/4, -length_platform/4, 0],
                   [-length_platform*np.sqrt(3)/4, -length_platform/4, 0]] +
                   [[0, 0, length_platform/2]] * 3,
    linkOrientations=[p.getQuaternionFromEuler([-np.pi/2, 0, 0]), 
                      p.getQuaternionFromEuler([-np.pi/2, 0, -np.pi*2/3]),
                      p.getQuaternionFromEuler([-np.pi/2, 0, np.pi*2/3])] +
                      [cyl_rot] * 3,
    linkInertialFramePositions=[[0, 0, 0]] * 6,
    linkInertialFrameOrientations=[[zero_rot]] * 6,
    linkParentIndices=[0, 0, 0, 1, 2, 3],
    linkJointTypes=[p.JOINT_FIXED] * 3 + [p.JOINT_REVOLUTE] * 3,
    linkJointAxis=[[0, 0, 0]] * 3 + [[0, 0, 1]] * 3
)

p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.POSITION_CONTROL, targetPosition=h1, force=500)
p.setJointMotorControl2(anchor_id_2, 1, controlMode=p.POSITION_CONTROL, targetPosition=h2, force=500)
p.setJointMotorControl2(anchor_id_3, 1, controlMode=p.POSITION_CONTROL, targetPosition=h3, force=500)

# На стартовую позицию
for i in range(1, 240):
    p.stepSimulation()
    #time.sleep(dt)

p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(anchor_id_2, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(anchor_id_3, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)

p.setJointMotorControl2(platform_id, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(platform_id, 3, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(platform_id, 5, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)


# Соединение приводов с платформой
constraint_id_4 = p.createConstraint(
    parentBodyUniqueId=anchor_id_1, 
    parentLinkIndex=1,
    childBodyUniqueId=platform_id,
    childLinkIndex=1,
    parentFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi]),
    childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    jointType=p.JOINT_FIXED, 
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, length_leg/2],  
    childFramePosition=[0, 0, 0]  
)
constraint_id_5 = p.createConstraint(
    parentBodyUniqueId=anchor_id_2, 
    parentLinkIndex=1,
    childBodyUniqueId=platform_id,
    childLinkIndex=3,
    parentFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi]),
    childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    jointType=p.JOINT_FIXED, 
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, length_leg/2],  
    childFramePosition=[0, 0, 0]
)
constraint_id_6 = p.createConstraint(
    parentBodyUniqueId=anchor_id_3, 
    parentLinkIndex=1,
    childBodyUniqueId=platform_id,
    childLinkIndex=5,
    parentFrameOrientation=p.getQuaternionFromEuler([0, 0, np.pi]),
    childFrameOrientation=p.getQuaternionFromEuler([np.pi/2, 0, 0]),
    jointType=p.JOINT_FIXED, 
    jointAxis=[0, 0, 0],  
    parentFramePosition=[0, 0, length_leg/2],  
    childFramePosition=[0, 0, 0]  
)

# Координаты точек концах платформы
b = np.array([[0, length_platform], 
              [3**(1/2)/2*length_platform, -1/2*length_platform], 
              [-3**(1/2)/2*length_platform, -1/2*length_platform]])

# Создание массивов для графиков
t = np.array([0])
position = np.array([[p.getBasePositionAndOrientation(platform_id)[0][0], 
                      p.getBasePositionAndOrientation(platform_id)[0][2], 
                      p.getEulerFromQuaternion(p.getBasePositionAndOrientation(platform_id)[1])[1]]])
legs = np.array([[p.getJointState(anchor_id_1, 1)[0], 
                  p.getJointState(anchor_id_2, 1)[0],
                  p.getJointState(anchor_id_3, 1)[0]]])
legs_velocity = np.array([[p.getJointState(anchor_id_1, 1)[1], 
                           p.getJointState(anchor_id_2, 1)[1],
                           p.getJointState(anchor_id_3, 1)[1]]])

calculate_position = np.array([position[0]])
calculate_legs_velocity = np.array([[0, 0, 0]])

# Заданная скорость
u = np.array([0.1, -0.1, 0.1])

# Симуляция
for j in range(10*240):
    t = np.append(t, t[-1]+dt)

# Обратный якобиан
    J_inv = np.zeros((3, 3))
    R = np.array([
        [np.cos(position[-1][2]), -np.sin(position[-1][2])],
        [np.sin(position[-1][2]),  np.cos(position[-1][2])]])
    
    for i in range(3):
        BiC = b[i] @ R 
        B_i = BiC + position[-1][0:2]

        n_i = B_i - a[i]
        n_i = n_i / np.linalg.norm(n_i)  
        
        J_inv[i, 0] = n_i[0]
        J_inv[i, 1] = n_i[1]
        J_inv[i, 2] = n_i[0] * BiC[1] - n_i[1] * BiC[0]

    dq = J_inv @ (u)
    calculate_legs_velocity = np.vstack([calculate_legs_velocity, dq])
    calculate_position = np.vstack([calculate_position, calculate_position[-1] + u * dt])

    p.setJointMotorControl2(anchor_id_1, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=dq[0])
    p.setJointMotorControl2(anchor_id_2, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=dq[1])
    p.setJointMotorControl2(anchor_id_3, 1, controlMode=p.VELOCITY_CONTROL, targetVelocity=dq[2])
    
    #time.sleep(dt)
    p.stepSimulation()

    position = np.vstack([position, [p.getBasePositionAndOrientation(platform_id)[0][0], 
                                     p.getBasePositionAndOrientation(platform_id)[0][2], 
                                     p.getEulerFromQuaternion(p.getBasePositionAndOrientation(platform_id)[1])[1]]])

    legs = np.vstack([legs, [p.getJointState(anchor_id_1, 1)[0], 
                             p.getJointState(anchor_id_2, 1)[0],
                             p.getJointState(anchor_id_3, 1)[0]]])
    legs_velocity = np.vstack([legs_velocity, [p.getJointState(anchor_id_1, 1)[1], 
                                      p.getJointState(anchor_id_2, 1)[1],
                                      p.getJointState(anchor_id_3, 1)[1]]])


# Завершение симуляции
p.disconnect

print ('Фактические координаты и угол:', np.round(position[-1], 5))

# Отрисовка графиков
plt.figure('x, z, \u03c6')
plt.subplot(3, 1, 1)
plt.grid(True)
plt.xlabel('Time, s')
plt.ylabel('p$_x$, m')
plt.plot(t, calculate_position[:,0], color='r', label='Теор.')
plt.plot(t, position[:,0], label='Факт.')
plt.legend()
plt.subplot(3, 1, 2)
plt.grid(True)
plt.xlabel('Time, s')
plt.ylabel('p$_z$, m')
plt.plot(t, calculate_position[:,1], color='r', label='Теор.')
plt.plot(t, position[:,1], label='Факт.')
plt.legend()
plt.subplot(3, 1, 3)
plt.grid(True)
plt.xlabel('Time, s')
plt.ylabel('\u03c6, rad')
plt.plot(t, calculate_position[:,2], color='r', label='Теор.')
plt.plot(t, position[:,2], label='Факт.')
plt.legend()
plt.tight_layout()

plt.figure('X(Z)')
plt.grid(True)
plt.xlabel('X, m')
plt.ylabel('Z, m')
plt.plot(calculate_position[:,0], calculate_position[:,1], color='r', label='Теор.')
plt.plot(position[:,0], position[:,1], label='Факт.')
plt.legend()
plt.tight_layout()

plt.figure('legs velocity')
plt.subplot(3, 1, 1)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('s' + '\u0307' + '$_1$, m')
plt.plot(t, legs_velocity[:, 0], label='Факт.')
plt.plot(t, calculate_legs_velocity[:, 0], color='r', label='Теор.')
plt.legend()
plt.subplot(3, 1, 2)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('s' + '\u0307' + '$_2$, m')
plt.plot(t, legs_velocity[:, 1], label='Факт.')
plt.plot(t, calculate_legs_velocity[:, 1], color='r', label='Теор.')
plt.legend()
plt.subplot(3, 1, 3)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('s' + '\u0307' + '$_3$, m')
plt.plot(t, legs_velocity[:, 2], label='Факт.')
plt.plot(t, calculate_legs_velocity[:, 2], color='r', label='Теор.')
plt.legend()
plt.tight_layout()

plt.figure('График невязок')
plt.subplot(3, 1, 1)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('s' + '\u0307' + '$_1$, m')
plt.plot(t, abs(legs_velocity[:, 0] - calculate_legs_velocity[:, 0]), label='Невязка')
plt.legend()
plt.subplot(3, 1, 2)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('s' + '\u0307' + '$_2$, m')
plt.plot(t, abs(legs_velocity[:, 1] - calculate_legs_velocity[:, 1]), label='Невязка')
plt.legend()
plt.subplot(3, 1, 3)
plt.grid(True)
plt.xlabel('Время')
plt.ylabel('s' + '\u0307' + '$_3$, m')
plt.plot(t, abs(legs_velocity[:, 2] - calculate_legs_velocity[:, 2]), label='Невязка')
plt.legend()
plt.tight_layout()
plt.show()