#V1.0
#Фиксируем данную версию в связи с достигнутой целью: решение ОЗК с визуализацией в контейнере
#Основные проблемы этой версии
#это отсуствие оптимального решения и ручной ввод точек без управления, отсуствие построения траектории с сглаживанием
#Следующий этап - это поиск решение ОЗК с поиском наиболее оптимального решения. Например, используя рукописную функцию с решением выведенным геометрически
#Заметка: рассмотреть вариант с поиском решения через кватернионы. У них большой потенциал. Необходимо решить ПЗК для двух звенника используя кватернионы

#V1.1
#Фиксируем данную версию и отправляем в gitlab. 
#Цели: теперь точки вводятся, а также ОЗК здесь автоматические решает, чтобы найти наиболее подходящий случай
import sys
import numpy as np
from math import pi
from math import sin
from math import cos
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt
from spatialmath import SE3
import roboticstoolbox as rtb  
from roboticstoolbox import ET  

def create_3dof(size):
    ets = (  
        ET.Rz() * ET.tz(size[0]) *     # Базовое вращение
        ET.Ry() * ET.tx(size[1]) *     # Плечо  
        ET.Ry() * ET.tx(size[2])       # Предплечье
    )  
    robot = rtb.Robot(ets, name="V1.1")  
    return robot


def env_init(robot):
    env = PyPlot()
    env.launch()
    env.add(robot)
    robot.q = [0,0,0]
    env.step()
    return env

def vis(env,robot, target, sol=None):
    robot.q = sol.q
    env.step()
    ax = env.ax
    ax.scatter(target.t[0], target.t[1], target.t[2], 
        c='red', s=200, marker='*', label=f'Target {target.t}')
        
        # Достигнутая точка
    T_achieved = robot.fkine(sol.q)
    ax.scatter(T_achieved.t[0], T_achieved.t[1], T_achieved.t[2],
        c='green', s=100, marker='o', label='Achieved')
    env.ax.legend()
    env.ax.set_xlabel('X')
    env.ax.set_ylabel('Y') 
    env.ax.set_zlabel('Z')
    env.ax.set_title('Robot Manipulator')
    
    # Обновляем отображение
    plt.draw()
    plt.pause(0.01)


robot = create_3dof([0.1, 0.2, 0.3])
env = env_init(robot)
q_init = [0, 0, 0]
print(f"Введите координаты через точку в порядке x y z. Чтобы выйти нажмите q")
user_input = input()
while user_input != "q":
    coords = [float(coord) for coord in user_input.split()]
    target = SE3(coords[0], coords[1], coords[2])
    sol = robot.ikine_LM(target, q0=q_init, mask=[1, 1, 1, 0, 0, 0])
    print(f"Результат: {sol.success}")
    if (sol.success):
        print(f"Углы: {sol.q}")
        T_achieved = robot.fkine(sol.q)
        print(f"Целевая позиция: {target.t}")
        print(f"Достигнутая позиция: {T_achieved.t}")
        print(f"Фактическая ошибка: {np.linalg.norm(T_achieved.t - target.t):.6f}")
        vis(env, robot, target, sol)
        q_init=sol.q
    else:
        print(f"Цель недостижима")
    user_input = input()

    

