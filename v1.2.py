import sys
from roboticstoolbox import mstraj
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
from pynput import keyboard

def create_3dof(size):
    ets = (  
        ET.Rz() * ET.tz(size[0]) *     
        ET.Ry() * ET.tx(size[1]) *       
        ET.Ry() * ET.tx(size[2])
    )  
    robot = rtb.Robot(ets, name="V1.2")  
    return robot

class RobotController:
    def __init__(self, robot, env):
        self.robot = robot
        self.env = env
        self.position = robot.fkine(self.robot.q).t
        self.step_size = 0.02
        self.stop = 1
        self.keys = set()
        self.list_of_pos = []
        self.change = 1
    
    def on_press(self, key):
        try: 
            self.keys.add(key.char)
        except AttributeError: 
            pass
    
    def on_release(self, key):
        try: 
            self.keys.discard(key.char)
        except AttributeError: 
            pass
        if key == keyboard.Key.esc:
            return False

    def mov(self,pos):
        target = SE3(*pos)
        status = 1
        sol=self.robot.ikine_LM(target,q0=self.robot.q, mask=[1,1,1,0,0,0])
        if sol.success:
            self.robot.q=sol.q
            self.env.step()
            self.change = 1
        else: 
            status = 0
            self.change = 0
        return status
    
    def trajectory(self):
        """Запускает траекторию через все сохраненные точки"""
        if len(self.list_of_pos) < 2:
            print("❌ Для траектории нужно минимум 2 позиции!")
            return
    
        try:
            via_points = np.array(self.list_of_pos)
            print(f"Создание траектории через {len(via_points)} точек...")
        
        # 🔧 ИСПРАВЛЕНИЕ: mstraj возвращает ОДИН объект Trajectory
            traj = mstraj(
                via_points,
                dt=0.01,    # Временной шаг
                qdmax=2.0,  # Максимальная скорость
                tacc=0.00001    # Время разгона
            )
        
        # 🔧 Получаем данные из объекта Trajectory
            q_traj = traj.q  # Позиции
        # qd_traj = traj.qd  # Скорости (опционально)
        # qdd_traj = traj.qdd  # Ускорения (опционально)
        
            print(f" Траектория создана: {len(q_traj)} шагов")
        
            for i, q in enumerate(q_traj):
                self.robot.q = q
                self.env.step()
                print(f"Шаг {i+1}/{len(q_traj)}", end='\r')
                plt.pause(0.05)
        
            print(f"\n Траектория завершена!")
        
        except Exception as e:
            print(f" Ошибка в траектории: {e}")
        
    def update_robot(self):
        old_pos = self.position.copy()
        if 'w' in self.keys: self.position[0] += self.step_size
        if 's' in self.keys: self.position[0] -= self.step_size
        if 'a' in self.keys: self.position[1] += self.step_size
        if 'd' in self.keys: self.position[1] -= self.step_size
        if 'q' in self.keys: self.position[2] += self.step_size
        if 'e' in self.keys: self.position[2] -= self.step_size
        if 'i' in self.keys and self.change == 1:
            self.list_of_pos.append(self.robot.q.copy())
            self._space_pressed = True
            self.change =0
            print("SAVE\n")
        elif 'i' not in self.keys:
            self._space_pressed = False
        if 'u' in self.keys: self.trajectory()
        if {'w', 's', 'a', 'd', 'q', 'e'} & self.keys:
            self.stop = self.mov(self.position)
        if self.stop == 0: 
            self.position = old_pos
        return self.stop

    
    def start(self):
        print("Управление роботом с клавиатуры")
        print("W/S - движение по X")
        print("A/D - движение по Y")
        print("Q/E - движение по Z")
        print("ESC - выход")
        print("I - сохранение координаты")

        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while listener.running:
                success = self.update_robot()
                print(f"{success} Позиция: X={self.position[0]:.3f}, Y={self.position[1]:.3f}, Z={self.position[2]:.3f}", end='\r')
                plt.pause(0.03)

# Основная программа
if __name__ == "__main__":
    plt.rcParams['keymap.save'] = ['ctrl+s']
    plt.rcParams['keymap.quit'] = ['ctrl+q']
    robot = create_3dof([0.1, 0.15, 0.1])
    env = PyPlot()
    env.launch()
    env.add(robot)

    
    # Запускаем контроллер
    controller = RobotController(robot, env)
    controller.start()
    
    print("\nПрограмма завершена")
    plt.close('all')  # Закрывает все окна matplotlib