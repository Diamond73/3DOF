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
        self.keys = set()
    
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
    
    def update_robot(self):
        
        if 'w' in self.keys: self.position[0] += self.step_size
        if 's' in self.keys: self.position[0] -= self.step_size
        if 'a' in self.keys: self.position[1] += self.step_size
        if 'd' in self.keys: self.position[1] -= self.step_size
        if 'q' in self.keys: self.position[2] += self.step_size
        if 'e' in self.keys: self.position[2] -= self.step_size
        
        target = SE3(*self.position)
        sol = self.robot.ikine_LM(target,q0=self.robot.q, mask=[1, 1, 1, 0, 0, 0])
        
        if sol.success:
            self.robot.q = sol.q
            self.env.step()
        
        return sol.success
    
    def start(self):
        print("Управление роботом с клавиатуры")
        print("W/S - движение по X")
        print("A/D - движение по Y")
        print("Q/E - движение по Z")
        print("ESC - выход")
        
        # Запускаем слушатель клавиш
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while listener.running:
                success = self.update_robot()
                status = "Succes" if success else "error"
                print(f"{status} Позиция: X={self.position[0]:.3f}, Y={self.position[1]:.3f}, Z={self.position[2]:.3f}", end='\r')
                
                # Обновляем график
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