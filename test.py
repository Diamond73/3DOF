# Установка: pip install pynput
from pynput import keyboard
import matplotlib.pyplot as plt
import numpy as np
from spatialmath import SE3
import roboticstoolbox as rtb
from roboticstoolbox.backends.PyPlot import PyPlot
from roboticstoolbox import ET

def create_3dof(size):
    ets = (  
        ET.Rz() * ET.tz(size[0]) *
        ET.Ry() * ET.tx(size[1]) * 
        ET.Ry() * ET.tx(size[2])
    )  
    robot = rtb.Robot(ets, name="V0.1")  
    return robot

class RobotController:
    def __init__(self, robot, env):
        self.robot = robot
        self.env = env
        self.position = [0.3, 0.0, 0.2]
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
        # Останавливаем слушатель при нажатии ESC
        if key == keyboard.Key.esc:
            return False
    
    def update_robot(self):
        """Обновляет позицию робота на основе нажатых клавиш"""
        # Обработка движения
        if 'w' in self.keys: self.position[0] += self.step_size
        if 's' in self.keys: self.position[0] -= self.step_size
        if 'a' in self.keys: self.position[1] += self.step_size
        if 'd' in self.keys: self.position[1] -= self.step_size
        if 'q' in self.keys: self.position[2] += self.step_size
        if 'e' in self.keys: self.position[2] -= self.step_size
        if 'r' in self.keys: self.position = [0.3, 0.0, 0.2]  # Сброс
        if '+' in self.keys: self.step_size = min(0.1, self.step_size + 0.005)
        if '-' in self.keys: self.step_size = max(0.001, self.step_size - 0.005)
        
        # Решаем ОК и обновляем робота
        target = SE3(*self.position)
        sol = self.robot.ikine_LM(target, mask=[1, 1, 1, 0, 0, 0])
        
        if sol.success:
            self.robot.q = sol.q
            self.env.step()
        
        return sol.success
    
    def start(self):
        print("🎮 Управление роботом с клавиатуры")
        print("W/S - движение по X")
        print("A/D - движение по Y")
        print("Q/E - движение по Z")
        print("R - сброс позиции")
        print("+/- - изменить шаг движения")
        print("ESC - выход")
        print("=" * 50)
        
        # Запускаем слушатель клавиш
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            while listener.running:
                success = self.update_robot()
                status = "✅" if success else "❌"
                print(f"{status} Позиция: X={self.position[0]:.3f}, Y={self.position[1]:.3f}, Z={self.position[2]:.3f} | Шаг: {self.step_size:.3f}", end='\r')
                
                # Обновляем график
                plt.pause(0.03)

# Основная программа
if __name__ == "__main__":
    # Создаем робота и среду
    robot = create_3dof([0.1, 0.2, 0.3])
    env = PyPlot()
    env.launch()
    env.add(robot)
    
    # Показываем начальную позицию
    robot.q = [0, 0, 0]
    env.step()
    
    # Запускаем контроллер
    controller = RobotController(robot, env)
    controller.start()
    
    print("\n👋 Программа завершена")
    env.hold()