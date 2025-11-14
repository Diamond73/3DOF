#!/usr/bin/env python3
import sys
import numpy as np

print(f"Python version: {sys.version}")
print(f"NumPy version: {np.__version__}")

try:
    import roboticstoolbox as rtb
    from spatialmath import SE3
    import matplotlib.pyplot as plt
    
    print("=== Robotics Toolbox Test ===")
    print(f"Robotics Toolbox version: {rtb.__version__}")
    print(f"Spatialmath version: {sm.__version__}")

    # Простой тест без сложных операций
    robot = rtb.models.DH.Puma560()
    print(f"✓ Robot created: {robot.name}")

    # Прямая кинематика
    T = robot.fkine(robot.qz)
    print("✓ Forward kinematics successful")

    print("🎉 Basic functionality test passed!")

except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()