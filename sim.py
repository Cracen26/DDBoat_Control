import numpy as np
import matplotlib.pyplot as plt
import math

class RobotController:
    def __init__(self, x, y, heading):
        self.x = x  
        self.y = y 
        self.heading = heading
        self.trajectory = [(x, y)]  

    def move_to_target(self, x_target, y_target, heading_target, k_linear=1, k_angular=4): # k linear = 0.9, k angular = 2
        
        dx = x_target - self.x
        dy = y_target - self.y
        distance = np.sqrt(dx**2 + dy**2)

        # Calculate target heading
        heading_desired = np.arctan2(dy, dx)
        
        w = self.swatooth(heading_desired - self.heading)
        
        # Control law
        k_linear = (distance/100)*10
        linear_velocity = k_linear * distance  
        angular_velocity = k_angular * w 

        left_thruster, right_thruster = self.thrusterSpeed(linear_velocity, angular_velocity)

        self.heading += angular_velocity * 0.1 
        self.heading = self.swatooth(self.heading)
        self.x += linear_velocity * np.cos(self.heading) * 0.1 
        self.y += linear_velocity * np.sin(self.heading) * 0.1 

        self.trajectory.append((self.x, self.y))

        return left_thruster, right_thruster

    def thrusterSpeed(self, linear_velocity, angular_velocity):
        left_thruster = linear_velocity - angular_velocity
        right_thruster = linear_velocity + angular_velocity
        return left_thruster, right_thruster

    def swatooth(self, angle):
        return (angle+np.pi)%(2*np.pi)-np.pi 
    

robot = RobotController(15, 5, np.pi / 4)  
x_target, y_target = 5, 25  
heading_desired = np.pi / 2

# Simulate the robot movement
steps = 200
plt.figure(figsize=(6, 6))
plt.xlim(0, 30)
plt.ylim(0, 30)
plt.plot(x_target, y_target, 'rx', markersize=10, label='Target')
for _ in range(steps):
    plt.plot(robot.x, robot.y, 'bo', markersize=5, label='Robot')
    # final heading plot
    heading_x = robot.x + np.cos(robot.heading)
    heading_y = robot.y + np.sin(robot.heading)
    plt.plot([robot.x, heading_x], [robot.y, heading_y], 'r-', lw=2, label='Heading')

    plt.draw()
    plt.pause(0.01)
    robot.move_to_target(x_target, y_target, heading_desired)

plt.ioff()  
plt.show()
    
