import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
import time

class Bot():
    #self.goal_position of type State, use State.get_thetas() for motorpos array
    def __init__(self, init_motor_pos):
        print(init_motor_pos)
        self.move_queue = []
        self.num_ctrls = len(init_motor_pos)
        self.current_position = State(init_motor_pos)
        self.goal_position = State(np.zeros(self.num_ctrls))
        self.current_velocities = []
        #PID stuff
        self.time = 0
        self.integral = 0
        self.time_prev = -1e-6
        self.e_prev = 0
        self.Kp = 0.00006
        self.Ki = 0.00002
        self.Kd = 0.00001
        #self.Kp = 0.6
        #self.Ki = 0.2
        #self.Kd = 0.1
        self.start_time = time.time()

    def motor_control(self):
        #captures the goal position at 0.5s
        if (time.time() - self.start_time) > 0.3 and all([theta == 0 for theta in self.goal_position.get_thetas()]):
            goal = self.current_position.get_thetas().copy()
            self.goal_position = State(goal)
        #5 seconds for control to start
        if time.time() - self.start_time > 5:
            ctrl = []
            for i in range(0, self.num_ctrls):
                ctrl.append(self.PID(self.goal_position.get_thetas()[i], self.current_position.get_thetas()[i]))
            return ctrl
    
    #controller modified from https://softinery.com/blog/implementation-of-pid-controller-in-python/
    def PID(self, goal, curr):
        # PID calculations
        e = goal - curr
            
        P = self.Kp*e
        self.integral = self.integral + self.Ki*e*(self.time - self.time_prev)
        D = self.Kd*(e - self.e_prev)/(self.time - self.time_prev)

        # calculate manipulated variable - MV 
        MV = P + self.integral + D
        
        # update stored data for next iteration
        self.e_prev = e
        return MV
    
    def update_time(self, time):
        self.time_prev = self.time
        self.time = time
    
    def update_state(self, pos, vel):
        self.current_position = State(pos)
        self.current_velocities = vel
    
    def print_state(self):
        print(f"t = {time.time() - self.start_time}s, qpos = {self.current_position.get_thetas()}, goalpos = {self.goal_position.get_thetas()}")


class State():
    def __init__(self, motors):
        self.motor_positions = motors
    
    def compare_to(self, other_position, accuracy):
        if len(self.motor_positions) == len(other_position.motor_positions):
            deviations = []
            for id in range(0, len(self.motor_positions) - 1):
                deviations.append(abs(self.motor_positions[id] - other_position.motor_positions[id])/self.motor_positions[id])
            avg_percentage_deviation = 0
            for deviation in deviations:
                avg_percentage_deviation += deviation
            avg_percentage_deviation /= (len(deviations) - 1)
            if avg_percentage_deviation < accuracy:
                return True
        print('error: invalid state comparison')
        return False
    
    def get_thetas(self):
        return self.motor_positions