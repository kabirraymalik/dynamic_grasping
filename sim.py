import time
import mujoco
import mujoco.viewer
from control import *

view_refresh_rate = 100 #Hz

#m = mujoco.MjModel.from_xml_path('resources/unitree_go2/scene.xml')
m = mujoco.MjModel.from_xml_path('resources/trossen_wx250s/scene.xml')
d = mujoco.MjData(m)

goalpos = []

def init_control(model,data):
    currpos = []
    for i in range(0, len(data.ctrl)):
        currpos.append(data.qpos[i])
    return currpos

def controller(model, data, robot):
    robot.update_state(data.qpos.copy(), data.qvel.copy())
    data.ctrl = robot.motor_control()

robot = Bot(init_control(m,d))

paused = False
last_view_refresh = time.time()
last_physics_refresh = time.time()

with mujoco.viewer.launch_passive(m,d, show_left_ui = False, show_right_ui = False) as viewer:
    while viewer.is_running():
        start_step_time = time.time()
        if not paused:

            #timing physics
            if last_physics_refresh < (time.time() - m.opt.timestep):
                mujoco.mj_step1(m,d)
                controller(m, d, robot)
                mujoco.mj_step2(m,d)
                last_physics_refresh = time.time()
            
            #timing view
            if time.time()-last_view_refresh > 1/view_refresh_rate:
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
                last_view_refresh = time.time()
                viewer.sync()
                robot.print_state()


