import time
import mujoco
import mujoco.viewer

view_refresh_rate = 100 #Hz

m = mujoco.MjModel.from_xml_path('resources/unitree_go2/scene.xml')
m = mujoco.MjModel.from_xml_path('resources/trossen_wx250s/scene.xml')
d = mujoco.MjData(m)

goalpos = []
currpos = []

def init_controller(model,data):
    start_pos = d.qpos.copy()
    for i in range(0, len(d.ctrl)):
        goalpos.append(start_pos[i])
        currpos.append(start_pos[i])

def controller(model, data):
    pos = data.qpos.copy()
    for i in range(0, len(d.ctrl)):
        currpos.append(pos[i])
    pass

init_controller(m,d)
mujoco.set_mjcb_control(controller)

paused = False
last_view_refresh = time.time()
last_physics_refresh = time.time()

with mujoco.viewer.launch_passive(m,d) as viewer:
    while viewer.is_running():
        start_step_time = time.time()
        if not paused:

            #timing physics
            if last_physics_refresh < (time.time() - m.opt.timestep):
                mujoco.mj_step(m,d)
                last_physics_refresh = time.time()
            
            #timing view
            if time.time()-last_view_refresh > 1/view_refresh_rate:
                with viewer.lock():
                    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
                last_view_refresh = time.time()
                viewer.sync()


