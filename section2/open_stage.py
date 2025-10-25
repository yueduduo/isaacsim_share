'''open_stage.py '''

from isaacsim import SimulationApp 
simulation_app = SimulationApp({"headless": False}) 

from omni.isaac.core.utils.stage import open_stage
from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim

usd_path = r"F:\Desktop\IsaacSim分享\section2\Collected_env\env.usd" 
open_stage(usd_path)
world = World()

# 创建香蕉的xform对象
banana = SingleXFormPrim(name="banana", prim_path="/World/banana")

# 在获取仿真数据前要调用reset方法，确保环境正确初始化
world.reset()
for i in range(500):
    # 获取香蕉的位姿
    position, orientation = banana.get_world_pose()
    print("Banana position is : " + str(position))
    print("Banana's orientation is : " + str(orientation))
    
    world.step(render=True) 
simulation_app.close()         