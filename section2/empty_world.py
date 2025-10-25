'''empty_world.py'''

# 不要from isaacsim.simulation_app import SimulationApp 
# isaacsim.模块 是软件内部的api，python解释器会报错说找不到。
# 如下这两行必须在导入模块之前就导入执行
from isaacsim import SimulationApp 
simulation_app = SimulationApp({"headless": False}) 
from isaacsim.core.api import World

world = World()
world.reset()

for i in range(500):
    world.step(render=True) # execute one physics step and one rendering step
simulation_app.close()      # close Isaac Sim