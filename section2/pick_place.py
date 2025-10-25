'''pick_place.py'''

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.stage import open_stage
from isaacsim.core.api import World
from isaacsim.core.prims import SingleXFormPrim

from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController

usd_path = r"F:\Desktop\IsaacSim分享\section2\Collected_env\env.usd" 
open_stage(usd_path)
world = World()
banana = SingleXFormPrim(name="banana", prim_path="/World/banana")

# 创建机器人Articulation和PickPlaceController
franka:Franka = world.scene.add(Franka(prim_path="/Franka", name="franka")) 
controller = PickPlaceController(
            name="pick_place_controller",
            gripper=franka.gripper,
            robot_articulation=franka,
            end_effector_initial_height=1.0+0.3,
            events_dt=[0.008, 0.005, 1, 0.01, 0.05, 0.05, 0.0025, 1, 0.008, 0.08], 
        )

world.reset()
for i in range(50): # 先执行50步仿真，得到物体掉落后稳定
    world.step()

franka.gripper.set_joint_positions(franka.gripper.joint_opened_positions)

# 确定抓取点和放置点
banana_position, banana_orientation = banana.get_world_pose()
banana_position[2] -= 0.01
goal_position = banana_position.copy()
goal_position[0] += 0.2
goal_position[2] += 0.05
print(banana_position)
print(goal_position)

for i in range(1000000):
    # 获取当前机械臂关节位置
    current_joint_positions = franka.get_joint_positions()
    # 计算需要执行的动作
    actions = controller.forward(
            picking_position=banana_position,
            placing_position=goal_position,
            current_joint_positions=current_joint_positions,
    )
    # 执行动作
    franka.apply_action(actions)
    
    world.step(render=True) 
simulation_app.close()     