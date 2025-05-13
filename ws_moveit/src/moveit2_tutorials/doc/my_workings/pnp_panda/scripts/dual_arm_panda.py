#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from tf2_msgs.msg import TFMessage
from moveit.core.robot_state import RobotState
from moveit.planning import (MoveItPy,MultiPipelinePlanRequestParameters,PlanRequestParameters)
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
import tkinter as tk
from tkinter import messagebox
import threading
import time

class MoveItMotionPlanner(Node):

    def __init__(self, node_name="moveit_py", transform_handler=None):
        """Initialize the MoveItMotionPlanner class."""
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.panda = MoveItPy(node_name=node_name)
        self.components = {
            "left": {
                "arm": self.panda.get_planning_component("left_panda_arm"),
                # "hand": self.panda.get_planning_component("left_hand"),
                # "arm_hand": self.panda.get_planning_component("left_panda_arm_hand"),
                "robot_model": self.panda.get_robot_model(),
                "robot_state": RobotState(self.panda.get_robot_model()),
                "plan_params": PlanRequestParameters(self.panda,"ompl_rrtc")
            },
            "right": {
                "arm": self.panda.get_planning_component("right_panda_arm"),
                # "hand": self.panda.get_planning_component("right_hand"),
                # "arm_hand": self.panda.get_planning_component("right_panda_arm_hand"),
                "robot_model": self.panda.get_robot_model(),
                "robot_state": RobotState(self.panda.get_robot_model()),
                "plan_params": PlanRequestParameters(self.panda, "ompl_rrtc")
            },
            "both":{
                "both_arms": self.panda.get_planning_component("both_arms"),
                "robot_model": self.panda.get_robot_model(),
                "robot_state": RobotState(self.panda.get_robot_model()),
                "plan_params": PlanRequestParameters(self.panda, "ompl_rrtc")                
            }
        }
        self.planning_scene_monitor = self.panda.get_planning_scene_monitor()
        self.logger.info("MoveItPy components initialized for both left & right pandas.")
        self.transform_handler = transform_handler
        if self.transform_handler is None:
            self.transform_handler = TfTransformHandler()
        self.openleftgripper = self.create_publisher(Bool,"OpenLeftGripper",10)
        self.openrightgripper = self.create_publisher(Bool,"OpenRightGripper",10)
        self.closeleftgripper = self.create_publisher(Bool,"CloseLeftGripper",10)
        self.closerightgripper = self.create_publisher(Bool,"CloseRightGripper",10)

    def plan_and_execute(self, side, component_type, goal_state=None, random=None, pose_goal=None, joint_constraint=None,
                         dual_arm_state=None, multi_plan_parameters=None, sleep_time=0.0):
        """Generalized function to plan and execute motion for left or right components."""
        component = self.components[side][component_type]
        plan_params = self.components[side]["plan_params"]
        self.logger.info(f"Planning {component_type} for {side} side")
        component.set_start_state_to_current_state()
        if goal_state:
            component.set_goal_state(configuration_name=goal_state)
        elif random:
            component.set_goal_state(robot_state=random)
        elif pose_goal:
            component.set_goal_state(pose_stamped_msg=pose_goal, pose_link=f"{side}_panda_link8")
        elif joint_constraint:
            component.set_goal_state(motion_plan_constraints=[joint_constraint])
        elif dual_arm_state:
            component.set_goal_state(robot_state=dual_arm_state)

        try:
            if multi_plan_parameters:
                plan_result = component.plan(multi_plan_parameters=multi_plan_parameters)
            else:
                plan_result = component.plan(plan_params)

            if plan_result:
                self.logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                self.panda.execute(robot_trajectory, controllers=[])
            else:
                self.logger.error("Planning failed")
        except Exception as e:
            self.logger.error(f"Error during planning or execution: {e}")
        time.sleep(sleep_time)

    def plan_arm_to_predefined_state(self, side, goal_state):
        #goal_state : "ready" or "extended"
        self.plan_and_execute(side=side, component_type="arm", goal_state=goal_state, sleep_time=1.0)

    def plan_arm_with_pose_stamped(self, side, pose_goal):
        #pose_goal : position(x,y,z) , orientation (w,x,y,z)
        self.plan_and_execute(side=side, component_type="arm", pose_goal=pose_goal, sleep_time=1.0)

    # def plan_hand_to_predefined_state(self, side, goal_state):
    #     #goal_state : "open" or "close"#
    #     self.plan_and_execute(side=side, component_type="hand", goal_state=goal_state, sleep_time=0.5)      

    def plan_both_arms_to_predefined_state(self,goal_state):
        #goal_state : "ready" or "extended"
        self.plan_and_execute(side="both", component_type="both_arms", goal_state=goal_state, sleep_time=1.0)
    
    def plan_both_arms_with_pose_stamped(self, pose_goal_left, pose_goal_right):
        robot_state = self.components["both"]["robot_state"]
        success_left = robot_state.set_from_ik(
            joint_model_group_name="left_panda_arm",  # The joint model group name for the left arm
            geometry_pose=pose_goal_left,
            tip_name="left_panda_link8",
            timeout=0.5)
        success_right = robot_state.set_from_ik(
            joint_model_group_name="right_panda_arm",  # The joint model group name for the right arm
            geometry_pose=pose_goal_right,
            tip_name="right_panda_link8",
            timeout=0.5)
        if success_left and success_right:
            self.plan_and_execute(side="both", component_type="both_arms", dual_arm_state=robot_state, sleep_time=1.0)
        else:
            if not success_left:
                self.logger.error("IK failed for left arm.")
            if not success_right:
                self.logger.error("IK failed for right arm.")  

    def add_collision_objects(self):
        """Add collision objects to the planning scene."""
        from shape_msgs.msg import SolidPrimitive
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject

        # Define each object: (type, position, dimensions)
        objects = [
            ("box", (0.0, 1.0, 0.41526), (1.8, 1.52, 0.1)),    # Table
            ("box", (0.0, 0.0, 0.65), (0.39, 0.2, 1.0)),       # Robot body
            ("cylinder", (0.0, 0.0, 1.2), (0.1, 0.15)),      # Neck (height = 0.3, radius = 0.05)
            ("sphere", (0.0, 0.0, 1.35), (0.15,)),               # Sphere (radius = 0.1)
        ]

        with self.planning_scene_monitor.read_write() as scene:
            collision_object = CollisionObject()
            collision_object.header.frame_id = "world"
            collision_object.id = "custom_objects"

            for obj_type, position, dims in objects:
                pose = Pose()
                pose.position.x = position[0]
                pose.position.y = position[1]
                pose.position.z = position[2]

                primitive = SolidPrimitive()
                if obj_type == "box":
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = list(dims)  # [x, y, z]
                elif obj_type == "sphere":
                    primitive.type = SolidPrimitive.SPHERE
                    primitive.dimensions = list(dims)  # [radius]
                elif obj_type == "cylinder":
                    primitive.type = SolidPrimitive.CYLINDER
                    primitive.dimensions = list(dims)  # [height, radius]
                else:
                    continue  # skip unknown types

                collision_object.primitives.append(primitive)
                collision_object.primitive_poses.append(pose)
                collision_object.operation = CollisionObject.ADD

            scene.apply_collision_object(collision_object)
            scene.current_state.update()

    def remove_collision_objects(self):
        """Remove all collision objects from the planning scene."""
        with self.planning_scene_monitor.read_write() as scene:
            scene.remove_all_collision_objects()
            scene.current_state.update()

    def shutdown(self):
        """Shutdown the ROS client library."""
        rclpy.shutdown()

    def plan_arm_to_target(self, side, orientation=None,topic=None,offset=(0,0,0)):
        # Set default orientation if none provided
        if orientation is None:
            orientation = {}
        offset_x,offset_y,offset_z= offset
        self.transform_handler.start_subscription(topic)
        time.sleep(0.5)
        if not self.transform_handler.latest_translation:
            self.logger.error("Failed to get the latest object position.")
            return
        # Create and set pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = 'world'
        # pose_goal.header.frame_id = f"{side}_panda_link0"
        pose_goal.pose.position.x = self.transform_handler.latest_translation['x'] + offset_x
        pose_goal.pose.position.y = self.transform_handler.latest_translation['y'] + offset_y
        pose_goal.pose.position.z = self.transform_handler.latest_translation['z'] + offset_z
        pose_goal.pose.orientation.w = orientation.get('w', 0.0)
        pose_goal.pose.orientation.x = orientation.get('x', 0.0)
        pose_goal.pose.orientation.y = orientation.get('y', 0.0)
        pose_goal.pose.orientation.z = orientation.get('z', 0.0)
        self.plan_arm_with_pose_stamped(side=side ,pose_goal=pose_goal)

    def plan_both_arms_to_target(self, orientationleft=None, orientationright=None, topic_left=None, topic_right=None,
                                 offsetleft=(0,0,0), offsetright=(0,0,0)):
        if orientationleft is None:
            orientationleft = {}
        if orientationright is None:
            orientationright = {}
        offsetleft_x,offsetleft_y,offsetleft_z = offsetleft
        offsetright_x,offsetright_y,offsetright_z = offsetright

        self.transform_handler.start_subscription(topic_left)
        time.sleep(0.5)
        if not self.transform_handler.latest_translation:
            self.logger.error("Failed to get the left sphere position.")
            return
        pose_goal_left = Pose()
        pose_goal_left.position.x = self.transform_handler.latest_translation['x'] + offsetleft_x
        pose_goal_left.position.y = self.transform_handler.latest_translation['y'] + offsetleft_y
        pose_goal_left.position.z = self.transform_handler.latest_translation['z'] + offsetleft_z
        pose_goal_left.orientation.w = orientationleft.get('w', 0.0)
        pose_goal_left.orientation.x = orientationleft.get('x', 0.0)
        pose_goal_left.orientation.y = orientationleft.get('y', 0.0)  
        pose_goal_left.orientation.z = orientationleft.get('z', 0.0)

        self.transform_handler.start_subscription(topic_right)
        time.sleep(0.5)
        if not self.transform_handler.latest_translation:
            self.logger.error("Failed to get the left sphere position.")
            return
        pose_goal_right = Pose()
        pose_goal_right.position.x = self.transform_handler.latest_translation['x'] + offsetright_x
        pose_goal_right.position.y = self.transform_handler.latest_translation['y'] + offsetright_y
        pose_goal_right.position.z = self.transform_handler.latest_translation['z'] + offsetright_z
        pose_goal_right.orientation.w = orientationright.get('w', 0.0)
        pose_goal_right.orientation.x = orientationright.get('x', 0.0)
        pose_goal_right.orientation.y = orientationright.get('y', 0.0)
        pose_goal_right.orientation.z = orientationright.get('z', 0.0)
        
        self.plan_both_arms_with_pose_stamped(pose_goal_left, pose_goal_right)

class TfTransformHandler(Node):

    def __init__(self):
        super().__init__('tf_transform_handler')
        self.logger = self.get_logger()
        self.latest_translation = None
        self.sub_ = None

    def start_subscription(self, topic):
        if self.sub_ is None:
            self.sub_ = self.create_subscription(
                TFMessage, topic, self.callback_data, 10)
            self.logger.info(f"Subscription to {topic} created")

    def callback_data(self, msg: TFMessage):
        # Assuming there's only one target and we take the first transform
        transform = msg.transforms[0]
        self.latest_translation = {
            'x': transform.transform.translation.x,
            'y': transform.transform.translation.y,
            'z': transform.transform.translation.z
        }
        self.logger.info(f"Received position: {self.latest_translation}")
        self.unsubscribe()

    def unsubscribe(self):
        if self.sub_ is not None:
            self.destroy_subscription(self.sub_)
            self.sub_ = None
            self.logger.info("Subscription destroyed")

class TkinterGUI:

    def __init__(self, root):
        self.root = root
        self.root.title("MoveIt Motion Planner")
        self.transform_handler = TfTransformHandler()
        self.motion_planner = MoveItMotionPlanner(transform_handler=self.transform_handler)
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.transform_handler,), daemon=True)
        self.ros_thread.start()
        self.create_buttons()
        self.trial = 0

    def create_buttons(self):
        # Helper function to create buttons for both left and right sides
        def create_arm_hand_buttons(side, col):
            tk.Label(self.root, text=f"{side.capitalize()} Arm", font=("Arial", 14, "bold")).grid(row=0, column=col, pady=5, padx=5, sticky='ew')
            tk.Button(self.root, text="Ready State", command=lambda: self.plan_arm_to_predefined_state(side, "ready")).grid(row=1, column=col, pady=5, padx=5, sticky='ew')
            tk.Button(self.root, text="Humanoid State", command=lambda: self.plan_arm_to_predefined_state(side, "humanoid")).grid(row=2, column=col, pady=5, padx=5, sticky='ew')
            tk.Button(self.root, text="Move to Sphere", command=lambda: self.plan_arm_to_sphere(side)).grid(row=3, column=col, pady=5, padx=5, sticky='ew')
            # tk.Button(self.root, text="Open State", command=lambda: self.plan_hand_to_state(side, "open")).grid(row=4, column=col, pady=5, padx=5, sticky='ew')
            # tk.Button(self.root, text="Close State", command=lambda: self.plan_hand_to_state(side, "close")).grid(row=5, column=col, pady=5, padx=5, sticky='ew')

        # Create buttons for both sides
        create_arm_hand_buttons("left", 0)
        create_arm_hand_buttons("right", 1)

        tk.Label(self.root, text="Both Arms", font=("Arial", 14, "bold")).grid(row=0, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Ready Simultaneously", command=lambda: self.executesimultaneously("ready")).grid(row=1, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Humanoid Simultaneously", command=lambda: self.executesimultaneously("humanoid")).grid(row=2, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Both to Spheres", command=lambda: self.plan_both_arms_to_spheres()).grid(row=3, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Magazine Loading", command=lambda: self.magazine_loading()).grid(row=4, column=2, pady=5, padx=5, sticky='ew')
        
        tk.Label(self.root, text="Planing Scene", font=("Arial", 14, "bold")).grid(row=0, column=3, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Add Obstacles", command=self.add_obstacles).grid(row=1, column=3, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Remove Obstacles", command=self.remove_obstacles).grid(row=2, column=3, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Temporary Algorithm Testing", command=self.algorithm_testing).grid(row=3, column=3, pady=5, padx=5, sticky='ew')
        
        # Create the Exit button spanning all columns
        tk.Button(self.root, text="Exit", command=self.exit_program).grid(row=5, column=0, columnspan=4, pady=20, sticky='ew')

        # Adjust column weights for even button spacing
        for i in range(5):
            self.root.grid_columnconfigure(i, weight=1)

    # Plan both arms to the "ready" state simultaneously
    def executesimultaneously(self,goal_state):
        self.motion_planner.plan_both_arms_to_predefined_state(goal_state)
        messagebox.showinfo("Info", f"Planning both arms to {goal_state} state completed.")

    def plan_arm_to_predefined_state(self, side, goal_state):
        self.motion_planner.plan_arm_to_predefined_state(side, goal_state)
        messagebox.showinfo("Info", f"Planning {side.capitalize()} arm to {goal_state} state completed.")

    def plan_hand_to_state(self, side, goal_state):
        """Generic method to plan left or right hand to a predefined state."""
        self.motion_planner.plan_hand_to_predefined_state(side, goal_state)
        messagebox.showinfo("Info", f"Planning {side.capitalize()} hand to {goal_state} state completed.")
    
    def plan_arm_to_sphere(self, side):
        orientation = {'w': 0.0, 'x': 0.92388, 'y': 0.38268, 'z': 0.0}
        self.motion_planner.plan_arm_to_target(side, orientation,'tf_sphere')
        # self.motion_planner.plan_arm_to_sphere(side, orientation,f'tf_sphere_{side}')
        messagebox.showinfo("Info", f"Planning {side.capitalize()} arm to pose completed.")

    def plan_both_arms_to_spheres(self):
        orientation = {'w': 0.0, 'x': 0.92388, 'y': 0.38268, 'z': 0.0}
        self.motion_planner.plan_both_arms_to_target(orientation,orientation,"tf_sphere_left", "tf_sphere_right")
        messagebox.showinfo("Info", f"Planning both arms to pose completed.")

    def magazine_loading(self):
        msg = Bool(data=False)
        msgleftopen = msg
        msgleftclose = msg
        msgrightopen = msg
        msgrightclose = msg
        self.motion_planner.openleftgripper.publish(msgleftopen)
        self.motion_planner.closeleftgripper.publish(msgleftclose)
        self.motion_planner.openrightgripper.publish(msgrightopen)
        self.motion_planner.closerightgripper.publish(msgrightclose)
        orientDownwards = {'w': 0.0, 'x': 0.92388, 'y': 0.38268, 'z': 0.0}
        orientFrontwards = {'w': 0.65328, 'x': -0.65328, 'y': -0.2706, 'z': -0.2706}
        orientLeftwards = {'w': -0.2706, 'x': 0.65328, 'y': -0.2706, 'z': 0.65328}
        orientRightwards = {'w': 0.65328, 'x': -0.2706, 'y': -0.65328, 'z': 0.2706}
        orientLeftArm = {'w': 0.70711, 'x': -0.5, 'y': -0.5, 'z': 0.0}
        orientRightArm = {'w': 0.5, 'x': -0.70711, 'y': 0.0, 'z': -0.5}

        self.motion_planner.plan_both_arms_to_target(orientDownwards,orientDownwards,"tf_magazine", "tf_bullet1",(0,-0.02,0.2),(0,0,0.2))
        msgleftopen=self.get_opposite_msg(msgleftopen)
        self.motion_planner.openleftgripper.publish(msgleftopen)
        msgrightopen=self.get_opposite_msg(msgrightopen)
        self.motion_planner.openrightgripper.publish(msgrightopen)
        time.sleep(1.0)
        self.motion_planner.plan_both_arms_to_target(orientDownwards,orientDownwards,"tf_magazine", "tf_bullet1",(0,-0.02,0.12),(0,0,0.11))
        msgleftclose=self.get_opposite_msg(msgleftclose)
        self.motion_planner.closeleftgripper.publish(msgleftclose)
        msgrightclose=self.get_opposite_msg(msgrightclose)
        self.motion_planner.closerightgripper.publish(msgrightclose)
        time.sleep(1.0)
        self.motion_planner.plan_arm_to_target('right',orientLeftwards,"tf_sphere",(-0.11,0,0.1))
        self.motion_planner.plan_arm_to_target('left',orientRightwards,"tf_sphere",(0.12,0,0))

        bullets = [2,3,4,5]
        for bullet in bullets:
            topic = f"tf_bullet{bullet}"
            msgrightopen = self.get_opposite_msg(msgrightopen)
            self.motion_planner.openrightgripper.publish(msgrightopen)
            time.sleep(0.5)
            self.motion_planner.plan_arm_to_target('right', orientLeftwards, "tf_sphere", (-0.3,0,0.1))          
            self.motion_planner.plan_arm_to_target('right', orientDownwards, topic, (0, 0, 0.2))
            self.motion_planner.plan_arm_to_target('right', orientDownwards, topic, (0, 0, 0.11))
            msgrightclose = self.get_opposite_msg(msgrightclose)
            self.motion_planner.closerightgripper.publish(msgrightclose)
            time.sleep(1.0)
            # self.motion_planner.plan_arm_to_target('right', orientDownwards, topic, (-0.2, 0, 0.5))      
            self.motion_planner.plan_arm_to_target('right', orientLeftwards, "tf_sphere", (-0.3,0,0.1))          
            self.motion_planner.plan_arm_to_target('right', orientLeftwards, "tf_sphere", (-0.11,0,0.1))        

        time.sleep(0.5)
        msgrightopen=self.get_opposite_msg(msgrightopen)
        self.motion_planner.openrightgripper.publish(msgrightopen) 

        # Move left arm to place the magazine
        left_end_goal = PoseStamped()
        left_end_goal.header.frame_id = 'world'
        left_end_goal.pose.position.x = 0.3
        left_end_goal.pose.position.y = 0.6-0.02
        left_end_goal.pose.position.z = 0.5+0.12
        left_end_goal.pose.orientation.w = orientDownwards.get('w',0.0)
        left_end_goal.pose.orientation.x = orientDownwards.get('x',0.0)
        left_end_goal.pose.orientation.y = orientDownwards.get('y',0.0)
        left_end_goal.pose.orientation.z = orientDownwards.get('z',0.0)
        self.motion_planner.plan_arm_with_pose_stamped('left',left_end_goal)
        msgleftopen=self.get_opposite_msg(msgleftopen)
        self.motion_planner.openleftgripper.publish(msgleftopen)
        time.sleep(0.5)
        left_end_goal.pose.position.z = 0.5+0.2
        self.motion_planner.plan_arm_with_pose_stamped('left',left_end_goal)
        self.motion_planner.plan_both_arms_to_predefined_state('humanoid')

    def get_opposite_msg(self,current_msg):
         return Bool(data=not current_msg.data)    
            
    def add_obstacles(self):
        self.motion_planner.add_collision_objects()
        messagebox.showinfo("Info", "Obstacles added to the planning scene.")

    def remove_obstacles(self):
        self.motion_planner.remove_collision_objects()
        messagebox.showinfo("Info", "Obstacles removed from the planning scene.")
    
    def algorithm_testing(self):
        pass

    def exit_program(self):
        self.motion_planner.shutdown()
        self.root.quit()

def main():
    rclpy.init()
    root = tk.Tk()
    gui = TkinterGUI(root)
    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


