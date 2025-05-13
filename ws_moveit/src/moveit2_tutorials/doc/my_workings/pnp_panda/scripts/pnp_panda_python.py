#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from moveit.core.robot_state import RobotState
from moveit.planning import (MoveItPy,MultiPipelinePlanRequestParameters,PlanRequestParameters)
from moveit.core.kinematic_constraints import construct_joint_constraint
import tkinter as tk
from tkinter import messagebox
import threading

class MoveItMotionPlanner(Node):

    def __init__(self, node_name="moveit_py", transform_handler=None):
        """Initialize the MoveItMotionPlanner class."""
        super().__init__(node_name)
        self.logger = self.get_logger()
        self.panda = MoveItPy(node_name=node_name)
        self.panda_arm = self.panda.get_planning_component("panda_arm")
        self.panda_arm_hand = self.panda.get_planning_component("panda_arm_hand")
        self.panda_hand = self.panda.get_planning_component("hand")
        self.robot_model = self.panda.get_robot_model()
        self.robot_state = RobotState(self.robot_model)
        self.single_pipeline_plan_request_params = PlanRequestParameters(
        self.panda,"ompl_rrtc")
        self.logger.info("MoveItPy instance created")
        self.transform_handler = transform_handler
        if self.transform_handler is None:
            self.transform_handler = TfTransformHandler()
        self.pub = self.create_publisher(Bool,"GrippedState",10)
        self.pubopen = self.create_publisher(Bool,"GrippedOpen",10)
        self.pubclose = self.create_publisher(Bool,"GrippedClose",10)
        self.grippedstate=Bool()
        self.msgTrue=Bool()
        self.msgTrue.data=True

    def plan_and_execute(self, planning_component, single_plan_parameters=None, multi_plan_parameters=None, sleep_time=0.0):
        """Helper method to plan and execute a motion."""
        self.logger.info("Planning trajectory")
        try:
            if multi_plan_parameters is not None:
                plan_result = planning_component.plan(multi_plan_parameters=multi_plan_parameters)
            elif single_plan_parameters is not None:
                plan_result = planning_component.plan(single_plan_parameters=single_plan_parameters)
            else:
                plan_result = planning_component.plan()

            if plan_result:
                self.logger.info("Executing plan")
                robot_trajectory = plan_result.trajectory
                self.panda.execute(robot_trajectory, controllers=[])
            else:
                self.logger.error("Planning failed")
        except Exception as e:
            self.logger.error(f"Error during planning or execution: {e}")
        time.sleep(sleep_time)

    def plan_arm_to_predefined_state(self, goal_state):
        #goal_state : "ready" or "extended"
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name=goal_state)
        self.plan_and_execute(planning_component=self.panda_arm,
            single_plan_parameters=self.single_pipeline_plan_request_params, sleep_time=1.0)

    def plan_arm_to_random_state(self):
        self.robot_state.set_to_random_positions()
        self.panda_arm.set_start_state_to_current_state()
        self.logger.info("Set goal state random robot state")
        self.panda_arm.set_goal_state(robot_state=self.robot_state)
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=1.0)

    def plan_arm_with_pose_stamped(self, pose_goal):
        #pose_goal : position(x,y,z) , orientation (w,x,y,z)
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=1.0)

    def plan_arm_with_constraints(self, arm_joint_values):
        #arm_joint_values : panda_link 1,2,3,4,5,6,7
        self.panda_arm.set_start_state_to_current_state()
        self.robot_state.joint_positions = arm_joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.robot_model.get_joint_model_group("panda_arm"))
        self.panda_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=1.0)

    def plan_arm_with_multiple_pipelines(self):
        self.panda_arm.set_start_state_to_current_state()
        self.panda_arm.set_goal_state(configuration_name="ready")
        multi_pipeline_plan_request_params = MultiPipelinePlanRequestParameters(
            self.panda, ["ompl_rrtc", "pilz_lin", "chomp_planner"])
        self.plan_and_execute(planning_component=self.panda_arm,
            multi_plan_parameters=multi_pipeline_plan_request_params, sleep_time=1.0)
        
    def plan_hand_to_predefined_state(self, goal_state):
        #goal_state : "open" or "close"#
        self.panda_hand.set_start_state_to_current_state()
        self.panda_hand.set_goal_state(configuration_name=goal_state)
        self.plan_and_execute(planning_component=self.panda_hand, 
            single_plan_parameters=self.single_pipeline_plan_request_params, sleep_time=0.5)

    def plan_hand_with_constraints(self, finger_joint_values):
        #finger_joint_values : panda_finger_joint1,2 : 0.0 to 0.04
        self.panda_hand.set_start_state_to_current_state()
        self.robot_state.joint_positions = finger_joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=self.robot_state,
            joint_model_group=self.robot_model.get_joint_model_group("hand"))
        self.panda_hand.set_goal_state(motion_plan_constraints=[joint_constraint])
        self.plan_and_execute(planning_component=self.panda_hand, 
            single_plan_parameters=self.single_pipeline_plan_request_params, sleep_time=0.5)
        
    def plan_hand_with_constraints_to_grip(self):
        finger_joint_values = {
            "panda_finger_joint1": 0.009,
            "panda_finger_joint2": 0.009}
        self.plan_hand_with_constraints(finger_joint_values)

    def shutdown(self):
        """Shutdown the ROS client library."""
        rclpy.shutdown()

    def plan_arm_to_sphere(self, orientation=None):
        if not self.transform_handler.latest_translation:
            self.logger.error("Failed to get the latest object position.")
            return
        
        self.panda_arm.set_start_state_to_current_state()

        # Set default orientation if none provided
        if orientation is None:
            orientation = {}

        # Create and set pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.position.x = self.transform_handler.latest_translation['x']
        pose_goal.pose.position.y = self.transform_handler.latest_translation['y']
        pose_goal.pose.position.z = self.transform_handler.latest_translation['z']
        pose_goal.pose.orientation.w = orientation.get('w', 0.0)
        pose_goal.pose.orientation.x = orientation.get('x', 0.0)
        pose_goal.pose.orientation.y = orientation.get('y', 0.0)
        pose_goal.pose.orientation.z = orientation.get('z', 0.0)

        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=1.0)

    def plan_arm_hand_to_pick(self, orientation=None, height=0.3):
        """Plan and execute a motion to a specified pose using PoseStamped."""
        if not self.transform_handler.latest_translation:
            self.logger.error("Failed to get the latest object position.")
            return
        
        self.panda_arm.set_start_state_to_current_state()

        # Set default orientation if none provided
        if orientation is None:
            orientation = {}

        # Adjust the z translation to top of the object
        adjusted_z = self.transform_handler.latest_translation['z'] + height

        # Create and set pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.position.z = adjusted_z
        pose_goal.pose.position.x = self.transform_handler.latest_translation['x']
        pose_goal.pose.position.y = self.transform_handler.latest_translation['y']
        pose_goal.pose.orientation.w = orientation.get('w', 0.0)
        pose_goal.pose.orientation.x = orientation.get('x', 0.0)
        pose_goal.pose.orientation.y = orientation.get('y', 0.0)
        pose_goal.pose.orientation.z = orientation.get('z', 0.0)

        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=0.5)
        self.logger.info("Moved to Top")
        #self.plan_hand_to_predefined_state(goal_state="open")
        self.pubopen.publish(self.msgTrue)

        # Adjust the z translation by adding 0.127 (hand dimension) minus 0.02 (object dimension)
        adjusted_z = self.transform_handler.latest_translation['z'] + 0.127 - 0.02
        pose_goal.pose.position.z = adjusted_z
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=0.5)
        self.logger.info(f"Moved to grip position: {pose_goal.pose.position.z}")
        
        self.grippedstate.data = True
        self.pub.publish(self.grippedstate)        
        #self.plan_hand_with_constraints_to_grip()
        self.pubclose.publish(self.msgTrue)
        time.sleep(1.0)

        adjusted_z = self.transform_handler.latest_translation['z'] + height
        pose_goal.pose.position.z = adjusted_z
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=0.5)
        self.logger.info(f"Moved back to pose: {pose_goal.pose.position.z}")

    def plan_arm_hand_to_place(self, orientation=None, height=0.3):
        """Plan and execute a motion to a specified pose using PoseStamped."""
        if not self.transform_handler.latest_translation:
            self.logger.error("Failed to get the latest object position.")
            return
        
        self.panda_arm.set_start_state_to_current_state()

        # Set default orientation if none provided
        if orientation is None:
            orientation = {}

        # Adjust the z translation to top of the object
        adjusted_z = self.transform_handler.latest_translation['z'] + height

        # Create and set pose goal
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.position.z = adjusted_z
        pose_goal.pose.position.x = self.transform_handler.latest_translation['x']
        pose_goal.pose.position.y = self.transform_handler.latest_translation['y']
        pose_goal.pose.orientation.w = orientation.get('w', 0.0)
        pose_goal.pose.orientation.x = orientation.get('x', 0.0)
        pose_goal.pose.orientation.y = orientation.get('y', 0.0)
        pose_goal.pose.orientation.z = orientation.get('z', 0.0)

        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=0.5)

        # Adjust the z translation by adding 0.127 (hand dimension) minus 0.02 (object dimension) 
        adjusted_z = self.transform_handler.latest_translation['z'] + 0.127 - 0.02 + 0.05
        pose_goal.pose.position.z = adjusted_z
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=0.5)
        self.logger.info(f"Moved to grip position: {pose_goal.pose.position.z}")
        
        self.grippedstate.data = False
        self.pub.publish(self.grippedstate)        
        #self.plan_hand_to_predefined_state(goal_state="open")
        self.pubopen.publish(self.msgTrue)

        adjusted_z = self.transform_handler.latest_translation['z'] + height
        pose_goal.pose.position.z = adjusted_z
        self.panda_arm.set_goal_state(pose_stamped_msg=pose_goal, pose_link="panda_link8")
        self.plan_and_execute(planning_component=self.panda_arm, sleep_time=0.5)

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
        # Create buttons and place them in the grid
        tk.Label(self.root, text="Originial Arm Control", font=("Arial", 14, "bold")).grid(row=0, column=0, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm to Ready State", command=self.plan_arm_to_ready_state).grid(row=1, column=0, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm to Extended State", command=self.plan_arm_to_extended_state).grid(row=2, column=0, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm to Random State", command=self.plan_arm_to_random_state).grid(row=3, column=0, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm with PoseStamped", command=self.plan_arm_with_pose_stamped).grid(row=4, column=0, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm with Constraints", command=self.plan_arm_with_constraints).grid(row=5, column=0, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm with Multiple Pipelines", command=self.plan_arm_with_multiple_pipelines).grid(row=6, column=0, pady=5, padx=5, sticky='ew')

        tk.Label(self.root, text="Originial Hand Control", font=("Arial", 14, "bold")).grid(row=0, column=1, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Hand to Open State", command=self.plan_hand_to_open_state).grid(row=1, column=1, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Hand to Close State", command=self.plan_hand_to_close_state).grid(row=2, column=1, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Hand with Constraints to Grip", command=self.plan_hand_with_constraints_to_grip).grid(row=3, column=1, pady=5, padx=5, sticky='ew')

        tk.Label(self.root, text="Customized Program", font=("Arial", 14, "bold")).grid(row=0, column=2, pady=5, padx=5, sticky='ew') 
        tk.Button(self.root, text="Plan Arm_Hand to Pick Cube1", command=self.plan_arm_hand_to_pick_cube1).grid(row=1, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm_Hand to Pick Cube2", command=self.plan_arm_hand_to_pick_cube2).grid(row=2, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm_Hand to Pick Cylinder", command=self.plan_arm_hand_to_pick_cylinder).grid(row=3, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Plan Arm to Sphere", command=self.plan_arm_to_sphere).grid(row=4, column=2, pady=5, padx=5, sticky='ew')
        tk.Button(self.root, text="Pick & Place Cube", command=self.pick_and_place_cube).grid(row=5, column=2, pady=5, padx=5, sticky='ew')


        tk.Button(self.root, text="Exit", command=self.exit_program).grid(row=7, column=0, columnspan=3, pady=20, sticky='ew')

        for i in range(3):
            self.root.grid_columnconfigure(i, weight=1)

    def plan_arm_to_ready_state(self):
        self.motion_planner.plan_arm_to_predefined_state(goal_state="ready")
        messagebox.showinfo("Info", "Planning arm to ready state completed.")

    def plan_arm_to_extended_state(self):
        self.motion_planner.plan_arm_to_predefined_state(goal_state="extended")
        messagebox.showinfo("Info", "Planning arm to extended state completed.")

    def plan_arm_to_random_state(self):
        self.motion_planner.plan_arm_to_random_state()
        messagebox.showinfo("Info", "Planning arm to random state completed.")

    def plan_arm_with_pose_stamped(self):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "panda_link0"
        pose_goal.pose.position.x = 0.28
        pose_goal.pose.position.y = -0.2
        pose_goal.pose.position.z = 0.5
        pose_goal.pose.orientation.w = 1.0
        pose_goal.pose.orientation.x = 0.0
        pose_goal.pose.orientation.y = 0.0
        pose_goal.pose.orientation.z = 0.0
        self.motion_planner.plan_arm_with_pose_stamped(pose_goal)
        messagebox.showinfo("Info", "Planning arm with PoseStamped completed.")

    def plan_arm_with_constraints(self):
        arm_joint_values = {
            "panda_joint1": -1.0,
            "panda_joint2": 0.7,
            "panda_joint3": 0.7,
            "panda_joint4": -1.5,
            "panda_joint5": -0.7,
            "panda_joint6": 2.0,
            "panda_joint7": 0.0,}
        self.motion_planner.plan_arm_with_constraints(arm_joint_values)
        messagebox.showinfo("Info", "Planning arm with constraints completed.")

    def plan_arm_with_multiple_pipelines(self):
        self.motion_planner.plan_arm_with_multiple_pipelines()
        messagebox.showinfo("Info", "Planning arm with multiple pipelines completed.")

    def plan_hand_to_open_state(self):
        self.motion_planner.plan_hand_to_predefined_state(goal_state="open")
        messagebox.showinfo("Info", "Planning hand to open state completed.")

    def plan_hand_to_close_state(self):
        self.motion_planner.plan_hand_to_predefined_state(goal_state="close")
        messagebox.showinfo("Info", "Planning hand to close state completed.")

    def plan_hand_with_constraints_to_grip(self):
         self.motion_planner.plan_hand_with_constraints_to_grip()
         messagebox.showinfo("Info", "Planning hand to grip completed.")
            
    def plan_arm_hand_to_pick(self, topic):
        self.transform_handler.start_subscription(topic=topic)
        # Wait for the latest translation to be updated by the callback
        time.sleep(1)
        if self.transform_handler.latest_translation:
            orientation = {'w': 0.0037637, 'x': 0.922604, 'y': -0.385712, 'z': -0.00372501}
            self.motion_planner.plan_arm_hand_to_pick(orientation=orientation)
            messagebox.showinfo("Info", "Planning arm to pose completed.")
        else:
            messagebox.showerror("Error", "Failed to receive translation.")
    
    def plan_arm_hand_to_pick_cube1(self):
        self.plan_arm_hand_to_pick(topic="tf_cube1")

    def plan_arm_hand_to_pick_cube2(self):
        self.plan_arm_hand_to_pick(topic="tf_cube2")

    def plan_arm_hand_to_pick_cylinder(self):
        self.plan_arm_hand_to_pick(topic="tf_cylinder")

    def plan_arm_to_sphere(self):
        self.transform_handler.start_subscription(topic='tf_sphere')
        # Wait for the latest translation to be updated by the callback
        time.sleep(1)
        if self.transform_handler.latest_translation:
            orientation = {'w': 0.0037637, 'x': 0.922604, 'y': -0.385712, 'z': -0.00372501}
            self.motion_planner.plan_arm_to_sphere(orientation=orientation)
            messagebox.showinfo("Info", "Planning arm to pose completed.")
        else:
            messagebox.showerror("Error", "Failed to receive sphere translation.")

    def pick_and_place_cube(self):
        self.transform_handler.start_subscription(topic="tf_cube2")
        # Wait for the latest translation to be updated by the callback
        time.sleep(1)
        if self.transform_handler.latest_translation:
            orientation = {'w': 0.0037637, 'x': 0.922604, 'y': -0.385712, 'z': -0.00372501}
            self.motion_planner.plan_arm_hand_to_pick(orientation=orientation)
        self.transform_handler.start_subscription(topic="tf_cube1")
        time.sleep(1)
        if self.transform_handler.latest_translation:
            orientation = {'w': 0.0037637, 'x': 0.922604, 'y': -0.385712, 'z': -0.00372501}
            self.motion_planner.plan_arm_hand_to_place(orientation=orientation)
            self.trial+=1
            TrialNo = "Trial No: " + str(self.trial)
            messagebox.showinfo("Info", "Pick & Place Cube Completed.\n" + TrialNo)
        else:
            messagebox.showerror("Error", "Failed to receive translation.")
      
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