install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dual_arm_panda.py
  scripts/pnp_panda_python.py
  scripts/simple_version.py
  scripts/z_old_dual_arm_panda.py
  scripts/dual_arm_joint_commands_filter.py
  scripts/keyboard_input.py
  scripts/gesture_recognition.py
  scripts/gesture_servo_control.py
  scripts/non_humanoid.py
  scripts/twist_publish.py
  DESTINATION lib/${PROJECT_NAME}
)