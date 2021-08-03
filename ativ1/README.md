## Instructions to run on Windows

1. Open a Powershell terminal.
2. Build this workspace with `colcon build`
3. Install scripts with `.\install\setup.ps1`
4. Run node with `ros2 run controller listener`
5. In another Powershell terminal, run `ros2 topic pub /setpoint_pose geometry_msgs/msg/Pose "{position: {x: 1.2, y: 3.4, z: 2.3}, orientation: {x: 0.0, y: 0.0, z: 0.5, w: 0.866}}"`