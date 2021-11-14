# ur3-project
Location for ECE 470 Introduction to Robotics Project with the UR3 Arm

### Authors:
- Nate Carland
- Samuel Brunkow
- Zach Williams

### Installation Instructions:
- Clone github project into the catkin workspace source folder
- Run `source install.bash` script to install the necessary submodules.
- To reinstall the submodules for whatever reason, run `source install.bash -r`.
- `lab2pkg_py` needs to be in the source of the workspace as well and after run `catkin_make`.

### Execution Instructions:
- Source the respective workspace's `setup.bash` before running `ros` commands.
- Also run `source src/setup.bash` in order to set the necessary environment variables.
- In one terminal, kickoff the project's gazebo simulation: `roslaunch ur3_project project.launch`
- In another terminal, run the execution script: `rosrun ur3_project run.py`
- You should now be able to view the chess board from the perspective of the camera. 

### Links:
- [ur3-project Github](https://github.com/zjwilliams20/ur3-project)
- [ECE 470 Lab Page](http://coecsl.ece.illinois.edu/ece470/)
- [ECE 470 Course Page](https://publish.illinois.edu/ece470-intro-robotics/)
