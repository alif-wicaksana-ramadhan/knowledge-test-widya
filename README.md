# KNOWLEDGE TEST ROBOTICS SOFTWARE ENGINEER - WIDYA ROBOTICS

## PART A

### PDF Answer

The answer is in "./part_a/main.pdf"

## PART B

### Preparation

##### Go to the Workspace Folder

```bash
  cd ./part_b
```

##### Build the Application

```bash
  catkin_make
```

##### Source the Workspace

```bash
  source ./devel/setup.bash
```

### Run the First Application

##### Run the node from prime_from_fibonacci package

```bash
  rosrun prime_from_fibonacci run_script <theta>
```

##### Example:

```bash
  rosrun prime_from_fibonacci run_script 30
```

### Run the Second Application

##### Call the roslaunch to run the node

```bash
  roslaunch pcd_reader viewer.launch pcd_file_path:=<pcd_file_path>
```

##### Example:

Use the absolute path of the pcd file.

```bash
  roslaunch pcd_reader viewer.launch pcd_file_path:=/home/alif/catkin_ws/pcd/wolf.pcd
```

An rviz window showing the pointcloud data of the object will appear. And an input will shown in the terminal, put float number as the value of how many degree should the object turns. Press 'enter' after the desired degree to rotate the object.

##### Save the rotated object to pcd file

Do this by call a service in the pcd_reader node

```bash
  rosservice call /save "savepath: {data: <save_path>}"
```

##### Example:

Use the absolute path of target direction

```bash
  rosservice call /save "savepath: {data: /home/alif/catkin_ws/rotated_object.pcd}"
```
