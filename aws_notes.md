# AWS Robomaker Deployment

Notes on attempts to get this simulation deployed to AWS Robomaker, following [Developer Guide](https://docs.aws.amazon.com/robomaker/latest/dg/what-is-robomaker.html)

AWS separates the simulation application from the robot (controller) application.  However, since I've got a package with both, I decided there'd just be one application bundle.  I *think* you can separate the apps again later by specifying different launch files on AWS.  Therefore I started by creating two separate launch files: one for the example robot and similator only, and one for the controller.

Following [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-new.html):

For the control app:
* Skipped Steps 1 to 3 as I already had the package set up
* Step 4: create `src/farscope_group_project` folder and put `__init__.py` in there.  Not sure I needed this as I'm not exporting any modules.
* Skipped 5 to 7 as I already had my scripts and launch file
* Step 8 was critical - see git log for changes to `CMakeLists.txt` including all Python options enabled and explicitly installing all executable scripts.
* Skipped 9 as I'd worked on my `package.xml` already
* Carefully followed 10 but not convinced I need it

For the simulation app:
* Skipped 1 to 5 as already done *apart from* changing Gazebo GUI default to `false` in the simulator-only launch file.

I also needed to add the following to the CMakeLists.txt of the `neo_simulation` package so my built package could access its contents:
```
install(DIRECTORY launch scripts robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

Then moving on to [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/application-build-bundle.html#install-colcon) to install the `colcon` build tool.  I had to `sudo` the two `apt` installs but I did the `pip3` installs as local user, which worked OK but put the executable in `~/.local/bin/`.  A warning message said I ought to add that to my path, but for now I'm leaving it as is.

Also ensured relevant directories were included in the `install` directive mentioned.  Found by trial and error, but not sure this was key in the end.

Next move to the workspace root directory (so `cd ../..` from the package directory) and run `~/.local/bin/colcon build`.  I had to delete the `build` directory first as that had been done by `catkin_make` and `colcon` complained.  Might be a workaround. 

Then test by moving to a fresh terminal, without ROS set up (I don't have the `source` thing in my `.bashrc`), and run:
```
source install\setup.bash
roslaunch farscope_group_project farscope_example_robot_simulate.launch
```
Note the `setup.sh` version in the instructions didn't work for me but the `.bash` one did.  If successful, you should see the Gazebo GUI and the robot picking up a target.

Then on to `~/.local/bin/colcon bundle` - first I got an error "have you set your keys correctly?" which I fixed using the `wget` command [here](https://github.com/colcon/colcon-bundle/issues/100).

That's all for now - next steps are to get it up to AWS.