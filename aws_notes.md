# AWS Robomaker Deployment

Notes on attempts to get this simulation deployed to AWS Robomaker.

## Building it on AWS CLoud 9

I did all these initial steps logged as an Adminstrator IAM user - [see here](https://docs.aws.amazon.com/IAM/latest/UserGuide/getting-started_create-admin-group.html).

* Create an "input bucket" and an "output bucket" on S3 using [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-simjob.html#application-simjob-bucket)
* Set up an IAM role following [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-simjob.html#application-simjob-security) including bucket access and account number
* Create a cloud9 environment following [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/cloud9-create-ide.html).  I used ROS Melodic with Gazebo 9.
* In the IDE terminal, creating a new workspace in my home directory `mkdir -p farscope_ws\src`
* Cloning this and the two dependency repositories into `farscope_ws\src` using `git` in the terminal
* Add the following to the `install` section of `neo_simulation\CMakeLists.txt` so my built package could access its contents:
```
install(DIRECTORY launch scripts robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```
* Click Run -> Build -> Edit Configurations and make a new one for `farscope_ws`.  Run it.
* Before I could get bundling to to work, I had to run the following, copying [this discussion](https://forums.aws.amazon.com/thread.jspa?messageID=950062):
```
sudo pip3 uninstall colcon-bundle colcon-ros-bundle
pip3 install colcon-bundle colcon-ros-bundle
```
* Click Run -> Bundle -> Edit Configurations and make a new one for `farscope_ws`.  Run it.
* Click Run -> Launch Simulation -> Edit... (you get the idea).  Choose the same bundle `output.tar` for both sim and robot applications.  Use the S3 buckets and IAM roles from previous steps, and remember that the "input bucket" is where you upload the apps to.

### Sharing it with other users, e.g. students

* Create an IAM group for your users and add users
* Attach policies `AWSCloud9EnvironmentMember`, `AmazonS3ReadOnlyAccess`, `CloudWatchLogsReadOnlyAccess` and `AWSRoboMaker_FullAccess` to the group
* Logged in as Administrator, go to your environment and use the "share" button to invite those users.
* Share the URL from the `share` panel in the Cloud9 environment

## Building it: local Linux work

> I never got this to work!  Can build and bundle but it refused to run on AWS, citing missing launch files.

Following [Developer Guide](https://docs.aws.amazon.com/robomaker/latest/dg/what-is-robomaker.html)

### Preparing the Workspace

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

### Preparing to Build

Then moving on to [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/application-build-bundle.html#install-colcon) to install the `colcon` build tool.  I had to `sudo` the two `apt` installs but I did the `pip3` installs as local user, which worked OK but put the executable in `~/.local/bin/`.  A warning message said I ought to add that to my path, but for now I'm leaving it as is.

Also ensured relevant directories were included in the `install` directive mentioned.  Found by trial and error, but not sure this was key in the end.

### Building the Workspace

* Start in a new terminal with no ROS awarness and move to the workspace root directory (so `cd ../..` from the package directory)
* Delete the `build` `install` `bundle` directories
* Run `source /opt/ros/melodic/setup.bash`
* Run `rosdep install --from-paths src --ignore-src -r -y`
* Run `~/.local/bin/colcon build`.  

Then test by moving to a fresh terminal, without ROS set up (I don't have the `source` thing in my `.bashrc`), and run:
```
source install\setup.bash
roslaunch farscope_group_project farscope_example_robot_simulate.launch
```
Note the `setup.sh` version in the instructions didn't work for me but the `.bash` one did.  If successful, you should see the Gazebo GUI and the robot picking up a target.

Then on to `~/.local/bin/colcon bundle` - first I got an error "have you set your keys correctly?" which I fixed using the `wget` command [here](https://github.com/colcon/colcon-bundle/issues/100).  Then it worked and I had a new `output.tar` file in a `bundle` subdirectory.

That's all for now - next steps are to get it up to AWS.

### Deploying it: AWS work

Following [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/create-robot-application.html) and doing it via the web console.

* Sign in to AWS console.  I use the "Ireland" region as the UK one doesn't yet have robomaker.  I signed it with my root ID as I've not got my head round IAM yet.
* At step 3, I followed "new S3 destination", created a new `farscope-group-project-bucket` (no underscores allowed) with default settings, and uploaded the `output.tar` file that my `colcon bundle` step created.  Then chose that from the "Browse S3" button next to x86_64 and clicked "create".

Moving on to the [simulation app instructions](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-application.html) - largely same again, choosing the same `.tar` bundle and selecting ROS Melodic and Gazebo 9.

Now moving on to the actual simulation run using [these instructions](https://docs.aws.amazon.com/robomaker/latest/dg/create-simulation-job.html)

Step 1 Configure:

* Job duration to 5 minutes
* Fail behaviour to `fail`
* For IAM role I chose `make new role` and gave it name `farscope-sim-role`
* For output, I followed `new S3 destination` and made a new `sim-output` folder in my `farscope-group-project-bucket` before selecting that as destination

Step 2 Specify Robot App

* Chose my existing robot app and left version at default `$LATEST`
* Entered `farscope_group_project` as package and `farscope_example_robot_control.launch` as launch file

Step 3 Specify Simulation App

* Chose my sim app and left version at default `$LATEST`
* Entered `farscope_group_project` as package and `farscope_example_robot_simulator_only.launch` as launch file

Step 4 Review

* Just clicked OK!

Failed, so click on the logs to see output.  Consistently getting error `not found: "/home/aeagr/ros/group_project_ws/devel/local_setup.sh"` which makes sense as that's my local machine path and wouldn't exist on AWS.  Some information about a similar problem [here](https://github.com/colcon/colcon-bundle/issues/103) but haven't made sense of it yet.
