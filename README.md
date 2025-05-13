# UMRT ROS Template
ROS project repository template for the University of Manitoba Robotics Team.

Each new project must:
1. Fill in missing fields in package.in.xml
2. Fill in project name in CMakeLists.txt
3. Go into `umrt-build` package settings and give the new repo read permission
4. Go into `umrt-apt-image` package settings and give the new repo read permission
5. Go into `UMRoboticsTeam` organisation secrets and add the new repo to:
   - `APT_DEPLOY_KEY`
   - `APT_SIGNING_KEY`
6. Replace example files with real code, add source files to CMake targets, document it with Doxygen, and proceed