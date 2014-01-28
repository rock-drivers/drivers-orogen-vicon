Read this file before using the Vicon component in RoCK.

Requirements:

- Windows computer with 2 network cards. One connected to the Vicon cameras and one connected to your LAN.
- Vicon Nexus Software v1.4+ (Vicon Tracker is even better but not explained here).
- Additional (Linux) machine where you run the RoCK Vicon component.

Instructions:

- Open the Vicon Nexus Software
- Create a subject then create a segment using at least three markers (origin, primary axis and secondary axis). When naming your subjects and segments don't use blank spaces.
- You will need to configure the Vicon Nexus in order to be able to provide in real-time (Live mode) the Global Translation and Orientation of your segment.
  Otherwise it only tracks 3D positions of the markers that it sees. For that you will need to:
	1. Go to the "System" tab inside the "Resources" windows (left part of the main Nexus window)
	2. Click in the "Local Vicon System [100Hz]"
	3. See below in "Properties" sub window, in "Core Processor" field
	4. In "Processing level:" set it to "kinematic fit" in the deployed list.
	5. Once you do this, you should be able to see in Live mode your segment having attached to its origin marker a coordinate system with the 3 axes.
- Leave the Vicon Nexus running just like that, you don't need to start any capture process. Just make sure your subject is well seen by the cameras and markers are not blinking (loosing track).
- Go to your Linux machine and execute the test_vicon ruby script with three arguments: The IP of the windows machine that is used to connect to the LAN, the subject name and the segment name, e.g.:

$ ruby test_vicon.rb 10.0.0.11 my_subject my_segment

- Don't use blank spaces to name your subject/segment or you will mess it up with the arguments. Arguments are case sensitive.

The script outputs the 3D tranlation vector of the subject being tracked.
=