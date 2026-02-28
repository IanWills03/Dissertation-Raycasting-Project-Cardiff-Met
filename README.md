This is contains the python scripts developed throughout my Dissertation. In order to keep this repository lightweight and focused on source code, External libraries and local environments were excluded.
if you want to run this yourself, you will need to install the following external libraries in Python:
"opencv-python" - produces a video stream from the drones camera that can be displayed 
"djitellopy" - provides a list of basic commands for the DJI Tello-Talent Drone
"numpy" - handles the mathematics required for the raycasting algorithm and 3D positioning
"matplotlib" - plots the Flight trajectory onto a graph
After this, 
#configure the motion capture system, 
#create a client server relationship using static IP addresses and command/data ports, 
#run both the motion capture and natnet python clients to capture the drones position and then 
#finally run any of the "Flight_Trajectory" files in the folder.
