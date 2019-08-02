# Kuri---Odometry-GUI
This GUI is intended to drive the Kuri, and to visually represent its odometry and laser-scan ranges as it moves. (It's still under construction, though.)

To set up this up on a new computer:
1. Pull the contents of this repository.
2. In the .html file, input the file paths of the downloaded files.
3. In kineval_rosbridge.js, input the url of the computer being used.
4. Set Kuri as the ROS_Master
5. Install rosbridge (steps available on: wiki.ros.org/rosbridge_suite )

After ssh-ing into the Kuri,
1. On the user terminal of your command-line, enter `rosrun rosbridge_server rosbridge_websocket`
2. Load the .html file on your browser.

NB:
Currently, the rotation angle of the canvas marker is not completely in sync with the Kuri. (Working on it!)
All stylistic edits are welcome.
