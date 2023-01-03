# ME461 Final Project:

For the final project of the class ME 461: Computer Control of Mechanical Systems, my team chose to design and program a balancing segway robot that could follow a line, detect obstacles, and make intelligent decisions using a state machine. This was a group project, and I chose to focus on the implementation of line-following using openCV and a PID control loop. I used a provided openMV camera and took advantage of the built in computational functions to detect a line. I used serial communication (SPI) to send key information such as the angle and distance from the line being followed to the microcontroller (TI Launchpad). I programmed the microcontroller to convert this information to a singular error value that indicated how far the robot was from the line. I then put this information into a PID loop that aimed to reduce this error, which ultimately succeeded in following the line while also remaining balanced.

It didn't do this very smoothly, however, and frequently lost track of the line. Future work would include better PID tuning and improved camera placement to prevent the line from leaving the camera's field of view.

Hackster.io page: https://www.hackster.io/500844/me-461-final-project-line-following-segway-robot-b88849 </br>
Robot Demo: https://youtu.be/_paMzIPxlZE
