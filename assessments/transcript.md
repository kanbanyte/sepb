# Preliminary Video Transcript Draft
<!--
	Author: @vkach
	Editor(s): @dau501
	Year: 2023
-->

Hi!
We're HeeHooVision.
Last semester, you may remember the video we presented about setting up a robot arm to be able to pick and place parts that will be used along an assembly line.
Well, good news! We're happy to report that we've completed the project!
Through tons of research and development, and consistent communication with our client, we've been able to achieve the product that we set out to create!
We've built a solid foundation for the Factory of the Future to be able to continue developing this system and perfecting it for future applications.

Over the duration of this semester, our group has been hard at work training and developing machine learning models and
making them into easy to use and implement Python packages that the client will be able to easily use and continue developing.

> [Start showing images of view from camera, the cropped images and output from detection models.]

As you can see, we're able to capture images from the camera that is now mounted securely above the workstation and
run the appropriate models on specific cropped portions of the image, allowing us to develop multiple,
highly accurate models that can detect the locations of chips that are present and the location of the next case.
We're also able to detect the state of the assembly trays as they get populated throughout the pick and place task,
as well as when the tray has been emptied by the workstation attendant, who is assembling the final product.

> [Start showing images of terminal output for locations.]

Using something called ROS2, or the second iteration of the Robot Operating System,
we're able to set up communication between the machine learning models and the robot arm, passing the preassigned numbers for the chips and cases.

> [Start showing scrolling images of code that develops trajectories.]

Once the code that operates the robot arm receives this data,
functionality has been developed for the code to be able to automatically determine locations that the robot will need to move through,
to ensure that any parts that it picks and moves has no chance of interacting or bumping into its surrounding environment.

> [Images of factory of the future, and maybe some images of robotic assembly lines?]

All of this now enables the Factory of the Future, our client, to better support research into the use and aid of robotics within the assembly lines,
allowing our client to develop procedures that improve quality control and reducing human error in an assembly line,
thereby enhancing the output and efficiency that can be generated from assembly lines.

Now, let's look at the complete system, working completely independent of any user input.
As you can see, the system is correctly able to locate the location of the required chip and move the robot arm accordingly to pick up the parts.

> [Video of robot arm completing a cycle of the pick and place tasks.]

Thanks for listening everyone, and thanks to the Factory of the Future for allowing us to work on this project and develop these systems for them.
