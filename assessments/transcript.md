# Preliminary Video Transcript Draft
<!--
	Author: @vkach
	Editor(s): @dau501
	Year: 2023
-->

Hi!
We're HeeHooVision.
Last semester, you may remember the video we presented about setting up a robot arm to be able to pick and place parts that will be used along an assembly line.
Well, we're happy to report that we've made significant progress since you last heard from us and have completed to the project,
with tons of research and development plus consistent communication with the client to achieve a product that is both satisfactory and
will be a solid foundation for the client to continue development on in the future.

Over the duration of this semester, our group has been hard at work training and developing machine learning models and
making them into easy to use and implement Python packages that the client will be able to easily use and continue developing if they so desire in the future.

> [Start showing images of models detecting objects in images]

As you can see, we're able to capture images from the camera that is now mounted securely above the workstation and
run specific models on specific cropped portions of the image, allowing us to develop multiple,
high accurate models that can detect the locations of chips that are present and the location of the next case.
We're also able to detect the state of the assembly trays as they get populated through out the pick and place task,
once they have been emptied by the workstation attendant, who is assembly the final product.

Using something call ROS2, or the second iteration of the Robot Operating System,
we're able to set up communication between the machine learning models and the robot arm, passing the preassigned numbers for the chips and cases.
Once the code that operates the robot arm receives this data,
functionality has been developed for the code to be able to automatically determine locations position that robot will need to move through,
to ensure that any parts that it picks and moves has no chance of interacting or bumping into its surrounding environment.

All of this now enables the Factory of the Future, our client, to better support research into the use and aid of robotics within the assembly lines,
allowing our client to develop procedures that improve quality control and reducing human error in an assembly line,
thereby enhancing the output and efficiency that can be generated from assembly lines.

Now, let's look at the complete system, working completely independent of any user input.
As you can see, the system is correctly able to locate the location of the required chip and move the robot arm accordingly to pick up the parts.

Thanks for listening everyone, and thanks to the Factory of the Future for allowing us to work on this project and develop these systems for them.
