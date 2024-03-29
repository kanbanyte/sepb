<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s): @vkach, @Slothman1, @Huy-GV, @NickMcK14, @finnmcgearey
	Year: 2023
-->

`Usability Assessment Report`

<!-- TOC ignore:true -->
## Industry Project 24
|Name|Position|Email|
|:-|:-|:-|
|@Slothman1|Team Leader/Client Liaison|id@swin.student.edu.au|
|@dau501|Development Manager/Planning Manager|id@swin.student.edu.au|
|@finnmcgearey|Support Manager/Developer|id@swin.student.edu.au|
|@vkach|Quality Manager/Developer|id@swin.student.edu.au|
|@NickMcK14|Support Manager/Developer|id@swin.student.edu.au|
|@Huy-GV|Quality Manager/Developer|id@swin.student.edu.au|

<div class="page"/><!-- page break -->

<!-- TOC ignore:true -->
# Document Sign Off
|Name|Position|Signature|Date|
|:-|:-|:-|:-|
|@Slothman1|Team Leader/Client Liaison|student\_signature(&emsp;)|DD/MM/2023|
|@dau501|Development Manager/Planning Manager|student\_signature(&emsp;)|DD/MM/2023|
|@finnmcgearey|Support Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|
|@vkach|Quality Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|
|@NickMcK14|Support Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|
|@Huy-GV|Quality Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|

<!-- TOC ignore:true -->
# Table of Contents
<!-- TOC -->

* [Executive Summary](#executive-summary)
* [Usability Tasks](#usability-tasks)
* [Usability Metrics](#usability-metrics)
	* [Completed Scenarios](#completed-scenarios)
	* [Errors](#errors)
	* [Client Evaluations](#client-evaluations)
* [Reporting Results](#reporting-results)

<!-- /TOC -->

<div class="page"/><!-- page break -->

# Executive Summary
This document is a reflective analysis of the methodology utilised to evaluate the usability of the system.\
The evaluation is done through having a participant(s) perform actions in the system in a controlled setting; as the product is autonomous,
much of these tasks are booting up and establishing the system and subsequent powering/shutting down of said system.\
The participant is trained on how to use the system through written documentation;
where confusion arises requests and receives help from supervising team members.
The incorporation of user feedback and performance metrics helped identify areas of improvement and enhanced the prototype.
With the release of the end product on the horizon, the collected and used data from usability testing has been invaluable to the constant improvement to the system.

# Usability Tasks
The Usability Evaluation provided a large amount of detail into the specific tasks or goals that the system must accomplish to be considered a success,
mainly focusing around how the user (or test participant) will need to interact with the entire system.
Goals and aims were also listed to ensure that the system was able to meet any requirements that the client had set at the start of the project,
while maintaining the goal to achieve a seamless and easy experience.
This section will address the requirements set out in the Usability Evaluation Plan, and discuss whether they were achieved or not.

The robot arm was able to successfully achieve all of its requirements, problems arising along the way, but being solved with further problem solving by the team.
The robot arm initially was very difficult to set up and connect to the PC.
This required many commands to be sequentially completed to successfully launch the system.
While the system still requires multiple commands,
scripts have been written to complete several at a time in an automated fashion, there by reducing the inputs need from the user.
The robot arm is also able to stop automatically if impacted in an adverse fashion, and
this is a built in feature with the robot arm alongside the emergency buttons in the lab environment.
However, functionality has been developed to ensure that, post an event stops the robot mid movement,
the robot arm is able to move to a pre determined position in a safe manner, and continue it's normal function.

Asides from the above addressed criteria, the system is also able to complete the main tasks set out at the start of project.
Using the CV algorithms that have been developed, the system is able to successfully detect the presence of chips and cases in the lab environment, and
provide the correct locations of the respective chips and cases to the system for the robot arm to move to the correct locations and pick up the parts.
The CV algorithms are also able to detect if all the required parts have been accumulated on the assembly tray and is ready to be provided to the assembly worker, and
vice versa, the CV algorithm is also able to detect when the assembly tray has been emptied by the worker, and
can instruct the system to return the empty assembly tray to its original position.

Each of the CV algorithms are also able to present images with the results of the detection algorithms.
This allows the user to be able to verify the results of the algorithms and ensure that the outputs are correct, and
the actions implemented by the cobot achieve the required results.

Scalability was also a priority during development of the system.
Should any new positions be needed in the future, the user or developer can easily record and add positions, and
the new positions can easily be added to the procedures of the robot.
This design also ensures that should the user or developer makes changes to the specific movements of the robot, performance will not be affected.

Overall, as the system is able to achieve all the tasks that were laid out in the [Usability Evaluation Plan](evaluation.md),
the system can be considered a success and classified as straight-forward and intuitive,
with future development and customisation by the user a key priority in its design.

<div class="page"/><!-- page break -->

# Usability Metrics
Usability metrics serve as objective and data-driven tools for evaluating different parts of Project24;
providing insight into the system's performance, efficiency, and client satisfaction.
These metrics help understand how well Project24 aligns with the clients expectations and requirements, and where improvements can be made.

The following section looks at the usability metrics used to assess Project24 performance,
including the completed scenarios and the errors found.
By analysing these metrics, HeeHooVision aims to identify strengths and weaknesses within the system.
The analysis also identifies where refinements and improvements can be made.

## Completed Scenarios
The following usability scenarios have been evaluated:
* The vision system detects and guides the cobot to move the chips to the correct compartment in the white tray.
* The vision system detects and guides the cobot to move the case to the correct compartment in the white tray.
* The cobot moves the battery to the correct compartment in the white tray.
* The cobot moves the PCB shell to the correct compartment in the white tray.
* The vision system detects a correctly filled tray and guides the cobot to move it to the human operator working space.
* The vision system detects an emptied tray from the human operator working space and guides the cobot to move it back to be filled up again.
* The cobot makes no move in the following scenario:
	* The delivered tray has not been emptied by the human operator.
	* The trays to be delivered has not been filled with correct items.
	* The trays are missing/not represent.
* The cobot stops immediately when the emergency button is pressed.

<div class="page"/><!-- page break -->

## Errors
In the evaluation of Project24, several critical aspects of usability came to light, with a particular focus on the examination of errors.
These errors encompassed issues related to positioning accuracy, object recognition failures, communication glitches, and deviations in task execution.

Of notable concern were the instances of positioning inaccuracies,
where the system occasionally failed to accurately position the robotic arm and gripper, resulting in misalignments during task execution.
These inaccuracies were categorised based on their severity,
ranging from critical errors that significantly disrupted task performance to major errors with a noticeable adverse impact and
minor errors with comparatively minor consequences.

The evaluation also identified object recognition failures,
where the system encountered challenges in correctly identifying and distinguishing between objects, leading to errors in object handling.
Similar to positioning inaccuracies, these errors were classified into critical, major, and minor categories.

Furthermore, the evaluation documented instances of communication glitches between system components, which presented obstacles to the seamless execution of the task.
Once again, these glitches were assessed for severity and categorised as critical, major, or minor.

Lastly, deviations in task execution were acknowledged, with the system occasionally deviating from the intended path,
even when there were no overt issues with positioning or object recognition.
These deviations, like other error categories, underwent an evaluation that considered their severity within the critical, major, and minor spectrum.

The simultaneous documentation of these errors and
the classification of their severity provide a robust foundation for the improvement of the system's performance and
reliability through a process of iterative refinement.

## Client Evaluations
In the assessment of Project24, subjective evaluations play a pivotal role.
The clients feedback offers valuable insights, helping the HeeHooVision identify trends and patterns that guide iterative improvements.
By systematically collecting and assessing client evaluations, a bridge is established between technical development and practical needs.
This holistic approach ensures that the system continually evolves to better meet client expectations.

The incorporation of client perspectives and feedback into the development process is a commitment taken seriously.
It drives continual enhancements to the Robot Vision System, resulting in a more efficient solution.
Through this client-centric approach, the aim is to develop a system that truly serves its purpose.

# Reporting Results
Overall, the testing covered in this document leaves enough confidence in the system for it to achieve the goal of being autonomous.\
Going off the established criteria covered earlier, the system is very usable and meets the expectations laid out.
