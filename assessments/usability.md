<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s):
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

<div class="page"/><!-- page break -->

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
# Usability Tasks
The Usability Evaluation provided a large amount of detail into the specific tasks or goals that the system must accomplish to be considered a success,
mainly focusing around how the user (or test participant) will need to intereact with the entire system.
Goals and aims were also listed to ensure that the system was able to meet any requirements that the client had set at the start of the project,
while maintaining the goal to achieve a seemless and easy experience. This section will address the requirements set out in the Usability Evaluation Plan,
and discuss whether they were achieved or not.

The robot arm was able to successfully achieve all of its requirements, problems arising along the way, but being solved with further problem solving by the team.
The robot arm initially was very difficult to set up and connect to the PC. This required many commands to be sequentially completed to successfully lauch the system.
While the system still requires multiple commands, scripts have been written to complete sevaral at a time in an automated fashion,
there by reducing the inputs need from the user. The robot arm is also able to stop automatically if impacted in an adverse fashion,
and this is a built in feature with the robot arm alongside the emergency buttons in the lab environment. However, functionality has been developed to ensure that,
post an event stops the robot mid movement, the robot arm is able to move to a pre determined position in a safe manner, and continue it's normal function.

Asides from the above addressed criteria, the system is also able to complete the main tasks set out at the start of project.
Using the CV algorithms that have been developed, the system is able to successfully detect the presence of chips and cases in the lab environment,
and provide the correct locations of the respective chips and cases to the system for the robot arm to move to the correct locations and pick up the parts.
The CV algorithms are also able to detect if all the required parts have been accumulated on the assembly tray and is ready to be provided to the assembly worker, 
and vice versa, the CV algorithm is aslo able to detect when the assembly tray has been emptied by the worker,
and can instruct the system to return the empty assembly tray to its original position.

Overall, as the system is able to achieve all the task that were laid out in the Usability Evaluation Plan,
the system can be considered a success and classified as straight-forward and intuitive.

# Usability Metrics
## Completed Scenarios
## Errors
## Client Evaluations
# Reporting Results
