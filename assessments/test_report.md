<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s):
	Year: 2023
-->

`TEST REPORT`

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

* [Introduction](#introduction)
* [Results](#results)
	* [Test Items](#test-items)
	* [Test Cases](#test-cases)
		* [Tested Features](#tested-features)
		* [Functional Test Cases](#functional-test-cases)
		* [Non-functional Testing](#non-functional-testing)
	* [Testing Tasks](#testing-tasks)
* [Pass/Fail Criteria](#passfail-criteria)
	* [Product Level](#product-level)
	* [Testing Stages](#testing-stages)
* [Outcome](#outcome)

<!-- /TOC -->

<div class="page"/><!-- page break -->

# Introduction
# Results
## Test Items
## Test Cases
### Tested Features
### Functional Test Cases
Based on the features listed in the previous section, and they're specific criteria,
a series of the test cases were developed in the test plan to ensure that all the criteria were met and the final product was of a satisfactory standard.

In this section of the report, each of the test cases and the details of their success/failure will be discussed.

#### Unit Tests
* **Establish connection with Robot Arm**

	The system was able to achieve the expected results, with the Linux PC able send information to the robot arm across the network using ROS2,
	and the robot arm able to follow those instructions and move accordingly.

	Therefore, this test can be considered a success.

* **Set up Depth Camera**

	Once connect to the PC, the code that was developed by the team and the software that was provided by the camera manufacturers,
	were able to connect to the camera and view a feed from it.
	The code was also able to take and store pictures of what the camera was seeing.

	Therefore, this test can be considered a success.

* **Initialise Machine Learning (ML) Model**

	The trained ML model was able to obtain images from the camera of the workspace, correctly identify where in the appropriate feeders parts were located, and
	report them back.
	The processed images with the detected locations can also be viewed by the user, if needed.

	Therefore, this test can be considered a success.

#### Integration Tests
* **Connect ML model and Robot arm through PC**

	Using ROS2, the ML model is able to provide the required information to the ROS2 service that controls the movements of the robot arm.
	Using this information, the service is able to plan the movements required for the robot arm to pick and place the correct items in the correct locations.

	Therefore, this test can be considered a success.

* **The robot arm is able to move on command**

	After receiving the required movement information, the robot arm is able to execute them correctly and smoothly,
	with no interference or interaction with the surrounding environment.

	Therefore, this test can be considered a success.

* **The system is able to continue functioning autonomously**

	After completing a single cycle of the procedure,
	the robot system is able to repeat the process of the analysing images from the camera to find locations of parts, transmitting the data of the locations,
	planning the paths required of the robot arm, and making the robot arm conduct the actions.

	Therefore, this test can be considered a success.

### Non-functional Testing
## Testing Tasks
# Pass/Fail Criteria
## Product Level
## Testing Stages
# Outcome
