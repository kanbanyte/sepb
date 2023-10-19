<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s):
	Year: 2023
-->

*[Note:*
*This is a sample/template document for the Test Plan.*\
*Please improve, adapt and adjust to your project needs.]*

`TEST PLAN`

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
	* [Purpose](#purpose)
	* [Scope](#scope)
	* [References Material](#references-material)
	* [Objective](#objective)
	* [Resources Required](#resources-required)
	* [Environment Requirements](#environment-requirements)
* [Test Items](#test-items)
	* [Tested Features](#tested-features)
	* [Test Cases](#test-cases)
		* [Functional Test Cases](#functional-test-cases)
		* [Non-functional Testing](#non-functional-testing)
	* [Untested Features](#untested-features)
* [Strategy](#strategy)
	* [Roles and Responsibilities](#roles-and-responsibilities)
	* [Test Deliverables](#test-deliverables)
	* [Schedule](#schedule)
	* [Risk and Contingency](#risk-and-contingency)
	* [Testing Tasks](#testing-tasks)
* [Pass/Fail Criteria](#passfail-criteria)
	* [Product Level](#product-level)
	* [Testing Stages](#testing-stages)
	* [Criteria Requirements](#criteria-requirements)
		* [Suspension Criteria](#suspension-criteria)
		* [Resumption Requirements](#resumption-requirements)
	* [Approvals](#approvals)

<!-- /TOC -->

# Introduction
This document will outline its purpose, the scope and objective of testing, as well as the specific features and cases that will be tested.
The strategy for testing will be explained including the responsibilities of each role, test deliverables, the schedule for testing, and risks and contingency plans.\
Finally, the pass and fail criteria will be expounded to demonstrate the process for the final stages of testing.

## Purpose
The purpose of the test plan is to clearly state the required features and functions of the system to test and the strategy by which they will be tested.
The test plan will allow for a more efficient testing process to prepare the developed system for its final release by outlining test features,
functional and non functional test cases, as well as the pass and fail criteria for these tests.
Additionally, the test plan will highlight the features that will not be tested to allow for a more efficient testing process.

## Scope
The scope of the test plan will encompass testing of the computer vision system and of the cobot's movement both individually and together to ensure optimal function.
Development of the overall system is still in progress, so additional tests may be required and some may not be necessary in the future.

<!-- TOC ignore:true -->
### Constraints
* Testing of the cobot's movement can be done in a simulated environment, however must ultimately be performed at the F.o.F.
* Testing of the detection models can start with images captured for the training process, but it must be validated with images directly captured by the camera at the F.o.F.
* Testing the integration of the vision system and the movement system will need to be done at the F.o.F.

<!-- TOC ignore:true -->
### Methods of Testing
Different methods of testing will be conducted at various stages in development.\
These include:
* Unit testing
* Integration testing
* System Testing
* Acceptance Testing

## References Material
Previously produced documents have been referenced to ensure that the testing that is conducted satisfies the requirements in these documents.
These documents can provide further information if required:
* Project Plan
* Software Quality Assurance Plan
* Software Requirements Specification
* Detailed Design and Implementation Report

## Objective
The objective of the testing plan is to outline the tests required and the strategy for conducting and completing testing.\
This will ensure the system is functional at a quality level and in the most efficient manner possible.

## Resources Required
The following items will be require to ensure that the system is testable:
* A computer capable of communicating with the depth camera, and running the machine learning algorithm to detect where parts are located.
* Software that is able to communicate with the robot arm and move it.
* AA batteries.
* Chips for assembly.
* Front shells for assembly.
* A participant to act as the worker assembling the parts provided by the system.

## Environment Requirements
The test environment is situated at the Factory of the Future's operation desk, housing the robot arm and assembly line.\
The following equipment and items are required:
* Depth camera mounted above the working desk.
* Adequate illumination for optimal depth camera performance.
* URe5 robot arm.
* x2 chip trays.
* A functional conveyor belt that delivers the shell.
* A battery stack.
* A front shell stack.
* A white tray on which the shells, battery and chip are delivered to the human operator.

# Test Items
The product to be tested is the robot arm upgraded with object detection technology.
As the robot arm already possesses the ability to precisely pick and move items to the human operator,
the testing strategy centers around assessing its proficiency in selecting items that are currently available,
as opposed to the pre-existing trial-and-error approach across the entirety of the trays.

## Tested Features
The tested features are divided into 3 categories: the vision system, the cobot controller software and the integration between the two.\
The vision system is further divided into the AI Models and Camera Server:
* AI models: YOLOv5 models retrained on images captured by the camera.
Images are cropped to the region containing objects of interest during training and evaluation.
There are 3 AI models: chip detection, case detection and tray detection; all of which require a different crop box on the input image.
* Camera Server: The ROS service that captures images from the camera, run inference using the AI models and respond to the cobot controller.

The following features in the vision system are tested:
* AI Models:
	* The chip detection model correctly detects chips in all available slots from a correctly cropped image.
	* The case detection model correctly detects cases in all available positions on the case rack from a correctly cropped image.
	* The tray detection model correctly detects all three classes of trays from a correctly cropped image: Full, Partially Full and Empty.
* Camera Server:
	* A stable connection between the ZED camera and camera server is established.
	* Appropriate camera settings specified in a YAML configuration file are applied.
	* Images are cropped as specified in the configuration file before being fed to the AI models for evaluation.
	* The position of the chip and case to pick up or the best tray movement is calculated from the detections made by the AI models.

The following features in the robot system are tested:
* The cobot moves to pick up items from valid positions and transfer them to the tray.
* The cobot moves the trays from and to the human operator.

The following features of the integrated system are tested:
* Initiating the camera server and establishing a connection with the cobot controller.
* The camera server responds to requests from the cobot controller with positions of requested items or best tray movement to execute.
* The cobot controller performs no movement if directed by the camera server or if the returned position is invalid.

## Test Cases
The creation of well-structured test cases in important for evaluating software.\
This section outlines the creation of both functional and non-functional test cases, designed to assess the system's ability to perform the specified requirements.

### Functional Test Cases
<!--
1. Test Case(s)
	1. Steps
* Expected Results
-->

#### Establish connection with Robot Arm
1. Check if the connection between the PC and robot arm.\
Steps:
	1. Turn on robot arm.
	2. Ping the robot to ensure that it is on the same network and able to connect.
	3. Run initialisation commands from PC to set up the robot arm.
* The robot arm is able to receive instructions from the PC.

#### Set up depth camera
1. Check the connection between the PC and the depth camera.\
Steps:
	1. Turn on the depth camera and connect it to the PC.
	2. Start camera software to ensure that PC is receiving a signal from the camera.
* The PC is able to receive image data from the depth camera.

#### Intialise Machine Learning (ML) Model
1. Check if the ML model can receive data from camera.\
Preconditions:
Connection between the PC and the depth camera has been established.\
Steps:
	1. Intialise ML model and ensure that it is receiving data from the depth camera.
	2. Verify that the model is able to detect objects in the image data as required.
* The ML model is able to recognise the required objects and provide locations for them.

#### Connect ML model and Robot arm through PC
1. Check that the ML model is able to send locations to the robot arm.\
Preconditions:
The ML model is able to detect objects in the image data from the depth camera and the robot arm is able to receive location data from the PC.\
Steps:
	1. Run commands to set up connection between the ML model and the robot arm controller on the PC
* The robot arm is able to receive locations of parts to pick up from the ML model.

#### The robot arm is able to moves on command
1. Check that the robot arm is able to move to the locations provided by the ML model and pick up parts as instructed.\
Preconditions:
The connection between the ML model and the robot arm has been made.\
Steps:
	1. Instruct the system to pick up the next required part.
* The robot arm is able to move to the correct location to pick up the part, move the to the unloading location and place the part in the assembly tray.

#### The system is able to continue functioning autonomously
1. Verify that the robot is able to continue operation continuously.\
Preconditions:
The robot arm arm is able to move according to locations provided by the ML model.
* The ML model is able to continue detecting the next required parts and providing the location data to the robot arm.
The robot arm is then able to continue correctly picking up the part and placing them in the assembly tray.
Once the assembly tray is filled up with the required parts, the system is able to instruct the robot arm to move it to the ready position.
When all parts have been taken by the participant, the system is able to instruct the robot arm to return the tray to the original location and repeat the process.

### Non-functional Testing
#### Testing Goal
The goal of the above mentioned test case is the ensure that the system that has been developed is able to function correctly, repeatedly and consistently,
thereby allowing the participant to speed up their rate of production.

#### Testing Procedure
The test will be conducted in the presence of a participant. As the system completes its tasks and delivers the assembly tray of parts to the ready position,
the participant will be requested to provide feedback as to whether or not the task was completed by the system in a satisfactory manner.

#### Pass/Fail Criteria

The task of delivering and assembly tray of parts to the user must be completed in a smooth manner,
ensuring that all the correct parts remain on the tray while in motion, and none of the parts are damaged during the pick and place task.

#### Testing Constants
The locations of the parts are constant. The system will not need to search for a part, rather just check if a part is in a certain location.

## Untested Features
There are currently no features that won't be tested.\
This is because all features are being introduced for the first time.\
All introduced features can be tested and therefore will be tested.

# Strategy
To ensure the successful implementation of the Robot Vision System for the Pick and Place Task, a comprehensive testing strategy will be employed.
This strategy encompasses a structured approach to testing that spans different phases of development.
* Unit Testing\
Unit tests will be conducted to validate the individual components of the vision system.\
This includes testing algorithms related to object detection, position estimation, and robot control.
* Integration Testing\
Integration tests will focus on verifying the interaction between different modules of the vision system.\
This will include testing the integration of computer vision algorithms with the Robot Operating System (ROS2).
* System Testing\
System tests will evaluate the overall performance of the Robot Vision System for the Pick and Place Task.\
The vision system will be tested with various object positions to ensure accurate pick and place operations.
* Acceptance Testing\
Acceptance tests will be conducted in collaboration with the client to ensure that the vision system meets the specified requirements and
fulfils the intended pick and place tasks.

## Roles and Responsibilities
* Team Leader and Client Liaison\
Coordinate and oversee the testing process.\
Communicate testing progress and results to the client.
* Development Manager and Planning Manager\
Define testing milestones and timelines.\
Allocate resources for testing activities.
* Support Manager and Developer\
Assist in preparing the testing environment.\
Execute unit tests and provide feedback on functionality.
* Quality Manager and Developer\
Conduct thorough testing of the vision system.\
Perform integration testing to ensure seamless functionality.\
Identify and document defects and issues.
* Unit Testing\
Developers will create and execute unit tests for individual vision components.\
Test cases will cover object detection accuracy, position estimation, and algorithmic correctness.
* Integration Testing\
Integration tests will be carried out by the quality team.\
Ensure seamless communication between computer vision algorithms and ROS2.
* System Testing\
The entire vision system will be subjected to rigorous system tests.\
Various object positions will be tested to validate pick and place accuracy.
* Acceptance Testing\
Collaborate with the client to conduct acceptance tests.\
Ensure that the vision system meets client requirements and expectations.

## Test Deliverables
The following key deliverables will be produced as part of the testing process:
* Test Plan
* Unit Test Cases and Results
* Integration Test Report
* System Test Scenarios and Outcomes
* Acceptance Test Documentation

## Schedule
Testing will be conducted and completed throughout semester 2.\
The schedule for testing will be as follows:
* Test cobot's movement/test accuracy of vision system
* Test integration of the movement and vision systems
* Test the overall system's performance in completing the pick and place task
* Fine tune system to perform its tasks within an acceptable threshold

## Risk and Contingency
The risks and contingencies associated with testing include the following:

<!-- TOC ignore:true -->
### Risks
1. Team members leave or become unable to work through illness or injury.
2. Time pressures may result in less time being allocated to testing.
3. Not all possible scenarios are able to be tested.
4. The scope of testing could end up going beyond the project's boundaries, resulting in time and resource overruns.
5. Potential danger and injuries could occur working in close proximity of the robot arm.

<!-- TOC ignore:true -->
### Contingencies
1. Have tasks delegated to at least 2 team members.
2. Set realistic time frames for testing activities during project planning and
if timelines become constrained, prioritise testing activities based on problem severity.
3. Create comprehensive test plans that use techniques like equivalence partitioning.
4. Clearly define the scope of testing in the test plan and establish a process to assess and approve any changes to the scope.
5. Ensure clear call outs are made before making the robot move to allow team members time to clear themselves from the danger zone.

## Testing Tasks
* Develop comprehensive unit test cases for individual components of the vision system,
including object detection algorithms, position estimation, and robot control mechanisms.
* Execute unit tests to verify the correctness of algorithms and identify any deviations from expected behaviour.
* Conduct integration tests to ensure seamless communication between computer vision modules and the Robot Operating System (ROS2).
* Validate data exchange, synchronisation, and compatibility among integrated components.
* Work closely with the client to conduct acceptance tests, verifying that the vision system meets the specified requirements and objectives.
* Obtain client approval on the system's performance and functionality.
* Review and validate the accuracy and comprehensiveness of documentation, ensuring that it effectively guides users and developers.

# Pass/Fail Criteria
These criteria help make sure that the project meets the necessary requirements as well as ensuring the project is at an acceptable standard of quality.
The following criteria provide clear guidelines for assessing whether the system meets the desired level of functionality, performance and
other relevant attributes whilst under testing conditions.

## Product Level
The testing will occur on the functionality of the product.

Test Case Pass/Fail Criteria:
* Arm retrieves item
	* Pass Criteria
		* An appropriate item is grabbed
	* Fail criteria
		* The arm grabs nothing
		* The arm grabs an inappropriate item
		* The Arm does nothing
* Arm places item
	* Pass Criteria
		* A grabbed item is placed
	* Fail criteria
		* The arm places the item incorrectly
		* The arm drops an item
		* The Arm does nothing
* Computer vision identifies items
	* Pass Criteria
		* CV consistently labels items correct
	* Fail Criteria
		* CV incorrectly labels items
		* CV labels background as something
		* CV doesn't label anything
* Camera outputs a feed
	* Pass Criteria
		* Camera outputs correctly
	* Fail Criteria
		* Camera feed is corrupted
		* Camera feed is in a different format than expected
		* Camera feed isn't outputting

## Testing Stages
During each development testing stage, success will be judged in the passing of the below criteria.

Testing level pass/fail criteria:
* Unit Testing
	* Pass criteria
		* All unit tests successfully pass.
		* Majority of testable code has an appropriate unit test.
		* Any bugs or errors are recorded and where plausible resolved.
	* Fail criteria
		* A unit test fails.
		* A non-negligible amount of code hasn't been tested.
* Integration Testing
	* Pass criteria
		* Developed modules have appropriate testing done on them.
		* Modules perform their expected function, even when run alongside other modules.
		* Any issues are recorded and where applicable; resolved.
	* Fail criteria
		* Modules fail to work when alone or alongside other modules
		* A large amount of critical errors
* System Testing
	* Pass criteria
		* Entire System has been tested.
		* All specified requirements have been met.
		* Minor Issues found have been logged and resolved accordingly.
		* System features are working appropriately.
	* Fail criteria
		* A specified requirement isn't met.
		* Unresolved issue(s) or a defect is found.
		* System features are not working appropriately.
* Acceptance Testing
	* Pass criteria
		* Client is satisfied with the presented product.
	* Fail criteria
		* Requirements not met.
		* Major issues identified by client.
		* Client is not satisfied.

## Criteria Requirements
Below are Criteria that will halt progress and the subsequent fix for those halts.

### Suspension Criteria
The testing process will be halted if at least one of the below criterias is met:
* The requirements have been changed.
* A critical error affecting multiple test cases has been discovered.
* The system design is found to be inappropriate.
* The system design or implementation undergoes a significant change.
* Involved hardware or testing resources become defective or unavailable.

### Resumption Requirements
* The design has been adjusted or redesign to suit the needs of the project.
* The error has been fixed, or a work-around has been identified.
* A new system design is finalised and ready for testing.
* Any problematic hardware has been replaced with an adequate alternative.

## Approvals
* The Team leader and the Testing manager must be in agreement to the completion of a testing level before proceeding to the next level.
* Changes or additions to the system's features would have to be approved by the client.
