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
	* [Features not to be Tested](#features-not-to-be-tested)
* [Strategy](#strategy)
	* [Roles and Responsibilities](#roles-and-responsibilities)
	* [Test Deliverables](#test-deliverables)
	* [Schedule](#schedule)
	* [Risk and Contingency](#risk-and-contingency)
	* [Testing Tasks](#testing-tasks)
* [Pass/Fail Criteria](#passfail-criteria)
	* [Product Level](#product-level)
	* [Testing Stages](#testing-stages)
	* [Suspension criteria and resumption requirements](#suspension-criteria-and-resumption-requirements)
		* [Suspension Criteria](#suspension-criteria)
		* [Resumption Requirements](#resumption-requirements)
	* [Approvals](#approvals)

<!-- /TOC -->

# Introduction
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
* Testing the integration of the vision system will the movement system will need to be done at the F.o.F.

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
As the robot arm already posseses the ability to precisely pick and move items to the human operator,
the testing strategy centers around assessing its proficiency in selecting items that are currently available,
as opposed to the pre-existing trial-and-error approach across the entirety of the trays.

## Tested Features
The following features will undergo testing:
* The system establishes a stable connection between the camera and robot arm controller software.
* The system inititates an image capture and inference request to the model before the robot arm resets to its initial position.
* The system identifies the presence of chips on both trays before determining which to pick up.
* The system identifies the presence of shells on from their stack before determining which to pick up.
* The system identifies the presence of batteries on from their stack before determining which to pick up.
* The system stops when items are not correctly positioned (e.g., fallen over, being outside of their allocated tray/stack etc.).
* The annotated images resulting from the inference are recorded for subsequent review.

## Test Cases
The creation of well-structured test cases in important for evaluating software.\
This section outlines the creation of both functional and non-functional test cases, designed to assess the system's ability to perform the specified requirements.

### Functional Test Cases
<!--
1. Test Case(s)
	1. Steps
* Expected Results
-->

#### Establish Communication
1. Check if connection is established between devices
	1. User turns on the device
	2. System automatically tries to establish a connection with the other device
* Connection is established between the two devices when the device is switched on

#### Send video feed
1. Check if instructor can send video feed\
Preconditions:
Connection between users has to be established
	1. Instructor clicks Send video
* Video stream connection is successfully setup and video feed would stream between both users

#### View video feed
1. Check if user can view operator's video feed\
Preconditions:
Connection between users has to be established
	1. System automatically broadcasts video stream to both users.
* Both users can view the operator's video feed on their displays

#### Send hand gestures
1. Check if Instructor can send hand gestures to Operator\
Preconditions:
The system has to have the video feed of the operator
	1. System automatically send instructor's hand gestures when application starts up
* The system should start obtaining the video feed of the operator and the video feed of the instructor.\
The video feed of the instructor is then processed to obtain footage only of the instructor's hand and not the background surroundings.
The image of the hands should then be superimposed on the operator's video feed and then sent back to the operator.
Both users should see video feed of both the operator's feed and the instructor's hand gestures to both users

#### Make sketch
1. Check if instructor can capture image\
Preconditions:
Instructor must have video feed to capture image from
	1. Instructor performs "screenshot" hand signal
* System captures image of video stream
2. Check if instructor can draw sketch on image
	1. Draw sketch on image by using their fingers
* System registers finger movements and overlays it onto image

#### Send sketch
1. Check if instructor can send sketched image to operator\
Preconditions:
Instructor has created sketch using "Make a sketch" function
	1. Instructor performs "wave forward" hand signal
* System sends image to the operator

#### View sketch
1. Check if operator can view image\
Preconditions:
Operator device must receive image
	1. System automatically replaces operator's video feed with sketched image
* Operator can view sketched image

#### Dismiss sketch
1. Check if operator can dismiss image after viewing\
Preconditions:
Operator device must receive image
	1. Operator performs "wave-away" hand signal
* System dismisses image, and view goes back to operator video feed

### Non-functional Testing
#### Testing Goal
The purpose of the following test cases is to make sure that the application we develop would not only be functional but be user friendly with a small learning curve,
allowing the average user to quickly adapt to the user interface.

#### Testing Procedure
The test would be performed on a set of participants, who will be asked to perform basic tasks using the applications.
Upon completing the task, the user would be expected to be able to give feedback on the difficulty of the task and
what their suggestion to improve the user interface would be.

#### Pass/Fail Criteria
The tasks given are very simple and would have constant pass criteria.
Each task should be completed before 30 seconds and would have a difficulty goal of being less than 5(1 being very easy, and 10 being very hard)

#### Testing Constants
All of the tests will be carried out on the application on the Smart Glasses, using sample data provided to them.

#### Testing Assumptions
The tests will be carried out in pairs of participants where on of the participants will play the role of the "operator" while the other acts as the "instructor".

Non-functional test cases:

Test Case(s)
1. Check if user can take a screenshot
2. Check if user can make a sketch
3. Check if user can send sketch
4. Check if user can dismiss the sketch
5. Check if user can send video

Steps
1. Perform "screenshot" action
2. User users finger to sketch on image
3. User performs "wave-forward" action
4. User performs "wave-away" action
5. User clicks send" video feed"

Expected Results
1. User must see screenshot on display
2. User must see image with draw path of his/her finger
3. Operator must acknowledge that a screenshot was received
4. User must see the sketch being removed from the operator's display
5. User must see video feed

## Features not to be Tested
Since all the features are being implemented for the first time and are all capable of being tested.
There are currently no features that will not be tested.

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
Risks associated with testing:

#### Risks
1. May be shortage of time which may result in less time for testing.
2. Requirements may change during development.
3. Team member leaves
4. Design lack flexibility to make changes

#### Contingency
1. Carry out tests frequently as the system is developed instead of assigning a particular date.
2. Keep updating requirements from client as frequent as possible to avoid last minute chaos.
3. Have each module worked on and tested by 2 members.
4. When designing allow room for accommodating changes.

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
## Product Level
Instructor and operator can switch roles upon accepting the connected user's request.

Test Case Pass/Fail Criteria:
* Pairing the glasses over a wireless connection/ Bluetooth connection
	* Pass Criteria
		* Glasses are paired up with a stable wireless connection with a third device acting as a server.
	* Fail Criteria
		* Glasses don't pair up successfully or connection isn't stable enough.
* Instructor can see hand gestures on operator's video feed.
	* Pass Criteria
		* Request sent from operator glasses and accepted by instructor glasses.
	* Fail Criteria
		* Request isn't sent or doesn't prompt instructor with video request.
* Receiving and viewing operator video feed from operator glasses on instructor glasses
	* Pass Criteria
		* Video is sent with a delay of less than 1000ms and a minimum frame rate of 20.
	* Fail Criteria
		* Video sent, has loss of frames.
* Sending hand gestures from instructor glasses
	* Pass Criteria
		* Instructor can see hand gestures on operator's video feed.
	* Fail Criteria
		* Instructor's video feed does not show hand gestures.
* Receiving only hand gestures (without video) from instructor glasses and viewing them on operator video feed.
	* Pass Criteria
		* Operator can see instructor's hand gestures on operator video feed.
	* Fail Criteria
		* Instructor's hand gestures are not shown at all/ or not shown clearly on operator's video stream.
* Make sketch
	* Pass Criteria
		* Instructor can capture a still image of video feed and draw a sketch on it using his/her finger.
	* Fail Criteria
		* Instructor can capture a still image of video feed and draw a sketch on it using his/her finger.
* Send sketch
	* Pass Criteria
		* Instructor is able to send a saved image of a sketch to the instructor.
	* Fail Criteria
		* Unable to save sketched image/ send it to instructor
* View sketch
	* Pass Criteria
		* Operator can download and view sketched image.
	* Fail Criteria
		* Operator is unable to view image.
* Dismiss sketch
	* Pass Criteria
		* Operator is able to dismiss image after viewing
	* Fail Criteria
		* Operator is unable to dismiss image after viewing

## Testing Stages
During each development stage test will be conducted and judged according to the following criteria.

Testing level pass/fail criteria:
* Unit Testing
	* Pass criteria
		* All unit tests have been passed successfully
		* At least 70% of all code written has been covered during testing.
		* All bugs and errors found have been logged and been accounted for.
	* Fail criteria
		* Not all unit tests have been passed
		* Not enough code written has been included in testing.
* Integration Testing
	* Pass criteria
		* 90% of all modules developed have been tested.
		* Modules perform their assigned function successfully when tested together.
		* All issues have been logged and corrected
	* Fail criteria
		* Modules fail to carry out their function when put together.
		* Too many critical issues found.
* System Testing
	* Pass criteria
		* Entire System has been tested.
		* 100% of all specified requirements have been successfully achieved.
		* Minor Issues found have been logged and fixed.
		* 100% of all system features functioning appropriately.
	* Fail criteria
		* Not all specified requirements have been achieved.
		* Critical issues and defects found during the test.
		* System features are not functioning accurately.
* Acceptance Testing
	* Pass criteria
		* When client is satisfied with the product.
	* Fail criteria
		* Does not achieve the requirements specified.
		* Critical issues found by client.
		* Does not satisfy the client.

## Suspension criteria and resumption requirements
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
* Both the Team leader and the Testing manager must agree to the completion of a testing level before moving on to the next level.
* Any changes or addition to the system features would have to be approved by the client.
