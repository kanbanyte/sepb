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
As outlined in the Test Plan document, the features to be tested are categorised into vision-related, cobot-related, and integration of the two.
This section outlines detailed features to be tested for all three categories.

For the vision system, the following items are tested on the camera and AI models:
* The chip detection model is tested on chips in all 48 possible slots with chips leaning in both directions.
Note that the red slots only hold blue chips whereas the white slots only hold the yellow chips.
* The case detection model is tested on all possible 17 possible positions on the case rack.
* The tray detection model is tested on all possible variations of a white tray, in all 3 positions.
The trays are divided into 3 classes: Full, Empty, and Partially Full, of which there are many possible combinations.
* The camera connects to the computer and successfully applies the settings specified in a configuration YAML file.
* Crop boxes specific to the chip model, tray model, and case model are applied to the raw image captured by the camera.

For the cobot, movements accuracy and speed are tested on all possible positions of items to be picked and moved.
These include:
* Movement to pick up chips from 48 possible slots and place them on the lower right compartment of the tray.
* Movement to pick up cases from 17 possible positions and place them on the lower left compartment of the tray.
* Movement to pick up PCB shells delivered to a specific location by the conveyor belt to the upper left compartment of the tray.
* Movement to pick up batteries from the battery rack and place them on the upper left compartment of the tray.
* Movement to pick up trays and deliver them to and from the human operator, including:
	* Movement to pick up a fully filled tray on the upper left to the human operator.
	* Movement to pick up a fully filled tray on the lower left to the human operator.
	* Movement to pick up an empty tray from the human operator back to a empty spot to be filled up again.

For integration-related features, the following items are tested:
* Bounding boxes from chip detection model correctly converted to chip positions defined by the cobot.
* Bounding boxes from case detection model correctly converted to case positions defined by the cobot.
* Bounding boxes from tray detection model correctly converted to best tray movement for the cobot.
* Signal containing item position or tray movement communicated to the cobot motion controller, which acts upon that information.

### Functional Test Cases
### Non-functional Testing
Following the requirements that were listed in the Test Plan,
the system was able to repeatedly complete its tasks correctly and consistently while carrying out its movements in a smooth manner.
This was especially important when moving the assembly tray filled with parts, ensuring that nothing was damaged in the process.

The testing was also carried out with the participant paying particular attention to the procedures and activities the system carried out.
As mentioned in the participants feedback, they were notably happy with the results that were produced.

As planned, the location of all the objects were kept constant, such as the locations of the chips, cases, and trays.
This ensured that the development of the system was targeted more towards the development of the movement and detection algorithms.

Overall, considering the goals and aims that the tests targeted, the results were successfully achieved.

## Testing Tasks
In the testing phase, HeeHooVision diligently tackled several essential tasks.
To begin with, they developed comprehensive unit test cases for the various components of the vision system,
encompassing object detection algorithms, position estimation, and robot control mechanisms.
The results of these unit tests consistently fell within the predefined parameters, with no observed deviations from the expected behaviour.

Following this, integration tests were conducted to ensure the seamless communication between the computer vision modules and the ROS2 framework.
These tests convincingly verified the effective interaction of the vision system components with ROS2,
confirming the smooth exchange of data, synchronization, and compatibility among the integrated modules.

Additionally, HeeHooVision worked closely with the client to execute acceptance tests.
During these assessments, they meticulously verified that the vision system met all specified requirements and objectives.
The client actively participated in these evaluations and expressed approval of the system's performance and functionality,
signifying alignment with their expectations.

Finally, HeeHooVision thoroughly reviewed and validated the accuracy and comprehensiveness of the documentation.
The primary objective was to ensure that it serves as a valuable guide for both end-users and developers.
It can be confidently stated that the documentation, after this validation, provides clear and accurate guidance, making it an invaluable resource for the future.

In summary, this testing phase was successful in verifying the functionality and performance of the vision system,
meeting client expectations and establishing robust documentation for reference.

# Pass/Fail Criteria
## Product Level
As discussed in the test plan, a long list of appropriate and relevant pass/fail criteria for the product level were established.
With the conclusion of the testing, it is important to look over these established criteria to ensure that the system as defined by the criteria passes.\
The initial criteria can be split into two sections, the arm/gripper criteria and the CV component.
We will cover the arm/gripper first.

<!-- TOC ignore:true -->
### Arm Retrieves Item
The testing of the system has concluded that the arm (and by extension the gripper) can successfully retrieve items.\
The auto stop feature of the gripper made this process simpler as well.
The robot having programmatically controlled movements, meant that the whole system worked beautifully once correctly established; a definite pass.

<!-- TOC ignore:true -->
### Arm Places Item
Similar to the prior criteria, this was also satisfactory.
Utilising similar technology, the arm and gripper were able to work in tandem to place items in the correct spots.

<!-- TOC ignore:true -->
### Computer Vision Identifies Items
Through thorough testing and training,
the model came out the other side robust and well trained as both effectively performing analysis and reporting correct evaluations majority of the time.\
In terms of the fail criteria, we consistently only had the third criteria fail (when CV doesn't label anything).
This occurs when there is serious disturbance to the image; such as the presence of a human head and/or body.\
The model contained instance of human arms within the training set; therefore, it performs just fine with minor interferences.\
Overall, the CV passed in all requested use cases.

<!-- TOC ignore:true -->
### Camera Outputs a Feed
With the way the model was programmed, it was established early to allow this functionality to be provided; as such, this was done early on and handled well.
As a result, this system is very solidly passing.

## Testing Stages
Referring to our test plan, relevant pass/fail criteria were established.
As Testing has concluded we will review the criteria again and compare them to the tested system.

<!-- TOC ignore:true -->
### Unit Testing
This testing stage was primarily applied to the AI code; or more to the point, the intermediary code of turning AI outputs into a ROS usable format.
So, going off the established requirements: the testing was sufficient, covering a good amount of code.\
Alongside this, a few bugs were identified and promptly fixed; overall this testing stage passed.

<!-- TOC ignore:true -->
### Integration Testing
In the development, much of the code was made into a library style; thus making integration easier and more consistent.
The integration is also made to ensure the AI and ROS code can interact and "talk" to each other effectively.\
As the criteria states, this section was well integrated and made solidly; therefore this would be a pass.

<!-- TOC ignore:true -->
### System testing
This section was of testing the system as a whole, this referring to the arm making and executing appropriate movements.
This stage was the longest as ensuring that the predicted outcomes were in fact working took a good degree of time;
as mentioned in the prior section [Product Level](#product-level), the system was effective and worked as intended.\
Judging off the established criteria, we get a confident pass.

<!-- TOC ignore:true -->
### Acceptance testing
A simple stage, asking the client if they approve.
Currently this section is a pass, they are happy with progress and the integration; with only minor changes requested.
Those particular changes being cosmetic/naming convention related rather than major issues with code or choices made around the project.

# Outcome
