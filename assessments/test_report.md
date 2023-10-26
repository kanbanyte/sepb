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
	* [Testing Tasks](#testing-tasks)
	* [Test Items](#test-items)
	* [Test Cases](#test-cases)
		* [Tested Features](#tested-features)
		* [Functional Test Cases](#functional-test-cases)
		* [Non-functional Testing](#non-functional-testing)
* [Pass/Fail Criteria](#passfail-criteria)
	* [Product Level](#product-level)
	* [Testing Stages](#testing-stages)
* [Outcome](#outcome)

<!-- /TOC -->

<div class="page"/><!-- page break -->

# Introduction
This document is made to report on and cover the results of the tests outlined in the Test Plan, while discussing and evaluating their success.
Throughout testing, many of the proposed and planned tests were removed;
these stages primarily being seen as either too difficult to test with our provided resources, not worth testing or just generally didn't fit the finalised product.
Below will cover the items being tested, the appropriate test cases for them as well as the pass/fail criteria of said tests.\
It is also going to justify the test choices and the validity of them in relation to the system.

The tests were primarily conducted within the FOF, on integration between AI and ROS systems.
Other tests were done in unit test format run from local machines and then recorded as needed.

# Results
The assessment centred on the AI-based vision system and cobot movement controller, testing various features outlined in the test cases.
Vision-related capabilities like chip and case detection along with camera functionality were closely examined.
The system's proficiency in cobot-related functions including item handling was a key focus.

Functional tests confirmed the system's robust performance ensuring precise robotic movements and item handling, maintaining autonomous functionality.
Non-functional testing affirmed consistent task execution, client satisfaction, and object positioning stability.

The validation process including unit and integration testing, client acceptance tests, and
documentation review; ensured functionality, documentation clarity plus accuracy for the intended application, and met client expectations.

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

## Test Items
The cobot already possesses the ability to pick up objects from pre-defined positions and
place them on a tray to be transferred to a human operator via a PLC controller.
The project's objective is to equip the cobot with an AI-based vision system that precisely guides the selection of objects,
and replicate the existing movements via a new software implemented with ROS, collectively known as the movement controller.
As a result, the focus of the tests will revolve around the new functionalities.

The tested items span multiple components of the system:
* The camera which captures images of the workspace, including:
	* Mounting the camera on a T-slot bar of the working space using 3D-printed components.
	* Apply camera settings (saturation levels, flip mode, brightness, etc.) to provide images that can be optimally processed by the AI models.
	* Establish connection to the camera from the computer controlling the cobot.
* AI models used to detect the presence of various types of objects for pickup, including:
	* Model to detect positions of chips.
	* Model to detect positions of cases.
	* Model to detect position of trays and classify them into 3 categories: Full, Empty, and Partially Full.
* Cobot movement controller, which includes the following components:
	* Action server to control the cobot by sending trajectories to the joints and gripper.
	* Action client to send movement requests to the server and keep the cobot operating in a loop.
* Integration between the AI models and movement controllers via a camera server.
	* Camera server is a ROS service supplying positions of items to the movement controller.
	The server relies on the AI models and the camera to retrieve bounding boxes of objects and
	convert them to positions or commands to be used by the movement controller.

## Test Cases
This section will cover the features that were tested as well as the functional test cases that were used to evaluate the features.
It is important that the success or failure of the test cases are properly described in order to evaluate the success of the project.

The features that were tested were chosen to be the most likely cases faced by real world use of the project.
Programming for every edge case was not the priority of this project so ensuring that software and
hardware worked in the intended way was deemed the most important for testing.

The test cases chosen were selected to test the entirety of the project as best as possible.
With many different parts that all interconnect, it was important to HeeHooVision that the initialisation of each component was tested.
This ensured that further testing would be valid as we could be confident all components worked individually and
any problems would be caused by connection issues rather than a individual component.

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
Based on the features listed in the previous section, and they're specific criteria,
a series of the test cases were developed in the test plan to ensure that all the criteria were met and the final product was of a satisfactory standard.

In this section of the report, each of the test cases and the details of their success/failure will be discussed.

**Establish Cobot Connection**\
The system was able to achieve the expected results, with the Linux PC able send information to the robot arm across the network using ROS2, and
the robot arm able to follow those instructions and move accordingly.

At times, the PC faced connectivity issues with the cobot, these problems stemmed from the PC shifting to another network due to external factors.
Upon identifying this issue, steps were promptly taken to incorporate a network check into the instructions and documentation.
As a result, the occurrence of this problem significantly decreased.

Therefore, this test can be considered a success.

**Setup Depth Camera**\
Once connect to the PC, the code that was developed by the team and the software that was provided by the camera manufacturers,
were able to connect to the camera and view a feed from it.
The code was also able to take and store pictures of what the camera was seeing.

Therefore, this test can be considered a success.

**Initialise Model**\
The trained AI models was able to apply inference settings (confidence threshold, intersection over union threshold, etc.),
retrieve images from the camera, correctly identify positions of objects and report them back.
The processed images with bounding boxes and labels are displayed to the user for visual verification.

Therefore, this test can be considered a success.

**Model and Cobot Connection via PC**\
Using ROS2, a camera server node is used to deliver information from the AI Models to the cobot action server controlling movements of the robot arm.
Using this information, the action server is able to plan the trajectories required for the robot arm to pick and place the correct items in the correct locations.

Therefore, this test can be considered a success.

**Cobot Movement via Command**\
After receiving the required movement information, the robot arm is able to execute them correctly and smoothly,
with no interference or interaction with the surrounding environment.

Therefore, this test can be considered a success.

**System Continues Autonomous Functionality**\
After completing a single cycle of the procedure, the robot system is able to repeat the process of the analysing images from the camera to find locations of parts,
transmitting the data of the locations, planning the paths required of the robot arm, and making the robot arm conduct the actions.

Therefore, this test can be considered a success.

### Non-functional Testing
Following the requirements that were listed in the Test Plan,
the system was able to repeatedly complete its tasks correctly and consistently while carrying out its movements in a smooth manner.
This was especially important when moving the assembly tray filled with parts, ensuring that nothing was damaged in the process.

The testing was also carried out with the participant paying particular attention to the procedures and activities the system carried out.
As mentioned in the participants feedback, they were notably happy with the results that were produced.

As planned, the location of all the objects were kept constant, such as the locations of the chips, cases, and trays.
This ensured that the development of the system was targeted more towards the development of the movement and detection algorithms.

Overall, considering the goals and aims that the tests targeted, the results were successfully achieved.

# Pass/Fail Criteria
This section outlines the standards that the project must meet to be considered acceptable for deployment,
by looking at the product level along with the testing stages.

The pass/fail criteria used is outlined in the [Test Plan](test_plan.md),
where it is described that the criteria will help make sure that the project meets the necessary requirements;
as well as, ensuring the project is at an acceptable standard of quality.
The predefined criteria provides clear guidelines for assessing whether the system meets the desired level of functionality, performance, and
other relevant attributes whilst under the testing conditions.

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
For the unit tests, we followed the format of having a few Null cases and invalid test cases,
these included exceptionally large numbers, negatives and inverted coordinates.

For cases and chips, all 17 and 48 valid positions were tested.
As for the trays, movement commands were returned in all cases,
defaulting to "no move" when none could be made.

<!-- TOC ignore:true -->
### Integration Testing
In the development, much of the code was made into a library style; thus making integration easier and more consistent.
The integration is also made to ensure the AI and ROS code can interact and "talk" to each other effectively.\
As the criteria states, this section was well integrated and made solidly; therefore this would be a pass.
Some proof of this was the successful results of the AI model as seen here:

**Chip Model**
|&emsp;|Chip/Truth|Background/Truth|
|:-|:-:|:-:|
|Chip/Predicted|130|0|
|Background/Predicted|0|0|

**Tray Model**
|&emsp;|Empty/Truth|Full/Truth|Paritial/Truth|Background/Truth|
|:-|:-:|:-:|:-:|:-:|
|Empty/Predicted|11|0|0|0|
|Full/Predicted|0|15|0|0|
|Paritial/Predicted|0|0|20|0|
|Background/Predicted|0|0|0|0|

**Case Model**
|&emsp;|Case/Truth|Background/Truth|
|:-|:-:|:-:|
|Case/Predicted|17|0|
|Background/Predicted|0|0|

<!-- TOC ignore:true -->
### System Testing
This section was of testing the system as a whole, this referring to the arm making and executing appropriate movements.
This stage was the longest as ensuring that the predicted outcomes were in fact working took a good degree of time;
as mentioned in the prior section [Product Level](#product-level), the system was effective and worked as intended.\
Judging off the established criteria, we get a confident pass.

<!-- TOC ignore:true -->
### Acceptance Testing
A simple stage, asking the client if they approve.
Currently this section is a pass, they are happy with progress and the integration; with only minor changes requested.
Those particular changes being cosmetic/naming convention related rather than major issues with code or choices made around the project.

# Outcome
The project evaluation with a focus on critical sections yielded favourable results.\
The system demonstrated effective item handling, despite occasional challenges posed by image disturbances.\
Extensive testing covered vision-related, cobot-related, and integration features; ensuring a comprehensive examination.
Functional test cases produced successful outcomes, and non-functional testing affirmed the system's reliability, positive client feedback, and safety.

Clear pass/fail criteria provided transparent evaluation guidelines.
At the product level, the system excelled in item handling and computer vision.
Across testing stages, encompassing unit, integration, system, and acceptance testing;
the project consistently met predefined criteria, affirming its readiness for deployment.

One of the biggest advantages to the way the project has been structured is its modular design philosophy.
This design allows future users and developers to easily modify, add improvements, or add new features during future development,
without affecting the functions of the rest of the project.
For AI-related components, individual objects have their own models which can be substituted in the future.
Modules in the AI package can also be tested independently of the cobot and the camera if needed.
Similarly, the cobot movement controller is also divided into multiple services and nodes that are highly cohesive and uncoupled with each other.

A significant weakness currently present in the system is the choice of programming language, as the project is currently based in Python.
Although the team initially planned to develop the program in C++, the decision was made to switch to Python during development,
since most online documentation for ROS2 and tools used in machine learning are based on Python.
This decision however hindered progress later on when working on the functionality of the gripper,
due to the fact that there was no documentation on this topic in Python.
A future consideration is to switch the core language of the program to C++, which is a statically type language as opposed to Python which is dynamic.
This will improve the developer's ability to detect errors or issues in the system.

Another future consideration that will need to be taken into account is the camera.
Currently, the images from the camera provide a low amount of detail for the CV algorithms, and
while the results at the moment are consistent and correct, accuracy of the systems can be improved.
A possible solution is to move the camera closer to the workstation, however this would create a risk of the cobot moving into the camera.
The better solution would be to get a camera that is able record a higher resolution image, and thereby provide more detail in the images.
