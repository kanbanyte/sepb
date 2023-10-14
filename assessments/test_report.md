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
## Testing Tasks
# Pass/Fail Criteria
## Product Level
## Testing Stages
# Outcome
