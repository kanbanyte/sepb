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
### Non-functional Testing
## Testing Tasks
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
# Outcome
