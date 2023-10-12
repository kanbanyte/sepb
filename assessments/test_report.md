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
