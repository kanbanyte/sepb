<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s):
	Year: 2023
-->

*[Note:*
*This is a sample/template document for the Usability Evaluation Plan.*\
*Please improve, adapt and adjust to your project needs.*\
*In particular, the project-specific questionnaires need to be included.]*

`Usability Evaluation Plan`

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
* [Methodology](#methodology)
* [Participants](#participants)
* [Training](#training)
* [Procedure](#procedure)
* [Roles](#roles)
* [Usability Tasks](#usability-tasks)
* [Usability Metrics](#usability-metrics)
	* [Scenario Completion](#scenario-completion)
	* [Errors](#errors)
	* [Subjective Evaluations](#subjective-evaluations)
	* [Time on Task](#time-on-task)
* [Usability Goals](#usability-goals)
* [Problem Severity](#problem-severity)
	* [Impact ranking](#impact-ranking)
	* [Frequency ranking](#frequency-ranking)
	* [Severity Ranking](#severity-ranking)
* [Reporting Results](#reporting-results)

<!-- /TOC -->

# Executive Summary
The document specifies the methodology by which usability is evaluated and explains how the results are interpreted.
The evaluation process involves the usage of a prototype and a participant in a controlled environment that closely emulates practical setting.
The evaluation methodology encompasses both quantitative and qualitative measures,
incorporating user feedback and performance metrics to identify areas of improvement from the prototype.
The collected data is analysed and serves as guide to further develop the prototype into a complete and satisfactory end-product.

# Methodology
This section outlines various factors that contribute to the usability evaluation process, all of which are tailored to the specific characteristics of the system.

<!-- TOC ignore:true -->
### Number of Participants
As the project aims to improve the robot arm that aids a human operator in assembling chips and
all the pick-and-place tasks are executed by the robot arm, only a single participant is required.

<!-- TOC ignore:true -->
### Environment
The experiment will be conducted in the Factory of the Future at Swinburne University, at the area where the robot arm and assembly chain is located.
The working space, which includes the robotic arm, the item trays and and the human operator will have full lighting.

<!-- TOC ignore:true -->
### Tools
A new depth camera is mounted on top of the working space, aiming down to the item tray to guide the movement of the robot arm.
Other tools include the items to be assembled, a conveyor belt that delivers a shell, and the robot arm, all of which already exist.

<!-- TOC ignore:true -->
### Measures
The metrics used involve precision rate, recall rate (also known as True Positive Rate) and time-to-complete a pick-and-place cycle.
Precision measures how conservative the system is when identifying the presence of an item.
Recall measures how sensitive the system is when identifying items that are actually present.
Recall is used a metric to gauge false-negative cases (skipping a slot that actually holds the item to be moved),
which whether the aid of objection detection technology is fully utilised.
Finally, the time consumed to complete a pick-and-place cycle, with a vision system incorporated, remains similar, if not superior, to the pre-existing system.

<!-- TOC ignore:true -->
### Satisfaction Assessment
After the experiment, the participant would be asked to fill a form detailing their experience with the system.
In particular, they will be prompted to comment on the safety and speed and efficiency in the comparison with the original system.

# Participants
The end user of the Vuzix Smart Glass would be an employee or trainee working with an instructor in a training program.
Apart from basic experience with technology, the participant would not be expected to have any additional technology skills.

Participants have the responsibility to complete tasks given to them while giving feedback by talking about their thought process while doing the task.
Participants are also asked to complete a demographic questionnaire pre-evaluation, and satisfaction questionnaire post evaluation.
The experiment would be a simulation of a real-life training scenario, therefore the participants would be students imitating the role of the operator and instructor.
Participant recruitment would be done through social media.

# Training
The participant will receive an introduction to the objectives of the pick-and-place task and their assigned role.\
Additionally, they will be instructed on assembling the device using the provided parts and guided on maintaining a safe distance from the cobot during its operation.

# Procedure
The evaluation test will take place at the Factory of the Future.\
The environment contains the cobot, various components for assembling, and the ZED2 camera for detecting the components.\
Prior to the evaluation, the participant will be trained to assemble the components, and the function of the cobot will be explained.

During the test, the participant will be observed and any difficulties they have will be noted.\
Additionally, any inaccuracies in the cobot's function will be noted to rectify in the future.

Following the evaluation, the participant will be questioned on any specific issues they experienced, and potentially what they think could be improved for easier use.

# Roles
During the evaluation, team members will be given roles to allow for better observation recording, training of the participant, and
assisting the participant should they require anything during the evaluation.

<!-- TOC ignore:true -->
### Observer
The observer will record the cobot's and participant's performance during the evaluation.
They will note any troubles the participant has in assembling the device, and any inaccuracies the cobot has in picking and placing any objects.

<!-- TOC ignore:true -->
### Assistant
The assistant will answer any questions the participant has during the test in order to help them.
Additionally, they should be able to rectify any problems the cobot has in performing its tasks, such as restarting the system should an error arise.

<!-- TOC ignore:true -->
### Trainer
The trainer will explain the evaluation test to the participant and answer any pre-emptive questions so that the participant understands what they need to do.

# Usability Tasks
As the goal of this project is the automation of a pick and place task, the resultant system will require limited human interaction.\
As such, what will be evaluated will be limited to the tasks that system completes.

These tasks include:
* Picking up the correct objects and placing them in the correct position on the assembly tray.
* Picking up and placing the assembly tray in the ready position when all parts and accumulated and placed in the assembly tray.
* Identifying when the assembly tray is empty and returning it to its original position.

# Usability Metrics
Usability requirements can be evaluated using usability metrics like scenario completion rates, error rates, subject evaluations and
time of completion will be used to calculate usability performance on tasks.

## Scenario Completion
A scenario is completed when the user has gotten the expected output or when the user requests assistance and cannot complete the task due to usability issues.

## Errors
There are two types of errors being critical and non-critical errors.
Critical errors are those of which prevent the user from getting the expected output and
non-critical errors are errors which the user can recover from but could potentially give an unexpected outcome.

## Subjective Evaluations
Subjective evaluations form a vital facet of the usability assessment for the Robot Vision System's Pick and Place Task.\
Analysing feedback trends will guide iterative improvements, seamlessly integrating user perspectives into the development process.
This holistic approach drives continual enhancements to the system's usability.

## Time on Task
Measuring time on task is pivotal in assessing the usability of the Robot Vision System for the Pick and Place Task.\
By observing participants as they complete representative task scenarios, valuable insights into user efficiency and system adaptation will be gained.
Analysis of the collected time data will inform system enhancements, aligning with user proficiency and satisfaction.
This data-driven approach ensures continual refinement and optimisation of the system's performance.

# Usability Goals
The usability goals for this usability evaluation would be a completion rate of 100% and an error-free rate of 80%.
The completion rate is the percentage of participants who successfully complete the task without critical errors.
The error-free rate is the percentage of participants who complete the task without both critical and non-critical errors.

# Problem Severity
Problems that are seen during the testing must be classified under severity which is dependent on impact and frequency.

## Impact ranking
#### High
A critical error which prevents the user from completing the task.

#### Moderate
A non-critical error which causes the user to have a larger time on task despite the task being completed.

#### Low
A user encounters a minor non-critical error.

## Frequency ranking
#### High
Greater than 30% of the participants encountered the issue

#### Moderate
13%-29% of the participants encountered the issue

#### Low
Less than 13% of the participants encountered the issue

## Severity Ranking
#### Severity 1
An issue with high severity and high impact preventing the user from completing the task.
This kind of issue must be fixed as quickly as possible.

#### Severity 2
An issue with moderate to high frequency with moderate impact which doesn't allow the user to complete the task efficiently.

#### Severity 3
An issue with either moderate frequency with low impact or low frequency with moderate impact that might frustrate some users.

#### Severity 4
An issue with low frequency and low impact that might increase user satisfaction if the problem is to be resolved.

# Reporting Results
The usability evaluation test will be concluded with a usability test report presenting the results with the usability metrics compared with the usability goals and
recommendations resolve the problems.
