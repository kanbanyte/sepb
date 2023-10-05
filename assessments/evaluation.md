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
	* [Impact Ranking](#impact-ranking)
	* [Frequency Ranking](#frequency-ranking)
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
The primary user of the system is an operator who undertakes the task of assembling items delivered by a robot.\
The operator does not need to manually assist the robot in any way unless the items have fallen over or are not correctly positioned,
in which cases the robot will halt entirely and await a manual restart.

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
To ensure that the above mentioned tasks are completed to a satisfactory level, a series of usability metrics will be employed.\
This allows the system to be evaluated and any shortcomings be identified and improved during the testing phase.

The following sections will be used to identify and categories metrics, and define how they will be assessed.

## Scenario Completion
The completion of the scenario requires that the system is able to correctly identify:
* The correct chip required, pick it up, and place it in the correct position on the assembly tray.
* The location of the next available battery and case, pick them up and place them in the correct position on the assembly tray.
* All present parts on the assembly tray, pick up the tray and place it in the ready position on the work surface.
* The empty assembly tray once the worker has used all the parts, pick up the tray and return it to its original position on the work surface.

## Errors
In evaluating the usability of the Robot Vision System for the Pick and Place Task, errors will be a focal point of assessment.\
These errors encompass positioning inaccuracies, object recognition failures, communication glitches, and execution deviations.\
Categorised by severity-critical, major, and minor-these errors will be logged in real-time and analysed to glean insights for enhancement.
This approach ensures a robust usability evaluation that informs iterative refinement of the system's performance and reliability.

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
The usability of the system will be measured in the success of the software, that being through successfully selecting appropriate objects for use.
The system will achieve a 100% success rate whereas the success of the software is harder to measure due to the variance that exists in prediction software.

# Problem Severity
Problems that are discovered during the  testing procedures will be classified based on severity, of which are influenced by impact and frequency.

## Impact Ranking
Ranking problems helps prioritize which issues need immediate attention and which ones can be addressed later.

#### High
Critical errors that have a significant negative effect on the user experience.\
High impact errors lead to frustration, task failure, and a substantial reduction in user satisfaction.

#### Moderate
Issues that negatively affect the user's experience noticeably but which may not be as serious as high impact problems.\
They might make users more frustrated and less productive.

#### Low
Non critical errors that have a minimal negative impact on the user's experience.\
They may cause slight confusion or inconvenience for the user, but they don't significantly hinder users from achieving their goal.

## Frequency Ranking
Ranking problem frequency is valuable for understanding whether an issue is an isolated incident or a recurring problem.

#### Frequent Occurrence
Greater than 20-50% of the participants encountered the issue across multiple sessions.\
These problems must be addressed to ensure usability is maintained.

#### Moderate Occurrence
10-20% of the participants encountered the issue.\
These issues still impact user experience and should be addressed to improve usability.

#### Rare Occurrence
Less than 10% of the participants encountered the issue.\
These problems may only affect a small subset of users, but they are still worth addressing to enhance the product's usability.

## Severity Ranking
Categorising usability issues by severity is important as it allows the team to prioritise problem-solving efforts,
enabling the team to focus resources on addressing critical user experience problems first.

#### High Severity
These issues have a severe negative impact on usability and heavily impact a user's ability to achieve their goals.\
These issues often lead to task failure and user frustration.\
High severity issues for this project include:

* Safety Critical Error:
The software occasionally fails to detect an obstacle in the robotic arm's path, leading to a collision risk.\
This is a high-severity problem because it directly threatens safety.

* System Freeze:
The system occasionally freezes during operation, rendering the robotic arm unresponsive.\
 This is a high-severity issue because it halts production and may result in damage to equipment.

* Inaccurate Object Detection:
The neural network consistently misidentifies the position of objects, causing the robotic arm to repeatedly miss its target.\
This high-severity problem impacts the system's efficiency.

#### Medium Severity
These problems have a moderate impact on usability, causing inconvenience reduced speed in user task completion.\
While not critical, these issues still need to be addressed to ensure a smoother user experience.\
Medium severity issues for this project include:

* Inefficient Path Planning:
The software takes longer than expected to calculate the optimal path for the robotic arm, causing minor delays in task completion.\
While this does not affect safety, it impacts efficiency.

* Interface Confusion:
Some operators occasionally struggle to understand a specific feature of the user interface, leading to slower operation.\
This is a moderate-severity issue as it affects usability.

* Intermittent Connectivity Issues:
The system experiences occasional communication problems between the depth camera, neural network, and robot operating software.\
While this can disrupt operation, it's not a constant or critical issue.

#### Low Severity
These issues have minimal impact on usability and do not significantly hinder users from completing their tasks.\
They may however reduce the quality of a user's experience and therefore should be addressed and fixed.

# Reporting Results
Upon conclusion of testing the Test results will be summarised in a clear and concise way with appropriate captions and information in a report.
The Report will cover the successfulness of the test, and where applicable improvements suggested.
