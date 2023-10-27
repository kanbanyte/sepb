<link rel="stylesheet" href="../styles/styles.css" type="text/css">

<!-- TOC ignore:true -->
# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s): @Slothman1, @NickMcK14, @finnmcgearey, @Huy-GV, @vkach
	Year: 2023
-->

`System Quality Assurance Plan (SQAP)`

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
## Domain Vocabulary
* **ASAP**: As Soon as Possible
* **COB**: Close of Business (5:00 PM)
* **GUI**: Graphical User Interface
* **IEEE**: Institute of Electrical and Electronics Engineers
* **SQAP**: Software Quality Assurance Plan
* **SRS**: Software Requirements Specification
* **SemVer**: Semantic Versioning
* **Project 24**: Robot Vision System For A Pick And Place Task

<div class="page"/><!-- page break -->

<!-- TOC ignore:true -->
# Contents
<!-- TOC -->

* [Chapter 1: Introduction](#chapter-1-introduction)
* [Chapter 2: Reference Documents](#chapter-2-reference-documents)
* [Chapter 3: Management](#chapter-3-management)
	* [Organisation/Roles](#organisationroles)
		* [Meeting Roles](#meeting-roles)
		* [Formal Review](#formal-review)
		* [Champion Roles](#champion-roles)
		* [Communication Roles](#communication-roles)
	* [Tasks and Responsibilities](#tasks-and-responsibilities)
		* [General Team Member Responsibilities](#general-team-member-responsibilities)
		* [Champions](#champions)
* [Chapter 4: Documentation](#chapter-4-documentation)
	* [Software Documents](#software-documents)
		* [Project Plan](#project-plan)
		* [Software Quality Assurance Plan SQAP](#software-quality-assurance-plan-sqap)
		* [Code Documentation](#code-documentation)
		* [Self-Assessment Reports](#self-assessment-reports)
		* [Software/System Requirements Specification SRS](#softwaresystem-requirements-specification-srs)
		* [System Architecture Design and Research Report SADRR](#system-architecture-design-and-research-report-sadrr)
		* [Audit Report](#audit-report)
	* [Management Documents](#management-documents)
		* [Meeting Agendas](#meeting-agendas)
		* [Meeting Minutes](#meeting-minutes)
* [Chapter 5: Standards, Practices, Conventions and Metrics](#chapter-5-standards-practices-conventions-and-metrics)
	* [Standards](#standards)
		* [Coding Standard](#coding-standard)
		* [Documentation Formatting Standard](#documentation-formatting-standard)
		* [Filename/Location Standards](#filenamelocation-standards)
		* [Software Versioning Strategy: SemVer](#software-versioning-strategy-semver)
		* [Document Releases](#document-releases)
	* [Practices](#practices)
		* [Communication Practices](#communication-practices)
		* [Meetings](#meetings)
		* [Semantic Versioning](#semantic-versioning)
		* [Coding practices](#coding-practices)
* [Chapter 6: Reviews and Audits](#chapter-6-reviews-and-audits)
* [Chapter 7: Testing](#chapter-7-testing)
	* [Requirement](#requirement)
	* [Use Case Generation](#use-case-generation)
	* [Installation and User Documentation Generation](#installation-and-user-documentation-generation)
* [Chapter 8: Problem Reporting and Corrective Action](#chapter-8-problem-reporting-and-corrective-action)
	* [Personnel](#personnel)
	* [Work](#work)
		* [Project Milestones](#project-milestones)
		* [Cross-Functional Tasks](#cross-functional-tasks)
		* [Task Creation](#task-creation)
		* [Task Assignment](#task-assignment)
		* [Task Life](#task-life)
* [Chapter 9: Tools and Methodologies](#chapter-9-tools-and-methodologies)
	* [Tools](#tools)
		* [Markdown](#markdown)
		* [GitHub](#github)
		* [SemVer](#semver)
		* [VS Code](#vs-code)
		* [IDEs](#ides)
		* [Virtual Machine](#virtual-machine)
		* [Discord](#discord)
	* [Agile Methodology: Kanban](#agile-methodology-kanban)
* [Chapter 10: Records Collection, Maintenance and Retention](#chapter-10-records-collection-maintenance-and-retention)
* [Chapter 11: Risk Management](#chapter-11-risk-management)
	* [Categorisation](#categorisation)
	* [Risks With Respect To The Work To Be Done](#risks-with-respect-to-the-work-to-be-done)
	* [Risks With Respect To The Management](#risks-with-respect-to-the-management)
	* [Risks With Respect To The Client](#risks-with-respect-to-the-client)

<!-- /TOC -->

<div class="page"/><!-- page break -->

# Chapter 1: Introduction
The robot vision system for a pick and place task will be tackled by Group 24 and following this document; the Software Quality Assurance Plan (SQAP),
it will be ensured that the projects requirements and quality standards are met.
The plan will outline the development process and testing procedures, including but not limited too reviewing, testing and integration.
In addition, the plan will describe several tools and methodologies that will be implemented and used to guarantee the solution's reliability,
maintainability and performance.
Finally, the plan will identify the team; `HeeHooVision`, responsible for the development and testing of the software as well as their roles and responsibilities.

# Chapter 2: Reference Documents
* ISO/IEC/IEEE 12207: <https://www.iso.org/standard/63712.html>
* ISO/IEC/IEEE 15288: <https://www.iso.org/standard/63711.html>
* ISO/IEC 25010: <https://www.iso.org/standard/35733.html>
* Google C++ Guidelines: <https://google.github.io/styleguide/cppguide.html>
* Google Python Guidelines: <https://google.github.io/styleguide/pyguide.html>
* Semantic Versioning: <https://semver.org/>
* Software Versioning: <https://en.wikipedia.org/wiki/Software_versioning>
* Introduction Software Versioning: <https://www.geeksforgeeks.org/introduction-semantic-versioning/>
* Software Release Life Cycle: <https://en.wikipedia.org/wiki/Software_release_life_cycle>
* Hungarian Notation: <https://en.wikipedia.org/wiki/Hungarian_notation>
* Ubuntu 22.04 LTS: <https://ubuntu.com/download/desktop>
* Kanban Agile Methodology: <https://www.nimblework.com/kanban/what-is-kanban/>

<div class="page"/><!-- page break -->

# Chapter 3: Management
## Organisation/Roles
The following section will detail the various roles within the team for various scenarios.

### Meeting Roles
These roles will be for regular meetings to discuss the current progress of the project and
will help ensure successful completion of the project and cohesion between team members.\
The meeting roles will consist of: Team Leader, Meeting Minutes, and Supervisor.

#### Team Leader
Coordinates meetings and ensure effective communication between meeting members.
Summarises the meeting, describing and allocating any future tasks to the relevant members.

#### Meeting Minutes
Records any important information and meeting minutes during the meeting for future reference and records.

#### Supervisor
Ensures meeting remains within time and each member is able to state and discuss their relevant item.
Keeps track of meeting minutes for the scribe to record.
Makes sure that only nobody talks over another to keep the discussion understandable and time efficient.

### Formal Review
Formal review meetings will be conducted to thoroughly check finished work in order to maximise the quality of the software architecture and design.
During the meeting, team members should provide constructive feedback and
ensure that the software architecture and design aligns with the project's objectives and adheres to quality standards.

<div class="page"/><!-- page break -->

### Champion Roles
A champion in a role is considered the primary person responsible for the quality of the work associated with that role.
While multiple team members could take on the same role, the champion of that role is essentially the leader of the role.
Champion roles are crucial for ensuring that the software is designed and developed according to best practices, meets the requirements of the project,
and is maintainable and scalable over time.\
The following will be the champion roles for the project: Software Architect, Software Designer, Software Developer, Cobot/Hardware Champion,
OS Integration Champion, API Champion, Documentation Champion, GitHub Management Champion, and Kanban Methodology Champion.

#### Software Architect
Responsible for creating the overall architecture of the system, defining its components and interactions.

#### Software Designer
Designs specific modules and subsystems utilising the architectural plans.

#### Software Developer
Responsible for writing the code that implements the design, working closely with the designer to ensure that the code meets the design specifications and requirements.

#### Cobot/Hardware Champion
Will be primarily responsible for working with the cobot and ensuring all code is compatible and functional.

#### OS Integration Champion
Ensures that all code functions on Ubuntu as there may be compatibility issues between Windows and Ubuntu.

#### API Champion
Should have extra understanding regarding the APIs used such as ROS to ensure no unnecessary development of functions occurs.
They will also ensure the functions used are the most appropriate for the situation.

#### Documentation Champion
Ensures all code documentation is descriptive and explains the relevant functions and modules sufficiently.
Additionally, they will make sure all developers write concise and informative comments in their code to facilitate documentation creation.

#### GitHub Management Champion
Has expertise in managing GitHub repositories that will allow for better collaboration and efficiency.
They will be able to answer questions regarding GitHub tools and techniques that will make creating and editing work and software much easier.

<div class="page"/><!-- page break -->

#### Kanban Methodology Champion
Expertly understands the Kanban methodology which provides the team with an efficient and more individualistic working approach.
This champion role will help the team meet goals by providing them with a better understand of the Kanban methodology.

### Communication Roles
The communication roles are those necessary for effective communication between team members and between the client and stakeholders.
These roles will take on the responsibility of understanding the relevant information they need to communicate and
be able to clearly provide such information to the group they need to communicate with.\
The communication roles to be established will be: Project Supervisor, Client, Team Leader, Quality Manager, Support Manager, Development Manager, and Planning Manager.

#### Project Supervisor
The project supervisor will be consulted regarding information about the project and will provide feedback on work when necessary.
They will also communicate with the team leader should they need to contact the team regarding any work or updates on project progress.

#### Client
The client will communicate with team members should they require updates on progress.
Team members, particularly those working with the cobot, can correspond with the client to seek clarification on specific requirements.

<div class="page"/><!-- page break -->

#### Team Leader
Responsible for facilitating communication between team members and ensuring that communication is effective and efficient.
They should also ensure that the project's software design documentation is complete, accurate, and
up-to-date to facilitate communication with the development team and stakeholders.

#### Quality Manager
Quality managers ensure quality is maximised throughout the project,
and therefore will be responsible for communicating quality requirements to the relevant team members.
They will also provide the project supervisor and client with updates regarding the quality of the deliverables currently being worked on.

#### Support Manager
The support managers are responsible for providing support to other team members, the project supervisor, or the client with regards to the project.
They will communicate with other team members to help them with work, research, or testing if necessary.
Should the supervisor or client require assistance with understanding the deliverables or software,
the support managers should relay information to them regarding these aspects.

#### Development Manager
Development managers coordinate with the software developers to keep progress steady and ensure code functions correctly.
They will be able to provide the supervisor, client, and team leader with updates regarding the development and testing process.

#### Planning Manager
The planning manager plans for the future documents, software components, etc.
to be delivered by potentially allocating work to the relevant members.
The will coordinate other managers to be able to meet requirements and deadlines to keep the project progressing efficiently and consistently.

<div class="page"/><!-- page break -->

## Tasks and Responsibilities
### General Team Member Responsibilities
The team members involved have several key responsibilities.

These responsibilities include:
* Understanding the overall project requirements.
* Creating:
	* an architectural design that meets those requirements.
	* a detailed design that defines the components and interactions of the software.
* Ensuring that the design is:
	* of high quality and adheres to the project's coding standards.
	* testable, and that it meets the project's functional and non-functional requirements.
* All team members must maintain accountability of tasks they're responsible for and ensuring that others members are also held accountable for their responsibilities.
* Each team member is responsible for communicating any issues that may hinder their input into the project and are encouraged to ask for help when needed.

<div class="page"/><!-- page break -->

### Champions
#### Team Leader
* Must ensure that all team members are keeping up to date with work and tasking.
* Responsible for organising and holding meetings with supervisor, client or team members.
* Will be the point of contact for any inter-personal issues that may arise.

#### Client Liaison
* Is responsible for maintaining contact with client and ensuring the client is up to date with the progress of the project.
* Must ensure that any communication with client is mindful and respectful.
* Will be the point of contact if any team member or the client needs to communicate with each other.

#### Support/Quality Manager
* Must ensure that documentation is completed by all developers and maintains certain standards.
* Must be able to provide assistance should any team member require it.

#### Developer
* Must ensure that all code follows standards and practices set out at the beginning of development.
* Responsible for delegating coding tasks and workloads.
* Must ensure that code is being developed at an appropriate pace to ensure targets are met.

#### Planning Manager
* Must ensure that all released versions follow label accordingly following standards and principles.
* Must ensure that all team members work aligns with the next planned version release.
* Will be able to provide assistance should any team member require it.

#### Development Manager
* Must ensure that all testing tasks are delegated appropriately.
* Must ensure that all testing ensures that the results satisfy requirements of the project.
* Responsible for making all team members aware of the results of testing.

<div class="page"/><!-- page break -->

# Chapter 4: Documentation
## Software Documents
This section will provide an overview of the various software documents that will be created to ensure that the software architecture and
design are thoroughly documented and therefore highly intelligible.

### Project Plan
This document will give a general outline of the project direction, including:
* Overview of the project description and background information.
* Details about the client, stakeholders, and other involved individuals.
* Objectives of the project.
* Acceptable standards and required skills necessary for completion.
* Project deliverables.
* Required research.
* Design, implementation, and testing.
* Risks associated with the project.
* Schedule and time line.

### Software Quality Assurance Plan (SQAP)
The SQAP is a document that describes the measures to be taken to ensure the maximisation of the quality of the project deliverables.\
This document will contain:
* The purpose of the document.
* The various roles required to successfully complete the project.
* Tasks and responsibilities of each member and role.
* Outline of the documentation that will be produced.
* Standards and practices that will be followed.
* Information about reviews and audits.
* Testing information.
* How problem reporting and corrective action will be taken.
* Tools and methodologies that will be used.
* Record collection and maintenance.
* Risk management.

<div class="page"/><!-- page break -->

### Code Documentation
Code documentation will contain the following:
* Description of the various created modules.
* Description of functions and classes and how to utilise them in code.
* Interdependency of each module or dependencies on external modules.

### Self-Assessment Reports
These reports will be introspective documents reporting on how each member thought each other performed, which will contain:
* A review of the client.
* Categories regarding metrics such as quantity and quality of work, communication, etc.

### Software/System Requirements Specification (SRS)
The SRS will describe the requirements of the software/system that will be developed.
It will include:
* Purpose of the document.
* Overall description of the system that is to be developed.
* Features of the software.
* System requirements such as hardware requirements.
* Acceptance Criteria for the software.
* Documentation to be delivered.
* Functional requirements of the software.
* Quality requirements.
* Interface requirements such as user interfaces, and hardware/software interfaces.

### System Architecture Design and Research Report (SADRR)
This document will give information about the overall system architecture and design, as well as additional research into the project.
Contained within this document will be:
* Overview of the document.
* Problem analysis which will outline the goals of the software system and any assumptions and simplifications.
* High-level system architecture and alternatives
* Additional research
	* Research into the application domain
	* Research into the system design
	* Research into platforms, languages, and tools

### Audit Report
Audits, whether carried out internally or externally, must produce a document.\
Anything that is found that doesn't follow the processes outlined in the SQAP will result in corrective actions.
Audit reports should assess the projects scalability, maintainability and suitability.
Additionally an evaluation of the design documents should be carried out during an audit.

<div class="page"/><!-- page break -->

## Management Documents
### Meeting Agendas
* This document will be of a markdown type (`.md`) and will be prepared by the team leader prior to each meeting.
* Team members are expected to contribute to the agenda, upon request their topic will be added.
* Requested topics are expected to be presented by the team member that requested them, they are expected to attend the meeting ready to present.
* Owners are expected to attend the meeting and topics accepted before the end of day prior to the meeting.

### Meeting Minutes
* Will be collected at every meeting.
* Must follow the minutes template as outlined in the GitHub repository.
* Will be collected as either a raw .txt file or Markdown file.
* Formatted minutes will be released in the repository no later than the following day's COB.
* A PDF copy of the minutes can also be emailed upon request, however,
members are expected to find the minutes in the repository and complete their actions independently.

<div class="page"/><!-- page break -->

# Chapter 5: Standards, Practices, Conventions and Metrics
Ensuing good quality work is produced standards are essential;
this section will cover many standards of varying parts of the programming process.\
These standards aim to provide a structured approach to software design and development,
these primarily being in regards to scalability, maintainability and integrability.\
These standards also provide a basis for testing and validation, ensuing that the system performs as expected and meets the expectations of the client.
By adhering to these standards the development team can mitigate potential risks and ensure the successful and satisfactory delivery of the project.

## Standards
The proposed software development process will adhere to a number of industry standards including:
* [ISO/IEC/IEEE 12207](https://www.iso.org/standard/63712.html) for software development life cycle processes.
* [ISO/IEC/IEEE 15288](https://www.iso.org/standard/63711.html) for system life cycle processes.
* [ISO/IEC 25010](https://www.iso.org/standard/35733.html) for software product quality.

These standards will ensure that the project is developed in a systematic and controlled manner, meets quality requirements, and is maintainable in the long term.
To ensure compliance with standards, the project team will identify and adopt additional guidelines relevant to the project's domain and requirements.

### Coding Standard
The software produced should follow coding standards that allow for the code to be more easily maintainable, readable, and errorless.
Therefore, the standards will describe rules for variable naming, code formatting, commenting, and error handling, which will be enforced through code reviews.\
Two standards in particular will need to be adhered to:
* [Google C++ Guidelines](https://google.github.io/styleguide/cppguide.html)
* [Google Python Guidelines](https://google.github.io/styleguide/pyguide.html)

These standards were chosen because they are very comprehensive and allow for a high-quality production of code, thus making it easier to understand and maintain.
Any code not adhering to these particular standards will be corrected before deployment to maintain quality standards.

Further information about coding practices can be found in the [coding practices](#coding-practices) section.

<div class="page"/><!-- page break -->

### Documentation Formatting Standard
All project documentation should be formatted according to Markdowns documentation formatting standards to ensure consistency and readability.
The formatting standard defines rules for all Project 24's documentation to ensure consistency and readability.\
By adhering to these guidelines, team members can quickly and easily find information and understand the contents of the documents.

These standards include:
* Use clear and concise writing
* Organize content using headings and subheadings
* Use bullet points or numbered lists to structure information
* Include tables and diagrams when appropriate to clarify information
* Use a consistent font and font size throughout the document
* Include a table of contents for multi-page documents
* Use a consistent file naming convention

In addition to these guidelines, all documentation should be created using Markdown.\
All team members are responsible for ensuring that their documentation adheres to the formatting standard.\
In addition, specific document types related to Project 24 may have additional formatting requirements, which will be outlined in their respective templates.

By adhering to the documentation formatting standard and using Markdown,
we can ensure that all project documentation related to Project 24 is consistent, clear, and easy to understand.

<div class="page"/><!-- page break -->

### Filename/Location Standards
* Management documents will be contained in their own folder in case more are added or needed.
* All folder names will be in lowercase.
* Whitespace will not exist in folder names, using `_` instead.
* A Markdown document will be kept in the main folder and act as a guide to the software.
* Files of the same type will get their own folder, for example meeting minutes and progress reports.
* Code files will be named after the appropriate task or class they provide.
* For files that require dates they will be presented with a DDMMYYYY format

#### Document Tree
The following document tree describes the SVN structure.\
Additional folders may be added at the discretion of team members after consulting with the SVN champion.

The following describes the Folder structure from the perspective of the **main** branch of the repository.
Additional folders exist; however, this should give an overview of expected conventions.
```diff
main
|	.gitignore
|	project24proposal.md
|	README.md
|
+-------.github
|	|	PULL_REQUEST_TEMPLATE.md
|	|	release.yml
|	|
|	\-------ISSUE_TEMPLATE
|			config.yml
|
+-------assessments
|		contribution.md
|		criteria.md
|		design.md
|		plan.md
|		sadrr.md
|		self-peer_review_template.md
|		sqap.md
|		srs.md
|
+-------minutes
|		minutes_template.md
|
+-------styles
|		contribution.css
|		styles.css
|		worklog.css
|
\-------worklogs
		worklog_template.md
```

<div class="page"/><!-- page break -->

### Software Versioning Strategy: SemVer

![three-part_version_number](https://upload.wikimedia.org/wikipedia/commons/thumb/8/82/Semver.jpg/330px-Semver.jpg)

*<sup>Figure Source: <https://en.wikipedia.org/wiki/Software_versioning></sup>*

Versioning is a critical aspect of software development that helps developers and users manage changes to a project.\
Semantic Versioning, or SemVer for short,
is a widely-adopted standard for versioning software projects that provides clear guidelines for how to version software releases and manage dependencies between them.

The SemVer standard uses a three-part version number in the format "MAJOR.MINOR.PATCH" to convey information about changes to the software:
* The MAJOR version number indicates significant changes that may not be backwards-compatible.
* The MINOR version number indicates an addition of backwards-compatible functionalities.
* The PATCH version number indicates backwards-compatible bug fixes or minor changes.

![different_components](https://media.geeksforgeeks.org/wp-content/uploads/semver.png)

*<sup>Figure Source: <https://www.geeksforgeeks.org/introduction-semantic-versioning/></sup>*

In order to use SemVer effectively, it's important to follow these guidelines:
* Increase the MAJOR version when making incompatible changes
* Increase the MINOR version when adding new functionality in a backwards-compatible manner
* Increase the PATCH version when making backwards-compatible bug fixes or other minor changes
* Use pre-release version numbers (such as 1.0.0-alpha) to indicate that a version is not yet stable or complete
* Use build metadata (such as 1.0.0+build.1) to indicate additional build information without changing the version semantics

Adherence to SemVer ensures that version numbers convey meaningful information about the state of the software and the nature of changes between releases.
This enables developers and users to make informed decisions about which versions of a project to use and when to upgrade, and
helps prevent compatibility issues between different versions of the same project.

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/0/07/Software_dev2.svg/360px-Software_dev2.svg.png" style="width: 40%; height: 40%"/>​

*<sup>Figure Source: <https://en.wikipedia.org/wiki/Software_release_life_cycle></sup>*

By following the guidelines provided by the SemVer standard,
teams can ensure that their software projects are versioned in a consistent and predictable way,
making it easier to manage dependencies and collaborate with other teams or individuals.

### Document Releases
If the need arises that a document is required for a University submission or by the client,
a systematic approach will be taken towards the development of documents within the team.
The following practices will ensure that quality control is maintained by all participants:
* Document generation and collaborative work will take place primarily on Github.
* Team members will all work collaboratively to assign themselves and others task and sections of documents to work on.
* Team members will ensure that fellow members are happy with their work by using Github features such as pull requests and issues to verify.
* Once all sections have been completed by all participants, the report will be export to .pdf format for submission.
* The final results is to be validated by all team members before submission to ensure that mistake are minimal and all required information is covered.

<div class="page"/><!-- page break -->

## Practices
This section will cover several practises that will be employed as a basis for quality control within the project.\
These practises will be followed closely throughout development, frequent audits will ensure that this is the case.\
Through rigorous upholding of the outlined practises the quality of the project will be maintained and ensured the deliverables be of high quality.

### Communication Practices
Effective communication between all parties associated with this project will use the practices laid out in this section to ensure quality control and
that the project progresses smoothly.
Communication with the following parties will use the respective practices:

#### Client
* Communication will occur primarily through Group Leader.
* Regular updates as to progress of research/development.
* Communication to primarily occur through email, face-to-face meetings can be arranged on an as needed basis.
* Communication is to be of a formal and respectful manner.
* All team members will be appraised of communication with the client, unless urgency requires otherwise.

#### Team
* Communication within group will primarily occur through Discord.
* Regular team meetings to discuss progress, issues, and feedback.
* Meetings will be arranged on Discord, unless a face-to-face meeting is required.
* Attendance to meetings is expected, except when absence has been discussed and understood by other team members.
* Collaborative work will be fostered by using Github features such as Discussions and Pull Requests for development.
* A Kanban Board will be used as a project management tool to track issues and pull requests, and to keep all team members up-to-date.

#### Supervisor
* Communication will occur primarily through Group Leader.
* Weekly meetings on Microsoft Teams to be held between team members and supervisor to discuss current progress and challenges being tackled.
* Communication to primarily occur through email, in-person meetings can be arranged on an as needed basis.
* Communication is to be of a formal and respectful manner.
* Inter-personal issues will be reported to supervisor to discuss possible actions and/or solutions for resolution.
* All team members will be appraised of communication with the client, unless urgency or personal issues require otherwise.

<div class="page"/><!-- page break -->

### Meetings
Regular meetings with all team members present will be held. The following goals and tasks should be completed for each week:
* All members are to attend each meeting.
* If a member cannot attend a particular meeting, all other members are to be notified ahead of time.
* All members are to provide updates on progress of current tasking, challenges that were completed and are currently being tackled.
* Members can use this time to ask for help from other members if required.
* A meeting agenda will be constructed for each regularly scheduled meeting to ensure all topics requiring discussion are covered.
* A meeting minutes document will be constructed for each meeting to ensure that all topics covered in the meeting are recorded accurately.
* A member of the team must be assigned the responsibility of both documents for each week.
Or different member can be delegated the task each week on agreement of all team members.

### Semantic Versioning
The software versioning strategy for this project will follow Semantic Versioning (SemVer).\
This strategy will ensure that versions are released in a manner that is compatible with the software dependencies and usage.\
This strategy will provide clarity to the team and stakeholders about the purpose and impact of each software release.

<div class="page"/><!-- page break -->

### Coding practices
This section includes coding guidelines.
Any rule mentioned in this documents will override third-party guidelines.

C++:
* Follow [the Google C++ Guideline](https://google.github.io/styleguide/cppguide.html).
* Always use Java-style indentations, regardless of number of lines.
* For `.cpp` files, place `#include` directives in the following order:
	1. Header file whose declarations are defined in the current file.
	2. Standard library header files.
	3. Third-party library header files.
	4. Other header files in the project.

Python:
* Follow [the Google Python guideline](https://google.github.io/styleguide/pyguide.html).

Shared:
* Public interfaces (functions, variables, constants, class names, etc.) must be fully documented.
* All classes must belong in separate files unless they are nested.
* All rules in the [project structure section](#guidelines-on-project-structure) and [namespace section](#guideline-on-naming-and-namespaces).

#### Guidelines on project structure
* Source files are placed in `src/`
* Tests are placed in `tests/`
* Documentation are placed in `documentation/`

#### Guideline on naming and namespaces
C++:
* All code must be placed in a namespace.
* Use `PascalCase` for templates, namespaces, enum names, enum values and class names.
* Use `SCREAMING_SNAKE_CASE` for constants.
* Use `snake_case` for functions, local variables, function parameters and member fields.
* Prefix private, non-constant member fields with underscore `_` instead of using the `this` pointer unless required.
* Avoid the [the Hungarian notation](https://en.wikipedia.org/wiki/Hungarian_notation).
* Prefer `using foo::bar` over `using namespace foo`.

Python:
* Use `PascalCase` for classes and errors.
* Use `snake_case` for functions, local variables, function parameters, member fields.
* Prefix private, non-constant member fields with underscore `_`
(Python does not support private variables but this should rule should be adhered to where possible for communication purposes).

<div class="page"/><!-- page break -->

# Chapter 6: Reviews and Audits
Outlined in this section will be a set of procedures used to validate project deliverables and
to verify team processes in regards to the defined requirements and standards.\
With regards to validation, it will be checked through internal and external reviews, and verification through audits.
These reviews and audits will help ensure that deliverables are up to scratch and product quality is maintained.
The information on these reviews and Audits are found earlier in the document under chapter 5.

# Chapter 7: Testing
For the testing phase of the project, a comprehensive approach will be used to ensure that the Robot Vision System is functioning optimally.

Unit testing will be performed on individual software components to verify their correct functionality.
This will likely make up most of the initial testing as the team has a lack of experience with computer vision and
robotic control so we will need to validate that our solutions are on the right track as we learn new skills.
This will largely be assisted by feedback from the client and other team members.

Integration testing will be conducted to ensure that the various components of the system are working together effectively.

Additionally, system-level testing will be carried out to validate that the entire Robot Vision System meets the project's requirements.
The testing process will be automated where possible, and any issues that arise will be documented and addressed promptly.
The team will also perform some usability/function testing to test edge cases where the placement of parts and
robot arm are in incorrect positions to test the robustness of the software solution and see what exceptions are thrown.
These tests will allow the team to design user friendly error messages.

We will work with the client to create metrics we can use to measure the success of the solution.

<div class="page"/><!-- page break -->

## Requirement
Our testing will aim to validate that our solution meets all the requirements outlined in the SRS above.\
This will require them team to have clear communication with the client to ensure their needs are met by our software.\
This will allow the team to create effective tests, specifically tailored to the requirements of the client.

## Use Case Generation
For the use case generation, we need to identify the various scenarios in which the robot will operate and the interactions with the system.

The use cases will:
* include details about the inputs, outputs, and actions of the system.
* be generated based on the requirements and will help in validating the system's functionality.
* also help in identifying any potential issues or edge cases that need to be addressed during the development process.

We will need to define our input domain in order to generate valid test cases.
For our software solution, this is all possible positions of chips, covers, batteries and trays.

Our system would be considered as an untestable program because the output cannot be verified, meaning it doesn't have a test oracle.
A test oracle is a procedure in which the outputs of a system can be verified against.
This means we will have to use metamorphic testing.

Metamorphic testing is defined by the following process:
* Defining an initial test case
* Identifying properties of the problem and metamorphic relations (MR)
* Creating follow-up test cases from the initial test case using previously developed MRs
* Verifying the MRs using the systems outputs

For this software solution,
an example of a MR is that the position of a chip placed in the holding bracket is the same regardless of the number of chips placed in adjacent slots.
Therefore, the theoretical output of the system should be that it can identify a specified chip irrespective of any arrangement of surrounding chips.

## Installation and User Documentation Generation
Effective Installation and User Documentation is an essential part of software development as it ensures that users can easily install, configure and use the software.

To achieve this, the project should include comprehensive and easy-to-follow installation instructions and user manuals for the vision or sensing system.
The documentation should be written in clear and concise language, and include relevant screenshots and diagrams to aid the users.
Additionally, the installation process should be rigorously tested to ensure that it is error-free and robust, and
the user documentation should be updated regularly to reflect changes in the software.

Our software solution doesn't require installation, the executable is run at time of operation.
All input and output files will be incorporated as part of the GUI.

The client will be given a full user guide that will outline how to interact with the GUI.
This document will give the client and user examples of how the software solution works and how it is to be used.
The code and the design documents' documentation will also be sent to the client.
These documents will allow the client to successfully maintain and implement our software solution in future projects.

<div class="page"/><!-- page break -->

# Chapter 8: Problem Reporting and Corrective Action
## Personnel
If an issue arises with personnel, the Team Leader is to be notified immediately, and
they will propose corrective action, which can include team reorganization, or protocol changes.
In the event of issues arising amongst team personnel, they should first be brought to the attention of the project manager or team leader for resolution.
Corrective action may include training, reallocation of tasks, or a discussion of expectations to ensure that team members are able to perform their roles effectively.
It is the responsibility of all team members to report any issues promptly to ensure that they are addressed in a timely manner.

## Work
### Project Milestones
Establishing project milestones is crucial to ensure successful planning, development, testing, and
deployment of the Robot Vision System using Kanban Agile methodology and Semantic Versioning.
A realistic and feasible set of project milestones should account for each stage of the project's lifecycle and
ensure each milestone is achieved within the specified time frame.
There is one project that being; **developing a perception system for a cobot**.\
This project will then have milestones to fulfil the Kanban methodology and allow for more focused goals.\
Some examples of these milestones are as follows:
* Mounting the camera and the information from the camera being processed.
* System is integrated with ROS correctly.
* Initial test build uploaded to provided hardware and tests pass.
* Machine learning model built and has a correct analysis of area at least 65% of the time.

The milestones should align with the Kanban board and consider all phases of the project lifecycle while factoring in possible risks and contingencies.
GitHub's issues, pull requests, and discussions can help in tracking progress,
identifying and resolving issues, and labelling each milestone to indicate the stage of development.

Regular reviews and updates of the project milestones are necessary to ensure alignment with the Kanban board and timely achievement of each milestone.
Any changes to the milestones should be communicated and discussed with the project team using GitHub's discussions to ensure everyone is on the same page.

<div class="page"/><!-- page break -->

### Cross-Functional Tasks
Cross-functional tasks and issues must indicate the base version they belong to, such as **Project 24 v0.1.0-a**.\
This information is essential for maintaining software quality and facilitating collaboration among cross-functional teams.\
For example, tasks may fall under the scope of **Project 24 v0.1.0-a** and
require collaboration among team members with expertise in different areas such as computer vision, sensors, robotics, and AI.

To ensure successful completion of cross-functional tasks, the team will adopt an Agile methodology using Kanban board to manage the development process.
Finally, the team will use Semantic Versioning (SemVer) for software versioning and
GitHub for source control to ensure that code changes are properly tracked and managed.

### Task Creation
Task creation involves several steps to ensure proper documentation and tracking throughout the development process.\
Requirements for pick and place tasks are analysed to identify specific system needs.

The team creates a design specification outlining how system components interact.\
Tasks are assigned to team members based on priority, and progress is monitored daily.

Issues and bugs are tracked using GitHub and resolved by the appropriate team member.\
Following these guidelines ensures proper task tracking, documentation, and consistent software quality.

### Task Assignment
Tasks will be assigned to team members based on their respective areas of expertise and availability.\
The Development Manager will be responsible for initially assigning tasks to team members.\
If a task cannot be assigned at the time of creation, the Development Manager will ensure that the task is assigned within 24 hours.\
Team members who are assigned tasks are expected to provide progress updates to the Development Manager on a regular basis.

### Task Life
Tasks would be created as **issues** on GitHub, and then moved across the Kanban board's columns as they progress through the workflow.
Each task would have an assignee responsible for working on it, and
a resolver responsible for ensuring that solutions are checked against appropriate standards and practices before **approving** the changes via *review*.

The task life would also include testing and maintenance stages, in which the task is verified and validated for functionality and
then maintained to ensure its continued operation and improvement over time.
Finally, the task would be *closed* via a **pull request** after it has been formally *reviewed* by the respective champion and *merged* into the **main** branch.

<div class="page"/><!-- page break -->

# Chapter 9: Tools and Methodologies
## Tools
### Markdown
This lightweight markup language is easy to learn with plain text formatting syntax that can be converted to other formats like PDF, and
produces HTML files that can be viewed in any web browser.\
A recommended editor for Markdown files is Visual Studio Code, which provides syntax highlighting and preview functionality.

### GitHub
GitHub is used as the issue tracking tool, providing a user-friendly and feature-rich platform to track issues, progress, and perform code reviews.
By utilizing GitHub for issue tracking,
team members can easily collaborate and communicate on project tasks, leading to more efficient and effective issue resolution.

### SemVer
All versioning updates by team members shall follow Semantic Versioning (SemVer) principles to avoid potential issues.\
By adhering to SemVer, team members can ensure that software versioning is consistent and understandable across the entire team.\
It is up to each individual to familiarize themselves with SemVer principles and incorporate them into their versioning workflow.

### VS Code
VS Code (Visual Studio Code) is the recommended code editor for the project, providing powerful debugging tools, built-in Git support, and a wide range of extensions.
By utilizing VS Code, team members can collaborate more efficiently and ensure consistency and quality in the project's codebase.
Its versatility and ease of use make it an ideal tool for project development.

<div class="page"/><!-- page break -->

### IDEs
In the development of software you need something to create and modify code files, below will be a select few of some possible ones for the project.

#### Visual studio 2019/2022
A development environment made by microsoft Visual studio is a powerful IDE specialised in C type language development.
With extensive support for C, C++ and C# it is the ideal and recommended solution to software development in C type languages.\
Visual studio also provides tools for developing other languages,
however they are lacking in as extensive features as the languages described above and other options should be sought out.

#### Pycharm
Pycharm is a specialised development environment for Python development,
with many helpful features PyCharm is a comprehensive and feature rich solution to Python programming.
With a variety of packages and options to customise the coding experience PyCharm allows programmers to explore all aspects to Python programming.

### Virtual Machine
The software system is expected to be compatible with Ubuntu 22.04 LTS.\
A virtual machine is the recommended development environment for team members who do not use Ubuntu as their primary operating system or dual boot into Ubuntu.

The recommended virtualizer is [VirtualBox 7.0.6](https://www.virtualbox.org/wiki/Downloads)
(available on both Windows and MacOS) running with the [Ubuntu 22.04 LTS](https://ubuntu.com/download/desktop) disk image.\
The virtual machine should have a minimum of 8 GB of RAM and 25 GB of storage.\
The host machine must support and enable virtualization.

### Discord
In order for team members to participate in Discord meetings, it is necessary for them to download and install the application on their device.
They also need to have access to a microphone and speakers to be able to communicate, and
should familiarize themselves with basic Discord features e.g., screen sharing.
Following these guidelines will ensure that all team members are prepared for successful meetings on Discord.

<div class="page"/><!-- page break -->

## Agile Methodology: Kanban

![kanban](https://www.nimblework.com/wp-content/uploads/2022/12/Simple-Kanban-Board-5-1024x628.webp)

*<sup>Figure Source: <https://www.nimblework.com/kanban/what-is-kanban/></sup>*

The Kanban Agile Methodology approach will prioritize collaboration, flexibility, and continuous improvement.
The Kanban board will be used to visualize the workflow and identify bottlenecks, enabling the team to respond quickly to changes and adapt the design accordingly.
Regular communication and meetings will ensure that the design is meeting the project requirements, and
the team will focus on delivering small, incremental changes to the design, allowing for feedback and iteration throughout the project lifecycle.

Some key points for utilizing Kanban for software architecture and design in this project are:
* Breaking down the overall design into smaller, more manageable components, and creating a backlog of tasks that need to be accomplished.
* Prioritizing the backlog based on importance and complexity, and assigning each task to a specific team member or group.
* Utilizing Kanban boards to visualize the workflow, track progress, and ensure visibility and collaboration among team members.

Kanban is an iterative software development process that emphasizes flexibility, continuous improvement, and customer collaboration.
This approach can help ensure that the software architecture and design process is efficient, effective, and customer-centric.
By delivering small, incremental changes and seeking continuous feedback, the team can ensure that the final product meets the needs of its users.

<div class="page"/><!-- page break -->

# Chapter 10: Records Collection, Maintenance and Retention
Management documents, which encapsulates Minutes, Agendas and Notes, will be added to the teams GitHub repository.\
Any form of administration in terms of breaches in standards and practices will be documented on the GitHub repository.\
Documents that are added to the GitHub repository and finalized should not be modified.

# Chapter 11: Risk Management
Unforeseen events can and will happen during the course of this project.
To ensure that a functional and high quality product is delivered on schedule, it is vital that risks are identified, analysed and accounted for.
This section involves risk categorization and listing corresponding countermeasures.

## Categorisation
For this project three major categories of risks have been identified:
1. Risks with respect to the work to be done.
2. Risks with respect to the management.
3. Risks with respect to the client.

In the following sections each of these categories have their major risks identified.
For each risk, a description, a probability of occurrence, its level of impact, mitigation strategy and contingency plan are given.

Both the probability of a risk occurring and its effect are quantified as being low, moderate or high.
Risks are managed with a mitigation strategy and a contingency plan.
The mitigation strategy is designed to reduce the likelihood of risks occurring and the contingency plan is used to minimize the impact of materialized risks.

<div class="page"/><!-- page break -->

## Risks With Respect To The Work To Be Done
|Rank|Name/Description|Occurrence Probability<br/>(H/M/L)|Severity<br/>(H/M/L)|Mitigation Strategy|Contingency|
|:-:|:-|:-:|:-:|:-|:-|
|1|GitHub repository loss.|L|H|Recurrent backups.|Restore using latest backup.|
|2|Temporary member absence.|L|M|Allocate work to at least 2 members.|Organize work handover to another member.|
|3|Inappropriate design.|M|M-H|Regular communication and contingency design.|Focus on fixing pain points or use the second design.|
|4|Un-deployable software.|M|H|Frequent deployment.|Re-prioritize tasks to focus on fixing deployment issues.|
|5|Lacks of skills/knowledge.|M|H|Start training early.|Re-assign tasks or allocate additional members.|
|6|Time shortage.|H|H|Careful and conservative planning.|Re-prioritize tasks and contact supervisor/client.|

1. GitHub repository loss.\
GitHub is a highly reliable product trusted by organizations of different scales, so the probability of a repository being lost,
either because of data corruption or cybersecurity breaches are low.
However, as the centralized storage for code and documents, its loss will severely affect the project.\
The repository should be often copied and retained by several members to reduce this risk.
In the unlikely event of repository loss, the latest back up is used and the team will perform a general check to find any missing material.
2. Temporary absence of team member.\
Team member can be temporarily absent due to illness or personal matters, and may not complete assigned work on time.
This temporary loss of productivity has a moderate impact on the team's progress.\
The risk can minimized by spreading work to multiple team members.
If a team member is absent and is unlikely to finish their task on time, the task will be transferred to a different member immediately.
The absent team member should alert the team of their condition and time estimates as soon as possible so the team can react accordingly.
3. Inappropriate design.\
The design step is crucial because it shapes how the product is created.
Miscommunication, lack of understanding or lack of perspectives can easily lead to bad design
and the implementation stage will be severely impacted if an unsuitable design is selected.\
Regular communication and strict peer reviews will help detect and eliminate problematic design early.
It may also be helpful to create contingency design in advance.
If it becomes apparent that the product is built along a bad plan, the team should find and fix pain points, or employ the contingency design.
4. Un-deployable software.\
Both the robotic control software and computer vision system are developed locally,
and there is a moderate risk of failing to deploy them to the respective hardware i.e. the robotic arm and depth camera.
This results in a loss of product functionality or full capabilities not delivered by the deadline.\
The preventative action for this risk is frequent deployment incomplete software to guarantee compatibility.
If a deployment fails, depending on the stage of the project, the team will assign a reasonable number of members to investigate and devise a fix.
5. Lack of skills or knowledge.\
Although team members come from a variety of background, they might find themselves unable to complete their task due to a lack of knowledge or experience.
The result is incomplete or low quality product, and the issue can propagate to other dependent tasks or features.\
The team can remedy this by familiarizing themselves with relevant concepts and techniques.
If a member overly struggles with their task, the team can swap their tasks or re-allocate an additional member to provide assistance.
6. Time shortage
The project can be incomplete by the deadline due to a variety of reasons: team member absence, project difficulty, management trouble.
The probability of this risk for a complex project is high, and it leads to a product not delivered as originally outlined,
or delivered without fulfilling all quality standards.\
Cautious planning, a conservative timeline as well as vigilant monitoring will reduce the probability of this happening.
If the team anticipates that the project will not be finished on time, a meeting is held to re-prioritize tasks based on their progress and importance.
The client and supervisor will also receive notifications in such case to provide further assistance.

<div class="page"/><!-- page break -->

## Risks With Respect To The Management
|Rank|Name/Description|Occurrence Probability<br/>(H/M/L)|Severity<br/>(H/M/L)|Mitigation Strategy|Contingency|
|:-:|:-|:-:|:-:|:-|:-|
|1|Team leader absence.|L|M-H|Co-leader selected in advance.|Emergency vote.|
|2|Team member leaves.|L|H|Distribute work between at least 2 members.|Organize work handover to another member.|
|3|Inability to hold regular team meetings.|L-M|M-H|Members communicate their schedules|Members who cannot attend meetings provide updates to the team asynchronously.|
|4|Supervisor absence.|L|L|None|Asynchronous updates and catch-up meeting if needed.|
|5|Conflicts within the team.|M|H|Regular communication.|Irregular meetings to resolve disputes.|

1. Temporary absence of team leader.\
The team leader can be temporarily absent due to illness or personal matters, and its impact varies depending on when it happens.\
To ensure smooth operation at all times, the team can pre-emptively allocate a second leader who automatically assumes control when the primary leader is away.
If both leaders are absent, an emergency team meeting is held to select another temporary leader.
2. Team member leaves.\
This is a relatively low risk due to the importance of the capstone project.
However, its impact is high because the team's total productivity is reduced and prior planning might no longer be appropriate.\
There is no way to prevent this from occurring, but to minimize its impact,
all tasks should be broken up and allocated to as many members as possible to avoid dependence on a single person.
3. Inability to hold regular meetings with all members present.\
This is a moderate-low risk.
The project requires frequent communication between team members and ideally the team should hold regular meetings to keep all members up to date.
Since each member has work and academic commitments it is possible that there may not be free time for a meeting where all members can attend.\
The risk can be mitigated by members communicating their schedules with others.
In the worst case scenario, members who cannot attend regular meetings need to write separate report and update themselves with the team progress on their own.
4. Temporary absence of Supervisor.\
The supervisor can be temporarily absent due to illness or scheduling conflict.
This is a low risk and depending on the stages of the project, will not have a significant impact on the project.\
When this event occurs, the team will continue as usual and the team leader will draft an email detailing the progress of the project,
questions for the supervisor and potentially organize a replacement meeting if necessary.
5. Conflicts within the team.\
Conflicts among the team are common in projects of any scale.
This risk has a significant impact as it directly harms the team's productivity and causes delays to planned work.\
The keys to keep the problem from happening and to remedy it if it occurs are frequent communication between team members and a democratically-run leadership.
To prevent disputes from spiralling out of control, team members should voice their opinions as soon as possible, either in team meetings or mails/messages.

<div class="page"/><!-- page break -->

## Risks With Respect To The Client
|Rank|Name/Description|Occurrence Probability<br/>(H/M/L)|Severity<br/>(H/M/L)|Mitigation Strategy|Contingency|
|:-:|:-|:-:|:-:|:-|:-|
|1|Client absence.|M|M-H|Regular discussion.|Avoid making important decisions.|
|2|Client replaced.|L|H|None|Update project progress to new client.|
|3|Requirement change.|M|H|None|Re-organize project plan and negotiate with client.|

1. Temporary absence of project client.\
The project client can be temporarily absent due to illness or other commitments.
The severity level varies from moderate to high, depending on the stage of the project.\
The client's absence will not affect the project if the team regularly communicates and keeps the clients up-to-date.
When the client is away, the team will continue as usual, but will delay making important decisions until the client is consulted.
2. Client is replaced.\
The project client may leave and replaced with someone else.
This is highly unlikely given that the client works in the Factory of the Future, when the project takes place.
However, this risk, if materialized, will have serious ramifications on the project.\
There is no measures to prevent this from happening, and when it does, the team can only inform the new client with the project and
ensure that their vision is aligned with the team's.
3. Requirement change.\
Requirements can change as the project evolves, which can be problematic to the ongoing work and future plans.\
The team have little influence over project requirements, but can lessen the level of impact by maintaining consistent communication with the client and
in extreme case, negotiate the requirements.
