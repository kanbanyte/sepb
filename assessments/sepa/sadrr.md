<link rel="stylesheet" href="../styles/styles.css" type="text/css">

# Robot Vision System For A Pick And Place Task
<!--
	Co-Author: @dau501
	Editor(s): @Slothman1, @Huy-GV, @vkach, @finnmcgearey, @NickMcK14
	Year: 2023
-->

`System Architecture Design and Research Report (SADRR)`

## Industry Project 24
|Name|Position|Email|
|:-|:-|:-|
|@Slothman1|Team Leader/Client Liaison|id@swin.student.edu.au|
|@dau501|Development Manager/Planning Manager|id@swin.student.edu.au|
|@finnmcgearey|Support Manager/Developer|id@swin.student.edu.au|
|@vkach|Quality Manager/Developer|id@swin.student.edu.au|
|@NickMcK14|Support Manager/Developer|id@swin.student.edu.au|
|@Huy-GV|Quality Manager/Developer|id@swin.student.edu.au|

<div class="page"/><!-- page break -->

# DOCUMENT SIGN OFF
|Name|Position|Signature|Date|
|:-|:-|:-|:-|
|@Slothman1|Team Leader/Client Liaison|student\_signature(&emsp;)|DD/MM/2023|
|@dau501|Development Manager/Planning Manager|student\_signature(&emsp;)|DD/MM/2023|
|@finnmcgearey|Support Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|
|@vkach|Quality Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|
|@NickMcK14|Support Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|
|@Huy-GV|Quality Manager/Developer|student\_signature(&emsp;)|DD/MM/2023|

# CLIENT SIGN OFF
|Name|Position|Signature|Date|
|:-|:-|:-|:-|
|@FelipMarti|Research Fellow|<br/>|&emsp;/&emsp;/2023|

|Organisation|
|:-|
|Swinburne's Factory of the Future<br/><br/><br/><br/>|

<div class="page"/><!-- page break -->

# Introduction
Project 24 aims to enhance cobot's capabilities by providing it with a perception system to perform pick and place tasks.
The robot currently performs these tasks without any form of perception system, and the position of the object to pick and place is predefined.

Leveraging various sensors; Project 24 will utilise state-of-the-art technologies in computer vision (CV), sensors, robotics, and
artificial intelligence (AI) to develop a real-time object detection, processing, and
analysis system that will provide the robot with the ability to continuously learn and adapt to it's environment.

This document will outline the system's general design, research requirements, and project management approach for successful completion of the project.\
Finally, this document will provide sufficient information on the design of the system such that those less involved can easily understand the project,
particularly clients and project leads.

## Overview
The purpose of this document is to provide a comprehensive overview of the system, outlining its key components and functionalities.
By presenting the system in a high-level way, this document aims to give a clear understanding of how the system is intended to operate and what it is capable of.

To achieve this goal, the document will utilise a range of visual aids, including diagrams and designs, to help illustrate the system's architecture and functionality.
Through these discussions, readers will be able to gain a thorough understanding of the system's capabilities,
including its use of CV, Robot Operating System 2 (ROS2), and ML for object recognition.

While this document will cover many of the fields that will require researching by the team,
it is important to note that it will not delve into low-level designs such as class diagrams and pseudocode.
Instead, it will focus on providing a high-level overview of the system's design, leaving the detailed implementation to be addressed in subsequent documents.

Finally, it is important to recognise that this document will not include the research being conducted on specific hardware components or implementation details.
Rather, it will provide a broad overview of the system's design and functionality, leaving the specifics to be addressed in future research and implementation efforts.

<div class="page"/><!-- page break -->

## Definitions, Acronyms and Abbreviations
The following definitions will be used consistently throughout the SRS document to ensure clarity and understanding:
* **cobot:**
The mechanical device (UR5e manufactured by Universal Robots) used to perform pick and place tasks.
* **CV/Perception System:**
Computer Vision (CV) is the technology used to provide the robot with visual feedback of the environment.
* **Sensors/Depth Camera:**
The device (ZED 2 Camera developed by Stereolabs) used to detect the presence of objects and provide feedback to the robot.
* **UR:**
Universal Robots
* **AI:**
Artificial intelligence, the technology used to provide the robot with the ability to learn and improve its performance.
* **OpenCV:**
Open Source Computer Vision Library, a library of programming functions mainly aimed at real-time computer vision.
* **PyTorch:**
An open-source machine learning library based on the Torch library.
* **ROS2:**
Robot Operating System 2, a framework for building robot software applications.
* **SemVer:**
Semantic Versioning, Version Control/Software Versioning Strategy.
* **ML:**
Machine learning, A type of AI where the computer learns.
* **OS:**
Operating System.
* **PLC:**
Programmable Logic Controller
* **Pub-sub:**
publisher-subscriber
* **SADRR:**
System Architecture Design and Research Report.
* **Project 24:**
The shorter alternative name of this project.\
The full name is *Robot Vision System For A Pick And Place Task*.
* **FOF:**
Factory of the Future, a Swinburne facility where the cobot is situated.
* **HeeHooVision:**
The name for the Capstone Team working on Project 24
* **ASAP:**
As Soon as Possible
* **COB:**
Close of Business (5:00 PM)

<div class="page"/><!-- page break -->

# Problem Analysis

![robot](https://cdn.discordapp.com/attachments/1068339402289455172/1087317608962998302/image.png)

The FOF cobot lacks a perception system, which limits its capabilities to moving objects that are located through pre-programmed coordinates,
therefore reducing the efficiency for pick and place tasks.
The proposed perception system will be built with cutting edge and state-of-the-art technologies,
primarily CV through the use of OpenCV and deep ML algorithms built with PyTorch.

Working in tandem these systems will both improve the efficiency of pick and place tasks and accommodate for disruptions in required materials.
The depth camera will integrate with the cobot through a software system, this system will be primarily running of the Ubuntu 22.04 LTS OS.
Within this system ROS2 will be running and be in control of moving the robot; however, without instructions this movement wont be improved from the current iteration.

The depth camera will utilise technologies mentioned above; primarily OpenCV to analyse the data and a PyTorch algorithm to use the visual data.
The enhanced design will allow for the cobot to have error margins and autonomously accommodate them and work around them,
this definitively improving the cobots current capabilities.
Additionally, the solution must be a standalone program to replace the current Programmable Logic Controller (PLC) system.

## System Goals and Objectives
The overall goal of this project is to implement an object perception system to allow the cobot to perform its required pick and place task.
This will require the creation of a program to detect the various objects it has to manipulate, a ML algorithm for it to improve its precision, and
a system to control the cobot's movement utilising the data collected from the perception system.

The system should allow the robot to complete its tasks without user input;
it should exclusively use the Depth Camera to perceive objects and move them around the environment.
The object detection and ML software should integrate with the robotic control system, implemented with ROS2 in Ubuntu 22.04 LTS,
to provide it with object position data, allowing the cobot to move to the correct position, pick up the object, and place it in the correct position.
The required movements and object manipulations will be performed by implementing nodes, topics, services, and actions in ROS2,
which will allow for a modular software design.

Additionally, the Depth Camera should be mounted in a centralised location, giving it full field of view of the task environment with all objects in frame.

<div class="page"/><!-- page break -->

## Assumptions
When developing the system design for this project, the following assumptions were made to further narrow down the scope of the project and ensure that it was feasible:
* The system will not have to account for parts that are not in their assigned locations.\
However, the system will have to halt when an object is detected and found to obstruct the cobot in completing its task.
* The fine motor controls for the cobot is already completed and accurate.\
As such, no development will need to be made to derive and implement transfer functions.
* The tools and software's used in the project (i.e., Depth Camera, cobot, and ROS2) are not subject to change.
* The types of objects the cobot will interact with are not subject to change.
* The locations of where the cobot will have to pick up objects is not subject to change.
* The locations of where the cobot will have to place objects is not subject to change.
* Objects that are almost identical, such as the PCB parts, will have labels to distinguish between different types, such as the PCB for a barometer vs. a thermometer.

## Simplifications
To ensure that the system that is being developed is feasible and able to be completed in the allotted time,
some aspects of the problem space have been designed to simplify the developmental process.

These aspects are as follows:
* The locations of parts that that the cobot will have to pick and place are predetermined.\
Therefore, the Depth Camera does not need to 'search' for parts, rather just check if a part is in a predetermined location.
* The cobot will only be allowed to perform a pick and place task if a part is in one of the predetermined locations.
* The individual parts that the cobot will have to pick and place will be color coded,
ensuring that the Depth Camera will be able to effectively differentiate between the required parts and the surrounding environment.

<div class="page"/><!-- page break -->

# High-Level System Architecture and Alternatives
Among the software requirements of Project 24, is the integration with the existing robot control system which uses ROS2.
The architecture for the additional perception software system must be compatible with the existing ROS2 design whilst maintaining modularity and
loose-coupling between individual components.
Although the project has established that the depth camera is the main tool that retrieves image inputs,
the new perception system should ideally encapsulate this detail and communicates visual data via an abstracted format.

### Preferred Architecture
The addition of the perception system is envisioned to be encapsulated in one software module and used a by a single node in a client-server architecture.
The diagram below captures the high-level view of the system by depicting core components.
```mermaid
flowchart TD
	CobotActionClient[/Cobot Action Client/]
	Camera[/Camera Server/]
	Joint[/Cobot Movement Controller/]
	CobotAction{Cobot Action Server}

	CobotActionClient -->|pick-place-task request| CobotAction
	CobotAction -->|pick-place-task response| CobotActionClient
	Camera -->|image data| CobotAction
	CobotAction -->|object position request| Camera
	CobotAction -->|Joint Trajectories| Joint
	Joint -->|Joint Trajectories| CobotAction
```

<!-- ![hlv](https://cdn.discordapp.com/attachments/.../.../image.png) -->

This design ensures modularity by encapsulating the entire computer vision system into a single component known as the Camera Node,
enabling independent development of other components, such as the robot arm control system or the Camera node.
Additionally, individual nodes in the system can assume the server and client role interchangeably,
depending on their relationships as one server might request data from another.

The client-server architecture promotes a loosely-coupled relationship between the perception system and other related components.
This design is also consistent with the primary architecture used by ROS2, and will be further explored in the [System Architecture](#system-architecture) section.
By encapsulating features into nodes and have them interact loosely via requests and responses, we ensure any runtime fault is isolated and
does not propagate through the entire system.

If the client wishes to extend the capabilities of the robot arm beyond the scope defined in this project,
they can easily register new components to the perception node and retrieve visual data without changes to the rest of the system.

<div class="page"/><!-- page break -->

### Alternatives
#### Messaging Queue
An alternative architecture considered for this project is the messaging queue system.
In this system, visual data is pushed into a message queue and clients can asynchronously retrieve and process that data.
The message queue decouples consumers from producers and facilitates asynchronous data transfers.
However, a single queue is limited to only one consumer hence multiple queues are needed, adding overhead that can harm performance.

#### Multiple Layers
As the name suggests, the system is divided into multiple layers, each with a well-defined responsibility.
The software system in this particular project may be divided into 3 layers:
* Perception Layer, which process inputs.
* Movement Layer, which computes the desired robot movement.
* Command Layer, which translates these movements into commands.

A significant drawback of this model is the lack of flexibility as newly added components must either be fitted into an existing layer or
the system will need modifications to accommodate said components.

## System Architecture
This section explores the chosen architectural design, client-server, in greater detail by representing components, their sub-components and
explains their responsibilities and relationships with each other.
At a high level, the major components in the perception system are the **Camera Server**,
the **Cobot Action Server**, the **Gripper Server** and the **Join Trajectory Topic**.

A core component that was omitted earlier is the **Join Trajectory Topic**, which allows the cobot to retrieve trajectories from the action server, and
the **Gripper Server**, which controls the gripper of the cobot as a separate unit from the arm.
This **Join Trajectory Topic** follows the publisher-subscriber model and
requires a message broker to facilitate communication between the physical robot arm and the action server.
There are two main types of message brokers:
* Content-based: subscribers declare the properties of the type of messages they are interested in,
which is then used by the broker to filter matching messages from the publisher.
* Topic-based (**preferred**): subscribers communicates their intentions by subscribing themselves to *topics*, which represent isolated logical channels.
Each topic concentrates on a distinct type of information, enabling publishers to categorize shared data without knowing which subscribers are listening to that topic.\
This is chosen as the broker mechanism for the joint trajectories because of its support in ROS2 and
suitability with the trajectories stream from the **Cobot Action Server**.

<div class="page"/><!-- page break -->

The diagram below demonstrates the core components of the system, their interactions and relationships within a single request loop.
The cobot is designed to perform pick-and-place task in a loop which is achieved by having a client node sending requests to the action server continuously.
Note that components or subcomponents that share the same name reference the same entity,
the repetition is deliberate and intended to capture the flow of a request through the system.
```mermaid
stateDiagram-v2
	direction TB
	state fork <<fork>>
	state join <<join>>
	state "Chip" as ChipService
	state "Case" as CaseService
	state "Tray" as TrayService
	state "ZED Camera" as ZED
	state "Cobot Action Server" as Action1
	state "Cobot Action Server" as Action2
	state "Cobot Action Server" as Action3
	state "Cobot Joint Trajectory Topic" as Topic
	state "Gripper Server" as Gripper

	state "Detection Models" as Detection {
		Chip
		--
		Case
		--
		Tray
	}

	state "Position Convertors" as Convertor {
		ChipService
		--
		CaseService
		--
		TrayService
	}

	state "Camera Server Node" as Camera {
		[*] --> ZED
		ZED --> Detection
		Detection --> Convertor
		Convertor --> [*]
	}

	[*] --> Action1 : Pick Place Request
	Action1 --> Camera : Object Position Request
	Camera --> Action2 : Object Positions
	Action2 --> fork
	fork --> Gripper : Gripper Trajectories
	fork --> Topic : Joint Trajectories
	Gripper --> join
	Topic --> join
	join --> Action3
	Action3 --> [*] : Pick Place Response
```

### Components
#### Cobot Action Server
The **Cobot Action Server** (or action server) is a ROS2 node that acts as a server that performs a pick-and-place task upon a request from an action client.
The action server is responsible for coordinating different nodes and services to obtain the positions of chips, cases, and tray movements,
build a list of trajectories, and publish them to a topic that the robot arm is subscribed to or sends a request to the **Gripper Server**.
Its interaction with the **Camera Server** and **Gripper Server** are synchronous,
with the action server stopping until satisfactory responses from these nodes are received.
With regards to the **Joint Trajectory Topic**, the action server also dictates the amount of time the cobot takes to perform a move based on the trajectories.

#### Camera Server
The **Camera Server** is a ROS2 node that provides the position of chips and cases as well as suggested cobot move when requested.
Detection requests are separated by 3 different services supported by the node: tray service, chip service, and case service.
All three services capture an image with the ZED camera and feed it to the relevant AI model to retrieve bounding boxes of objects.
The chip and tray service convert these bounding boxes into numbered positions of chips (1 to 48) and trays (1 to 17), respectively.
The tray service is different, instead of returning the position and classes of trays (full, empty or partially full),
it returns a suggested command related to the trays based on their current state.
The command follows the below set of logic:
* If assembly tray is empty, move it to the empty position of either tray 1 and tray 2.
* If the assembly tray is absent, a fully loaded tray 1 or 2 is moved to the assembly position.
* If tray 1 or 2 is empty, start loading that tray.
* If tray 1 or 2 is partially loaded, continue loading that tray.

This set of moves is encoded into an enumeration with 8 values and passed as an integer in the response.
Since all three services return a single integer, they share the same service interface and
the client is expected to interpret the returned signal based on the invoked service.
In this system, the primary client of the **Camera Server** is the **Cobot Action Server**.

#### Gripper Server and Join Trajectory Topic
These two components are responsible for moving the cobot, including its joints and the gripper.
Each movement of the cobot is encoded into a list of joint trajectories i.e., coordinates indicating where each joint should be in 3D space.
Movements to pick up and move all required objects are included and can be invoked by supplying a "goal" name which represent an individual move.

The gripper is controlled via a separate ROS2 node known as the **Gripper Server**.
The node requires connection to the PLC that controls the cobot.
On the PLC, a program called 'gripper_test.urp' has been created that runs a loop to check which digital output pins are active.
When the gripper service is called, a callback which sets the pin number and activates the gripper itself is invoked.
Once the pin has been activated, the gripper will open or close as defined in 'gripper_test.urp'.
After the pin number and state has been sent, the 'ROS.urp' program is loaded played, giving control back to the PC (i.e., the **Cobot Action Server**).

## Other Alternative Architectures Explored
### Messaging Queue
A message is data of any type that needs to be transmitted and a queue in a line of messages.\
A good example of a messaging queue is and email inbox,
data can be published to the inbox then the publisher can complete other actions, without needing an immediate response.
The messages sent to the queue are held there until a client is ready to read them.\
This is shown in the diagram below:
```mermaid
stateDiagram
	%% nodes
	state "Publisher" as P
	state "Queue" as Q
	state "Client" as C
	state fork <<fork>>

	%% transitions
	P --> fork
	fork --> Q : Message 1
	fork --> Q : Message 2
	fork --> Q : Message 3
	Q --> C : Data n
```

<!-- ![queue](https://cdn.discordapp.com/attachments/1094987833174925416/1104591857696120872/image.png) -->

The client can read the published data at any time and can send a response at any time.\
This is known as decoupling where two or more systems work together without being directly connected,
enabling a program to be changed or developed without affecting existing programs.\
However, the project requires multiple programs to send requests and replies to other programs.\
This would require many queues as to implement request and reply functionality between 2 programs requires a separate queue for both the request and the reply.
The large number of queues would reduce the speed and efficiency of the project, hence why this architecture was not chosen.
Message queues are also a one-one model and have no mechanism to subscribe to a particular topic or type of message,
whereas the chosen client-service architecture use well-defined interfaces to facilitate communication, with components allowed to freely request data as needed.
These drawbacks are the reason the message queue architecture was disregarded.

<div class="page"/><!-- page break -->

### Multiple Layers
The layered architecture is defined by multiple layers that are stacked on top of each other, with each layer responsible for a different operation.
Only layers that are adjacent may communicate to one another and only the lower layers never call an upper layer.\
The upper layers call the lower layers and the lower layers can only reply to upper layer.\
This is described in the figure below:
```mermaid
flowchart TB
	%% nodes
	L1{{Layer 1}}
	L2{{Layer 2}}
	L3{{Layer 3}}
	L4{{Layer 4}}

	%% transitions
	L1 -->|Request| L2
	L2 -->|Request| L3
	L3 -->|Request| L4
	L4 --x|Reply| L3
	L3 --x|Reply| L2
	L2 --x|Reply| L1
```

<!-- ![stack](https://cdn.discordapp.com/attachments/1094987833174925416/1104591857989730336/image.png) -->

As requests must flow through multiple layers without being able to skip over a layer, the time taken to execute tasks is greater when using this architecture.
The project requires multiple programs to interact with multiple other programs so it requires that layers can be skipped.\
The layered architecture is also not very flexible and scalable.
The architecture requires a rigid structure on the flow of the program which is not desired for the dynamic requirements of the project.
Fault-tolerance is also incredibly low in the multiple layer architecture since layers are stuck together,
if something in a layer fails, it will propagate to the other layers as well.\
For these reasons this architecture was not chosen.

<div class="page"/><!-- page break -->

# Research and Investigations

![ui](https://cdn.discordapp.com/attachments/1094987833174925416/1106108495226490971/GetAttachmentThumbnail.png)

HeeHooVision conducted several research efforts to ensure the success of the project.\
These research efforts were classified into three categories:
1. Understanding The System's Business/Application Domain.
2. Exploring Similar Existing Systems.
3. Researching Technological Platforms and Programming Languages.

### Understanding The System's Business/Application Domain
HeeHooVision conducted research on the pick and place task, the FOF, and the robotics industry.\
This research aimed to provide insights into the problem at hand and the requirements for the solution.
HeeHooVision also looked into the client's needs and expectations to ensure the solution aligned with their goals.

### Exploring Similar Existing Systems
HeeHooVision conducted a thorough review of current technologies related to providing robots with vision systems to pick and place tasks.\
This research aimed to identify potential solutions and best practices that could be adopted or adapted for this project.
HeeHooVision also explored existing robotic vision systems and analysed their architecture and implementation to inform the design of the proposed system.

### Researching Technological Platforms and Programming Languages
HeeHooVision explored various software development methodologies and frameworks to find the best fit for the project.\
This research aimed to ensure that the system would be compatible with the required platforms and
that HeeHooVision had the necessary skills and resources to implement the system successfully.
HeeHooVision also researched the required hardware and software tools,
such as the Universal Robots UR5e robotic arm, ZED 2 Depth Camera, ROS2, C/C++, Python, OpenCV, and PyTorch.

### Additional Investigations and Research Efforts
HeeHooVision further covered many topics to assist with achieving the project, the topics involve other conducted research on:
* OpenCV to understand its capabilities and how it could be used for image processing in the project.
* Data collection, tagging, and organisation to ensure that the collected data was usable and accurate.
* ROS2 to understand its capabilities and how it could be used for robot programming.
* Python and C++ coding principles and best practices to ensure that the code was clean, efficient, and maintainable.
* Potential mounting options for the sensors to ensure that they were positioned correctly for accurate data collection.
* ML algorithms and techniques to explore their potential applications in the project.

HeeHooVision also conducted research into GitHub to ensure that the project was well-organised and easily accessible to team members.\
They explored different organisational strategies and established guidelines for version control and collaboration to ensure that the project ran smoothly.

<div class="page"/><!-- page break -->

## Application Domain
The application domain for the system is in the field of robotics and AI, specifically computer vision, depth sensors, and automation.

Similar programs are utilised in the field of industrial automation,
where they are used to automate product assembly on a factory floor or carry out other repetitive operations that call for accuracy and precision.
For this reason, the software solution should be useable by those without knowledge of robotics so factory floor workers can operate the system.
This can be achieved with a user-friendly interface and informative troubleshooting messages.
The perception system allows the robotic arm to see and understand its surroundings,
enabling it to carry out more difficult jobs that call for a higher level of situational awareness, allowing for more dynamic and
complex tasks to be completed whilst reducing the need for physical labour and human interaction.

The types of jobs our system can perform are limited by the specifications of the UR5e robotic arm.
It has a reach of 850mm and can deliver a maximum payload of 5kg (Universal Robots).
With these constraints the application domain for the system is limited to pick and place tasks involving relatively small components in a relatively small environment.
As the software solution is only concerned with sending control commands to the UR5e robotic arm, it is not necessarily limited to the same application domain.
The software solution could be applied to a larger or smaller robotic arm,
if its controls are similar to the UR5e in order to complete tasks outside of the UR5e robotic arm's application domain.

Overall, the program's objective is to boost productivity and efficiency while lowering the demand for human intervention in industrial operations.
The system falls under the application domain of industrial automation and robotics with a focus on computer vision and AI.

## System Design
The system will consist of a perception system, and a robotic control program.

The vision system uses a ZED 2 Depth Camera.
The Depth Camera uses an 8-element lens with optically corrected distortion and
a wider f/1.8 aperture, the Depth Camera's field of view extends to 120&deg; and is capable of recording in, 720p, 1080p and 2.2K resolutions.
The perception system will use the images recorded by the Depth Camera and AI-related libraries in Python such as PyTorch and
OpenCV to perform real-time object detection, processing, and analysis to find the position of the chips.

Training the AI will require a collection of images called a dataset.
The dataset will contain hundreds to thousands of sample images of the environment we want the artificial intelligence model to recognize.
The AI will be designed for continuous learning and adaptation to new object types.
Every photo taken after the system's implementation will be stored in the data set and will be used to further train the AI.
The perception system will pass location data to the robotic control system automatically when the systems require location data, needing no human intervention.

The software system will interact with the ROS2 and Ubuntu systems,
with ROS2 providing movement instructions for the cobot and Ubuntu being used to run ROS2 and integrate with the provided hardware.
Within the ROS2 framework, all programs are considered nodes and serve a single modular purpose.
The ROS2 nodes and the vision system nodes will communicate by actions.
In ROS2 actions allow you to execute long running tasks, provide regular feedback, and are cancellable.
They consist of three parts: a goal, feedback, and a result.
An *action client* node sends a goal to an *action server* node that acknowledges the goal and returns a stream of feedback and a result.
This will create a feedback loop allowing the perception system and the robotic system to be in constant communication.
The data sent out between nodes can be recorded and replayed using bags.
This is useful for troubleshooting issues such as actions not being performed correctly and will help identify exactly what the issue it.

The system will also prioritize non-functional requirements such as reliability, modularity, security, and performance.

<div class="page"/><!-- page break -->

## Technical Platforms, Languages and Tools
A variety of technical platforms, languages, and tools will be required to successfully complete the project.
This sections will discuss these aspects in further depth, explaining their use with respect to the project.

### Ubuntu 22.04 LTS
The client has specified that the software should run on Ubuntu 22.04 LTS, and therefore the programs will be developed on this operating system.
Additionally, because development will also be undertaken remotely, a virtual machine may also be required to be installed to use Ubuntu.
VirtualBox will be used in this case because it is easy to set up, but development at the FoF will be conducted with the provided computer.

Ubuntu makes frequent use of the terminal to perform commands and to aid with development; a good understanding of the terminal will be required.
Some basic commands that will be necessary are as follows:
* `ls` - Lists the folders and files in the current directory
* `cd [directory]` - Changes the directory to the specified directory
* `touch [file_name]` - Creates a new file
* `mkdir [directory_name]` - Makes a new directory
* `clear` - Clears the terminal screen
* `sudo apt install [package]` - `sudo` runs a command with root privileges, `apt` manages packages, and `install` installs a package
* `sudo apt update` & `sudo apt upgrade` - Updates package cache and installs the update, respectively

### ROS2
ROS2 is a collection of packages that will be used for developing the software for the robot system.
It includes various terminal commands, programming packages for C++ and Python, and data analysis tools that make development for robotics more efficient.

<div class="page"/><!-- page break -->

#### Terminal Commands
In order to access ROS2 commands in the terminal, the following command needs to be run each time a new terminal is started:
```shell
source /opt/ros/humble/setup.bash
```

To have ROS2 commands included in any terminal at startup, this command can be used:
```shell
echo "source "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

ROS2 commands are accessed by using the `ros2` command.
Some important commands include:
* `run [package] [executable]` - Runs an executable file in a package, can be used to start nodes or other ROS2 structures
* `[type] list` - Lists all of the specified type which can be: `node`, `topic`, `action`, etc.
* `[type] info` - Returns a list of subscribers, publisher, services, and actions associated with that node
* `rqt_graph` - Starts an applications that displays the ROS2 graph, containing all nodes, topics, actions, services, etc.
* `pkg create --build-type [type] [package_name]` - Creates a ROS2 package with the specified type (either `ament_cmake` or `ament_python` for C++ or Python)

#### C++ and Python Libraries
ROS2 has two client libraries for C++ and Python that will be used, `rclcpp` and `rclpy` respectively.
The client libraries share many of the same method and class names to make switching development between the two languages easier.
Nodes, including publishers, subscribers, service clients, and
service servers are created by deriving a created class from the `Node` class defined in the client library.

Rather than creating a publisher, subscriber, client, or server by creating a separate class and linking it to a node, the methods `create_publisher()`,
`create_subscriber()`, `create_client()`, or `create_server()` can be called within the Node class.

Nodes are instantiated by using the `spin()` method in the `main()` method after `init()` is called and before `shutdown()` is called.

<div class="page"/><!-- page break -->

### UR5e Collaborative Robot Arm
The UR5e collaborative robot arm (cobot) is a versatile robotic arm with a payload of 5 kg, reach of 850 mm, and 6 degrees of freedom.
The payload limit is not necessary to consider because the mass of the objects it will be picking up in this project will be far below 5 kg.

However, the reach may be something to consider to define the boundaries of the cobot so as to not move an object out of bounds, damage the cobot, or injure somebody.
The 6 degrees of freedom allows it to translate and rotate objects in 3 directions and about 3 axes, enabling it to fully manipulate objects.

### ZED 2 AI Stereo Camera
The ZED 2 camera utilises binocular vision to sense depth much like a human would, which will allow the perception system to detect objects and
their positions to a much higher precision.
It has a depth range between 0.2 m and 20 m and a 120 degree field of view giving it a wider detection area and allows for better tracking of object positions.

The ZED 2 supports multiple video resolutions and frame rates including 2.2K at 15 fps, 1080p at 30 fps or 15 fps, and 720p at 60 fps, 30 fps, or 15 fps.
Higher resolutions would allow for better object detection whereas higher frame rates would allow for better position tracking.

### OpenCV
OpenCV is a software library used for developing computer vision software.
The perception system will use OpenCV in Python to utilise the video from the ZED 2 camera to detect objects in its field of view.

### PyTorch
PyTorch is a machine learning framework that allows for development in Python.
This will be used in the perception system to improve the accuracy of the object detection by recording perception data and training the system with this data.

## Other Research
In addition to the research indicated so far, research will have to be conducted by HeeHooVision into the specific methods that AI tools, such as PyTorch and OpenCV,
may use to analyse the data received.
ML techniques that can be used to enhance and improve the performance of real-time object detection, processing and
continuous learning will have to be thoroughly researched as well.

HeeHooVision needs to research and generate relevant measures to ensure that metrics such as reliability, modularity, security, and performance are met and
maintained throughout the project.
In addition to this, mounting for the Depth Camera needs to be researched and developed.
However, preliminary considerations are that the Depth Camera will preferably be mounted to a V-slot rail system, meaning CAD designs and 3D printing may be required.

<div class="page"/><!-- page break -->

# References
* Ubuntu (2023).
Download Ubuntu desktop.
Available at: <https://ubuntu.com/download/desktop> (Accessed: 21 April 2023).
* OpenCV (2023).
OpenCV.
Available at: <https://opencv.org/> (Accessed: 21 April 2023).
* PyTorch (2023).
PyTorch.
Available at: <https://pytorch.org/> (Accessed: 21 April 2023).
* ROS 2 (2023).
ROS 2 Documentation.
Available at: <https://docs.ros.org/en/humble/index.html> (Accessed: 21 April 2023).
* Universal Robots (2023).
THE UR5e.
Available at: <https://www.universal-robots.com/products/ur5-robot/> (Accessed: 21 April 2023).
* Stereolabs (2023).
Stereolabs Documentation.
Available at: <https://www.stereolabs.com/docs/> (Accessed: 23 April 2023).
* ROS Client Library API (2022).
rclcpp: ROS Client Library for C++.
Available at: <https://docs.ros2.org/latest/api/rclcpp/> (Accessed: 24 April 2023).
* ROS Client Library API (2019).
rclpy: ROS Client Library for Python.
Available at: <https://docs.ros2.org/latest/api/rclpy/> (Accessed: 24 April 2023).
* It's FOSS (2022).
31 Linux Commands Every Ubuntu User Should Know.
Available at: <https://itsfoss.com/essential-ubuntu-commands/> (Accessed: 29 April 2023).
* Towards Data Science (2018).
Train Image Recognition AI with 5 lines of code.
Available at: <https://towardsdatascience.com/train-image-recognition-ai-with-5-lines-of-code-8ed0bdd8d9ba> (Accessed: 30 April 2023).
* Universal Robots (2018).
UR5e Technical Details.
Available at: <https://www.universal-robots.com/media/1802778/ur5e-32528_ur_technical_details_.pdf> (Accessed: 30 April 2023).
* Stereolabs (2023).
ZED 2 camera.
Available at: <https://www.stereolabs.com/zed-2/> (Accessed: 30 April 2023).
* YouTube (2021).
Module 05: Architecture, Part 05: Layered Architecture.
Available at: <https://www.youtube.com/watch?v=WiXp2p4obe4> (Accessed: 3 May 2023).
* YouTube (2022).
What is a Message Queue?.
Available at: <https://www.youtube.com/watch?v=xErwDaOc-Gs&t=649s> (Accessed: 3 May 2023).
* ROS 2 (2023).
Understanding Topics.
Available at: <https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html> (Accessed: 3 May 2023).
* Stereolabs (2023).
ROS 2 - ZED Node.
Available at: <https://www.stereolabs.com/docs/ros2/zed-node/#published-topics> (Accessed: 3 May 2023).
* Mermaid (2023).
Flowchart.
Available at: <https://mermaid.js.org/syntax/flowchart.html> (Accessed: 5 May 2023).
* Wikipedia (2023).
Flowchart.
Available at: <https://en.wikipedia.org/wiki/Flowchart> (Accessed: 5 May 2023).
* Mermaid (2023).
State Diagram.
Available at: <https://mermaid.js.org/syntax/stateDiagram.html> (Accessed: 5 May 2023).
* The Unified Modelling Language (2023).
UML Association.
Available at: <https://www.uml-diagrams.org/association.html#navigability> (Accessed: 5 May 2023).
* University of Waterloo (2017).
Layered Architecture Design Activity.
Available at: <https://cs.uwaterloo.ca/~m2nagapp/courses/CS446/1171/Arch_Design_Activity/Layered.pdf> (Accessed: 6 May 2023).
