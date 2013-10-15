Torque-Control
==============

\documentclass[10pt,a4paper,final,oneside]{article}
\usepackage[latin1]{inputenc}
\usepackage[english]{babel}
\usepackage{amsmath}
\usepackage{amsfonts}
\usepackage{amssymb}
\usepackage{listings}

\title{Contact Information and Affiliation}
\author{}
\date{}

\begin{document}
\begin{center}
{\fontsize{20pt}{25pt}\selectfont Readme}
\end{center}
\vspace{-0.1cm}
\hrule
\vspace{0.1cm}
\hfill Torque Control of a KUKA youBot Arm
\vspace{1cm}



\subsection*{How to use the torque controller}

For the torque controller, the youbot\_oodl with enable torque messages and disable gripper sensor readouts has to be running. The torque controller can then be started with 
\begin{lstlisting}[language=bash]
rosrun  torque_control  torque_control
\end{lstlisting} 
if executed from the torque\_control folder. In a launch file the following syntax has to be used:

\begin{lstlisting}[language=bash]
<launch>

<node name="torque_control" pkg="torque_control" 
      type="torque_control" cwd="node" output="screen"/>

</launch>
\end{lstlisting}

This launches an action server without feedback. As the goal, a trajectory\_msgs/JointTrajectory needs to be defined. This can either be done manually or by calling the trajectory\_generator service. What is important is that the trajectory consists of values for joint positions, velocities and acceleration for all 5 arm joints. An example of how the torque\_controller is called can be found in the torque\_example.

\subsection*{How to use the trajectory generator}

The trajectory\_generator offers 4 different services:
\begin{itemize}
\item From\_JS\_2\_JS: generates a joint space trajectory to move all the joints from the start joint space position to the desired end joint space position in the same amount of time.
\item From\_JS\_2\_CS: generates a joint space trajectory to move the gripper in a straight line from the start joint space position to the Cartesian end position.
\item From\_CS\_2\_CS: generates a joint space trajectory to move the gripper in a straight line from the Cartesian start to the Cartesian end position.
\item Circular\_Trajectory: generates a circular joint space trajectory around the center point.
\end{itemize}

If feasible trajectories for the input parameters are found, the trajectory is returned in a trajectory\_msgs/JointTrajector message. "JS" in the service name stands for joint space. It expects a fully defined "brics\_actuator::JointPositions" message. The "CS" stands for Cartesian space. It is a "geometry\_msgs::Pose" message. Examples for how the different services are called, can be found in the trajectory\_generator/tester.cpp file.
\end{document}
