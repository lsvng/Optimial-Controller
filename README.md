**Introduction**
====

Control Law Theory is a branch of mathematics that focuses on the regulation of dynamic systems in engineered processes and machines. The goal is to create a model or algorithm that can apply inputs to the system to guide it towards a desired state, while reducing any negative impacts such as delay, overshoot, or steady-state error, and ensuring control stability.

The Control Law Theory operates by using the difference between the actual and desired values of a process variable as feedback to generate a control action that brings the process variable closer to the set point. In addition, control theory also includes the study of controllability and observability. The applications of control theory has a significant impact on various industries, including manufacturing, aviation, communications, and robotics, by facilitating the design of automated systems that have revolutionized these fields.

Control theory often relies on the use of block diagrams, a diagrammatic representation that simplifies the understanding of the transfer function. The transfer function is a mathematical representation of the relationship between the input and output of the system, and it is derived from the differential equations that describe the behavior of the system.

**About**
====
This project contains C++ examples on the implementation of advanced controllers, specifically designed for navigation applications. It leverages the principles of Control Law Theory to extract the gain of a system using complex mathematical formulas. 

The repository includes three popular and effective controllers:

- [PID Controller](https://en.wikipedia.org/wiki/PID_controller)
- [LQR Controller](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
- [MPC](https://en.wikipedia.org/wiki/Model_predictive_control)

The [Eigen3 Library](https://eigen.tuxfamily.org/index.php?title=Main_Page) plays a crucial role in this project by allowing for the creation of systems of equations that govern physical systems. The gain of the system is extracted while taking into account physical constraints, resulting in a stable solution with minimal iterations. This demonstrates the power of mathematical optimization techniques and their practical application in the field of navigation and control systems.
# Optimial-Controller
