**Introduction**
====

Control Law Theory is a branch of mathematics that focuses on the regulation of dynamic systems in engineered processes and machines. The goal is to create a model or algorithm that can apply inputs to the system to guide it towards a desired state, while reducing any negative impacts such as delay, overshoot, or steady-state error, and ensuring control stability.

The Control Law Theory operates by using the difference between the actual and desired values of a process variable as feedback to generate a control action that brings the process variable closer to the set point. In addition, control theory also includes the study of [controllability](https://en.wikipedia.org/wiki/Controllability) and [observability](https://en.wikipedia.org/wiki/Observability) to design and optimize control systems to meet specific performance criteria and ensure their reliability and safety. The applications of control theory has a significant impact on various industries, including manufacturing, aviation, communications, and robotics, by facilitating the design of automated systems that have revolutionized these fields.

Control theory often relies on the use of block diagrams, a diagrammatic representation that simplifies the understanding of the transfer function. The transfer function is a mathematical representation of the relationship between the input and output of the system, and it is derived from the differential equations that describe the behavior of the system.

**About**
====
This project includes C++ examples that demonstrate the implementation of advanced controllers. It utilizes Control Law Theory to generate control actions and leverages the [Eigen3 Library](https://eigen.tuxfamily.org/index.php?title=Main_Page) to construct transfer functions that govern the system and perform matrix operations.

This repository contains the following popular and effective controllers that have been extensively tested and proven to deliver outstanding performance:

- [Fuzzy Logic Controller](https://en.wikipedia.org/wiki/Fuzzy_control_system)
- [PID Controller](https://en.wikipedia.org/wiki/PID_controller)
- [LQR Controller](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator)
- [LQG Controller](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic%E2%80%93Gaussian_control)
- [MPC](https://en.wikipedia.org/wiki/Model_predictive_control)

Each of these controllers has been selected for its unique features and capabilities, making them an ideal choice for a variety of applications.
