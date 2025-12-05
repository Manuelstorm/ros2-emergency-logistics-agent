# Autonomous Robotic Agent for Emergency Logistics 🤖 📦

## 📌 Project Overview
Design and implementation of an automated planning system for an autonomous robot tasked with distributing essential goods. The system handles logical constraints, temporal planning, and operational simulation.

## 🛠️ Methodology & Activities

* **Automated Planning:** Defined the logistics domain using **PDDL**, modeling rules for robot movement, loading, and delivery.
* **Planner Development:** Developed a custom planner in **Java** (based on PDDL4J library) implementing the IDA* (Iterative Deepening A*) search algorithm. Designed a custom admissible heuristic (derived from "Adjusted Sum") to improve cost estimation and solving efficiency.
* **Temporal Optimization:** Extended the logical model to support **Temporal Planning**, managing actions with variable durations (e.g., travel time, loading operations).
* **Simulation & Integration:** Integrated the planning system into **ROS2 (Robot Operating System)** using the **PlanSys2** library. Developed **C++** components to simulate the physical execution of planned actions, bridging the gap between the logical plan and execution layer.

## 💻 Tech Stack
* **Core:** ROS2, PlanSys2
* **Languages:** Java, C++, Python
* **Planning:** PDDL, PDDL4J, IDA* Algorithm

---
*For full details, please refer to the [Project Report PDF](docs/Relazione_Progetto_AI.pdf).*
