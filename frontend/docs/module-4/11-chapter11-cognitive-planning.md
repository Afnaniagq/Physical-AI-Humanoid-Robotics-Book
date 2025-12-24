---
id: chapter11
title: "Chapter 11: Cognitive Planning (LLMs & ROS 2)"
sidebar_position: 11
---

# Chapter 11: Cognitive Planning (LLMs & ROS 2)

This chapter explores how to use Large Language Models (LLMs) for cognitive planning in robotics, translating high-level commands into actionable steps for a robot.

## Introduction to LLMs in Robotics

- The role of LLMs as a reasoning engine for robots.
- Bridging the gap between natural language and robot actions.

## Chain-of-Robotic-Thought Prompting

- A look at the "Chain-of-Thought" prompting technique.
- Adapting this technique for robotics to create "Chain-of-Robotic-Thought".
- This involves breaking down a command into a sequence of steps that a robot can execute.

### Few-Shot Prompting for Structured Output

To ensure LLMs provide output in a format usable by robotic systems (e.g., JSON for Nav2 goals), few-shot prompting is an effective strategy. By providing examples of desired input-output pairs, the LLM learns to conform to the specified schema.

Here are three distinct examples of system prompts designed to constrain LLM output to valid JSON schemas for Nav2 goal coordinates.

**Example 1: Simple X, Y Coordinates**

```json
{
  "system_prompt": "You are a robotic planning assistant. Your goal is to convert natural language navigation commands into a JSON object containing target x and y coordinates. Respond ONLY with the JSON object. Example:\nHuman: Go to the front of the kitchen.\nAssistant: {\"x\": 5.0, \"y\": 2.5}\nHuman: Navigate to the charging station.\nAssistant: {\"x\": 1.2, \"y\": 8.7}"
}
```

**Example 2: X, Y, and Orientation (Yaw)**

```json
{
  "system_prompt": "Convert navigation requests into JSON with x, y, and yaw (in radians). Only output JSON. Example:\nHuman: Head towards the meeting room, facing the door.\nAssistant: {\"x\": 10.0, \"y\": 3.0, \"yaw\": 1.57}\nHuman: Move to the main entrance, looking outwards.\nAssistant: {\"x\": 0.5, \"y\": 0.5, \"yaw\": 3.14}"
}
```

**Example 3: Goal with Frame ID and Task ID**

```json
{
  "system_prompt": "Generate a navigation goal in JSON format, including frame_id, x, y, yaw, and a descriptive task_id. Only provide JSON. Example:\nHuman: Take me to the office, task is to deliver package.\nAssistant: {\"frame_id\": \"map\", \"x\": 7.0, \"y\": 4.0, \"yaw\": 0.0, \"task_id\": \"deliver_package_office\"}\nHuman: Move to the lab for maintenance check.\nAssistant: {\"frame_id\": \"map\", \"x\": 2.1, \"y\": 6.8, \"yaw\": -0.78, \"task_id\": \"maintenance_check_lab\"}"
}
```

### Example Prompt


```
Human: Go to the kitchen and get me an apple.

Robot Thought:
1. I need to go to the kitchen.
2. I need to find an apple.
3. I need to pick up the apple.
4. I need to bring the apple to the human.

Plan:
1. Navigate to the "kitchen" location.
2. Use the vision system to locate an "apple".
3. Use the manipulator arm to "grasp" the apple.
4. Navigate back to the "human" location.
```

### Visualizing "Chain-of-Robotic-Thought" with Mermaid.js

Mermaid.js allows for easy creation of diagrams and flowcharts using text-based syntax. This can be very useful for visualizing the complex decision-making processes of a robot's cognitive planning.

```mermaid
graph TD
    A[Voice Command] --> B{Whisper Node};
    B --> C{Transcribed Text};
    C --> D{LLM Planner};
    D -- "Chain-of-Robotic-Thought" --> E{Robot Actions/Plan (e.g., JSON)};
    E --> F{Action Execution System};
    F -- "Nav2 Goals" --> G[Robot Navigation];
    F -- "Vision System" --> H[Object Detection];
    F -- "Manipulator Control" --> I[Grasping/Manipulation];
    G --> F;
    H --> F;
    I --> F;
    F --> J[Task Complete?];
    J -- No --> D;
    J -- Yes --> K[Report Success];
```


## Mapping LLM Plans to Nav2 Goals

- Converting a high-level plan from the LLM into concrete Nav2 actions.
- Defining predefined navigation points (e.g., "kitchen", "living room").
- Sending navigation commands to Nav2.
- Handling navigation feedback and potential failures.

### Python Example for Nav2 Goal

```python
# Placeholder for rclpy Python code for sending a Nav2 goal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import tf_transformations

class Nav2Commander(Node):
    def __init__(self):
        super().__init__('nav2_commander')
        self.navigator = BasicNavigator()

    def go_to_pose(self, x, y, yaw):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        self.navigator.waitUntilNav2Active()

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        quat = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.get  # noqa: E225
            self.get_logger().info('Feedback: %s' % feedback.current_pose)
        
        result = self.navigator.getResult()
        self.get_logger().info('Result: %s' % result)

def main(args=None):
    rclpy.init(args=args)
    commander = Nav2Commander()
    commander.go_to_pose(1.0, 1.0, 0.0) # Example: Go to x=1, y=1, yaw=0
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Operational Guardrails for LLM-Generated Goals

When using LLMs to generate robot goals, it is paramount to implement robust operational guardrails. These programmatic checks ensure that the LLM's output, despite its intelligence, does not lead to unsafe or undesirable robot behavior.

### Why Guardrails?

- **LLM Hallucinations**: LLMs can generate plausible but incorrect or unsafe instructions.
- **Context Misinterpretation**: The LLM might misunderstand the robot's current state or environment.
- **Out-of-Bounds Commands**: Generated goals might be outside the robot's operational limits or designated safe zones.

### Types of Guardrails

1.  **Semantic Validation**:
    -   Check if the LLM's plan aligns with the robot's high-level mission.
    -   Example: If the mission is "deliver package," reject a plan to "dance in the hallway."

2.  **Geometric Validation (Allowed Navigation Zone)**:
    -   Programmatically verify that any LLM-generated navigation goal (e.g., x, y coordinates) falls within a predefined "allowed navigation zone" in the robot's map.
    -   Implement a polygonal check or simple bounding box check.
    -   Reject goals that are too close to obstacles, off the map, or in restricted areas.

    ```python
    # Placeholder for Python code to validate a Nav2 goal against a safe zone
    from geometry_msgs.msg import PoseStamped

    def is_goal_in_safe_zone(goal_pose: PoseStamped, safe_zone_polygon: list) -> bool:
        # safe_zone_polygon could be a list of (x, y) tuples defining a polygon
        # This is a simplified check, a real implementation would use a proper
        # point-in-polygon algorithm or collision map.
        x, y = goal_pose.pose.position.x, goal_pose.pose.position.y
        # For demonstration, assume a simple square safe zone: x between 0 and 10, y between 0 and 10
        if 0 <= x <= 10 and 0 <= y <= 10:
            return True
        return False

    # Example usage:
    # my_goal = PoseStamped(...)
    # if not is_goal_in_safe_zone(my_goal, []):
    #     self.get_logger().warn("LLM-generated goal is outside safe navigation zone!")
    #     # Implement error handling or replanning
    ```

3.  **Kinematic/Dynamic Validation**:
    -   Ensure generated movements are within the robot's physical capabilities (speed, acceleration, joint limits).

4.  **Temporal Validation**:
    -   Check if the LLM's plan is feasible within a given time constraint.

By layering these guardrails, developers can harness the power of LLMs for flexible planning while maintaining control and ensuring robot safety.
