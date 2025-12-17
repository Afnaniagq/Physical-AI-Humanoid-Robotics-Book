---
id: 002-001
title: Isaac AI Robot Guide Spec
stage: spec
date: 2025-12-18
surface: cli
model: gemini-1.5-flash
feature: 002-isaac-ai-robot-guide
branch: 002-isaac-ai-robot-guide
user: 
command: /sp.specify "Generate a technical guide for \"Module 3: The AI-Robot Brain (NVIDIA Isaac™)\" in Markdown format.CONTEXT:- Course: Physical AI & Humanoid Robotics.- History: Students have completed ROS 2 (Mod 1) and Gazebo/Unity (Mod 2).- Objective: Transition to photorealistic simulation and GPU-accelerated perception.TASK:Provide the following in clean GitHub-Flavored Markdown:1. 3-DAY LESSON BREAKDOWN: - Day 1: Isaac Sim & Synthetic Data (Transitioning from URDF to USD; using Omniverse Replicator). - Day 2: Isaac ROS & Visual SLAM (VSLAM) (Utilizing GPU-accelerated GEMs). - Day 3: Nav2 & Bipedal Movement (3D occupancy grids with nvblox).2. CODE IMPLEMENTATION: - Provide a Python script example using 'isaac_ros_visual_slam' and 'nav2' to set a navigation goal for a humanoid.3. TECHNICAL DEEP DIVE: - Compare LiDAR-based SLAM vs. Isaac’s VSLAM in a Markdown table. Explain the superiority for humanoid \"Physical AI\" (e.g., handling camera shake/head-bobbing).4. TROUBLESHOOTING GUIDE: - List the top 3 'reality gap' issues (USD scaling, PhysX contact offsets, GPU memory management) and their fixes.FORMATTING: Use H2 and H3 headers, bold text for key terms, and proper code syntax highlighting." 
labels: [spec]
links:
  spec: specs/002-isaac-ai-robot-guide/spec.md
  ticket: 
  adr: 
  pr: 
files:
  - specs/002-isaac-ai-robot-guide/spec.md
  - specs/002-isaac-ai-robot-guide/checklists/requirements.md
tests:

---

## Prompt

Generate a technical guide for "Module 3: The AI-Robot Brain (NVIDIA Isaac™)" in Markdown format.CONTEXT:- Course: Physical AI & Humanoid Robotics.- History: Students have completed ROS 2 (Mod 1) and Gazebo/Unity (Mod 2).- Objective: Transition to photorealistic simulation and GPU-accelerated perception.TASK:Provide the following in clean GitHub-Flavored Markdown:1. 3-DAY LESSON BREAKDOWN: - Day 1: Isaac Sim & Synthetic Data (Transitioning from URDF to USD; using Omniverse Replicator). - Day 2: Isaac ROS & Visual SLAM (VSLAM) (Utilizing GPU-accelerated GEMs). - Day 3: Nav2 & Bipedal Movement (3D occupancy grids with nvblox).2. CODE IMPLEMENTATION: - Provide a Python script example using 'isaac_ros_visual_slam' and 'nav2' to set a navigation goal for a humanoid.3. TECHNICAL DEEP DIVE: - Compare LiDAR-based SLAM vs. Isaac’s VSLAM in a Markdown table. Explain the superiority for humanoid "Physical AI" (e.g., handling camera shake/head-bobbing).4. TROUBLESHOOTING GUIDE: - List the top 3 'reality gap' issues (USD scaling, PhysX contact offsets, GPU memory management) and their fixes.FORMATTING: Use H2 and H3 headers, bold text for key terms, and proper code syntax highlighting.

## Response snapshot

Generated a detailed feature specification for the "Isaac AI Robot Guide" including user scenarios, functional requirements, and success criteria. Created the `spec.md` and `checklists/requirements.md` files.

## Outcome

- ✅ Impact: Generated comprehensive feature specification.
- 🧪 Tests: (none)
- 📝 Files: spec.md, checklists/requirements.md created and updated.
- ⏭️ Next prompts: /sp.plan
- 💡 Reflection: Encountered significant challenges with PowerShell command-line parsing and string escaping for complex feature descriptions. The `-FeatureFile` approach was more robust but the `create-new-feature.ps1` script still reported a positional parameter error even when none was explicitly provided after `-FeatureFile`. This suggests a potential issue in how `create-new-feature.ps1` handles arguments or a subtle interaction with PowerShell's argument binding.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell argument parsing/escaping issues, `create-new-feature.ps1` script misinterpreting arguments.
- Graders run and results (PASS/FAIL): (none)
- Prompt variant (if applicable): (none)
- Next experiment (smallest change to try): Review `create-new-feature.ps1` for argument parsing logic when `-FeatureFile` is used.
