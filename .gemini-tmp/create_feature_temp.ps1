$featureDescription = @"
Generate a technical guide for "Module 3: The AI-Robot Brain (NVIDIA Isaac™)" in Markdown format.
CONTEXT:
- Course: Physical AI & Humanoid Robotics.
- History: Students have completed ROS 2 (Mod 1) and Gazebo/Unity (Mod 2).
- Objective: Transition to photorealistic simulation and GPU-accelerated perception.
TASK:
Provide the following in clean GitHub-Flavored Markdown:
1. 3-DAY LESSON BREAKDOWN: - Day 1: Isaac Sim & Synthetic Data (Transitioning from URDF to USD; using Omniverse Replicator). - Day 2: Isaac ROS & Visual SLAM (VSLAM) (Utilizing GPU-accelerated GEMs). - Day 3: Nav2 & Bipedal Movement (3D occupancy grids with nvblox).
2. CODE IMPLEMENTATION: - Provide a Python script example using 'isaac_ros_visual_slam' and 'nav2' to set a navigation goal for a humanoid.
3. TECHNICAL DEEP DIVE: - Compare LiDAR-based SLAM vs. Isaac’s VSLAM in a Markdown table. Explain the superiority for humanoid "Physical AI" (e.g., handling camera shake/head-bobbing).
4. TROUBLESHOOTING GUIDE: - List the top 3 'reality gap' issues (USD scaling, PhysX contact offsets, GPU memory management) and their fixes.
FORMATTING: Use H2 and H3 headers, bold text for key terms, and proper code syntax highlighting.
"@

& .specify/scripts/powershell/create-new-feature.ps1 -Number 1 -ShortName "isaac-ai-robot-guide" -FeatureDescription $featureDescription