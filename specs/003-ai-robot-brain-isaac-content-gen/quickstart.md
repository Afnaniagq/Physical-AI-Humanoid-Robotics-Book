# Quickstart Guide: Module 3: The AI-Robot Brain (NVIDIA Isaac™) Content Generation

**Feature**: 003-ai-robot-brain-isaac-content-gen
**Date**: 2025-12-17

This quickstart guide is for content developers contributing to Module 3 (The AI-Robot Brain) of the Physical AI & Humanoid Robotics textbook.

## 1. Access the Feature Branch

First, ensure you are on the correct feature branch:

```bash
git checkout 003-ai-robot-brain-isaac-content-gen
```

## 2. Locate Content Files

The primary content for Module 3 is located in:

```
frontend/docs/module3/
```

You will find the following files:

-   `_category_.json`: Defines the module's title and position in the sidebar.
-   `07-chapter7.md`: Chapter on Isaac Sim and Synthetic Data.
-   `08-chapter8.md`: Chapter on Isaac ROS and Perception.
-   `09-chapter9.md`: Chapter on Nav2 for Bipedal Robots.

## 3. Set up Development Environment

To preview your changes locally, navigate to the `frontend` directory and start the Docusaurus development server:

```bash
cd frontend
npm install
npm start
```

This will open your browser to `http://localhost:3000` (or similar), allowing you to see your content rendered in real-time.

## 4. Content Authoring Guidelines

-   **Format**: All content must be written in Docusaurus-compatible Markdown (`.md`).
-   **Structure**: Follow the outline provided in `specs/003-ai-robot-brain-isaac-content-gen/plan.md`.
-   **Code Blocks**:
    *   Use triple backticks for code blocks. Specify the language (e.g., `bash`, `python`, `yaml`, `json`).
    *   Ensure all ROS 2 commands, YAML configurations, and Python/USD scripts are technically accurate and runnable.
-   **Image Placeholders**: Use `[Image of <description>]` for all visual assets.
-   **Cross-references**: Link to other relevant chapters (e.g., Module 1 for ROS 2 basics, Module 2 for Digital Twins) using Docusaurus's linking syntax.
-   **`sidebars.js`**: If you add new chapters or significant sections, you may need to update `frontend/sidebars.js` to ensure proper navigation.

## 5. Technical Validation

-   **ROS 2 Commands**: Test commands in a ROS 2 Humble/Iron environment.
-   **Python/USD Scripts**: Verify functionality within Isaac Sim 2023.1+.
-   **YAML Configurations**: Validate Nav2 configurations.

## 6. Review & Commit

Before committing your changes:

-   **Proofread**: Check for grammatical errors, typos, and technical inaccuracies.
-   **Docusaurus Build**: Ensure the Docusaurus site builds without errors locally.
-   **Git Add & Commit**:
    ```bash
    git add frontend/docs/module3/ frontend/sidebars.js
    git commit -m "feat(module3): Add initial draft of Chapter X"
    ```
