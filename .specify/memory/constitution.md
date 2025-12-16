<!--
Sync Impact Report:
Version change: (draft) → 1.0.0
Modified principles: Content has been completely replaced with a new version based on user input.
Added sections: All sections are new as per the user's request.
Removed sections: All previous sections have been removed.
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
Follow-up TODOs: None
-->

# Physical AI & Humanoid Robotics Textbook and Integrated Learning Platform (V1.0)

**Project Goal:** Create a Docusaurus-based textbook on Embodied Intelligence and Humanoid Robotics, deployed to GitHub Pages, integrated with a full-stack RAG chatbot, and featuring chapter-level Urdu localization.

**Target Audience:** Advanced CS/Robotics Students.

### I. Core Content Principles

1.  **Focus:** Physical AI, Embodied Intelligence, and Humanoid Robotics (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA/LLMs).
2.  **Structure:** 13 Chapters + Intro/Conclusion, covering all Weekly Breakdown topics.
3.  **Style:** Academic, practical, with mandatory `Code Blocks` for ROS 2/Python and `[Image of X]` tags for complex visuals.

### II. Platform & Deployment Principles

1.  **Book Framework:** Docusaurus for static site generation.
2.  **Hosting:** Deploy to GitHub Pages.
3.  **Authoring Tools:** Use **Spec-Kit Plus** for workflow and **Claude Code** for content generation.

### III. RAG Chatbot Principles (Base Functionality)

1.  **Integration:** Embedded within the Docusaurus site.
2.  **Required Functions:**
    *   Answer Q&A based **only** on the book content.
    *   Answer Q&A based **only** on user-selected text on the page.
3.  **Technical Stack:** OpenAI Agents/ChatKit SDKs (Agent), FastAPI (Backend), Neon Serverless Postgres (Source DB), Qdrant Cloud Free Tier (Vector DB).

### IV. Localization Principle (Bonus Feature)

1.  **Requirement:** Implement a button/toggle at the **start of each chapter** for logged-in users.
2.  **Action:** Translate the chapter content **in-place to Urdu**, with the ability to revert to the original English content.

### V. Governance

All subsequent specifications, plans, and tasks must adhere to these principles and the defined Technical Stack.

---
**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15