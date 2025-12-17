---
id: 0001
title: Update Project Constitution
stage: constitution
date: 2025-12-15
surface: agent
model: Gemini
feature: none
branch: main
user: unknown
command: /sp.constitution
labels: [constitution, project-governance]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
 - .claude/commands/sp.plan.md
tests:
 - none
---

## Prompt

/sp.constitution  **write the entier output using standard Markdown formatting (heading,list,bolding) suitable for direct saving to constitution.md**                                                                          Physical AI & Humanoid Robotics Textbook and Integrated Learning Platform (V1.0)Project Goal: Create a Docusaurus-based textbook on Embodied Intelligence and Humanoid Robotics, deployed to GitHub Pages, integrated with a full-stack RAG chatbot, and featuring chapter-level Urdu localization.Target Audience: Advanced CS/Robotics Students.### I. Core Content Principles1.  **Focus:** Physical AI, Embodied Intelligence, and Humanoid Robotics (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA/LLMs).2.  **Structure:** 13 Chapters + Intro/Conclusion, covering all Weekly Breakdown topics.3.  **Style:** Academic, practical, with mandatory `Code Blocks` for ROS 2/Python and `[Image of X]` tags for complex visuals (e.g., , ).### II. Platform & Deployment Principles1.  **Framework:** Docusaurus for static site generation.2.  **Hosting:** Deploy to GitHub Pages.3.  **Authoring Tools:** Use **Spec-Kit Plus** for workflow and **Claude Code** for content generation.### III. RAG Chatbot Principles (Base Functionality)1.  **Integration:** Embedded within the Docusaurus site.2.  **Required Functions:**    * Answer Q&A based **only** on the book content.    * Answer Q&A based **only** on user-selected text on the page.3.  **Technical Stack:** OpenAI Agents/ChatKit SDKs (Agent), FastAPI (Backend), Neon Serverless Postgres (Source DB), Qdrant Cloud Free Tier (Vector DB).### IV. Localization Principle (Bonus Feature)1.  **Requirement:** Implement a button/toggle at the **start of each chapter** for logged-in users.2.  **Action:** Translate the chapter content **in-place to Urdu**, with the ability to revert to the original English content.### V. GovernanceAll subsequent specifications, plans, and tasks must adhere to these principles and the defined Technical Stack.

## Response snapshot

Successfully updated the project constitution to v1.0.0 and verified consistency with project templates.

## Outcome

- ✅ Impact: The project constitution has been updated to reflect the new principles and governance model.
- 🧪 Tests: No tests were run.
- 📁 Files: Updated .specify/memory/constitution.md and .claude/commands/sp.plan.md
- 🔁 Next prompts: Suggest running /sp.plan to create a plan for a new feature.
- 🧠 Reflection: The agent successfully updated the constitution and performed a consistency check, identifying and fixing an outdated agent-specific command.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): none
