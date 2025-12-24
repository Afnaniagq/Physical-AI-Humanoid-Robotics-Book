# Tasks: Module 4: Vision-Language-Action (VLA) Content Generation

**Input**: Design documents from `/specs/004-vla-content-gen/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [x] T001 Create directory `frontend/docs/module-4`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

(No foundational tasks for this feature)

---

## Phase 3: User Story 1 - Voice-to-Action Chapter (Priority: P1) 🎯 MVP

**Goal**: As a learner, I want to understand how to integrate OpenAI Whisper as a ROS 2 node so that I can transcribe voice commands.

**Independent Test**: A user can follow the chapter's instructions, run the Python snippets, and see transcribed text from an audio input.

### Implementation for User Story 1

- [x] T002 [US1] Create and populate `frontend/docs/module-4/_category_.json` with `{"label": "Module 4: Vision-Language-Action (VLA)", "position": 4}`
- [x] T003 [US1] Create `frontend/docs/module-4/10-chapter10-voice-to-action.md` with frontmatter (id: chapter10, title: "Chapter 10: Voice-to-Action (OpenAI Whisper)", sidebar_position: 10)
- [x] T004 [US1] Add content about OpenAI Whisper and ROS 2 integration to `frontend/docs/module-4/10-chapter10-voice-to-action.md`
- [x] T005 [US1] Add `rclpy` Python code snippets for a ROS 2 Whisper node to `frontend/docs/module-4/10-chapter10-voice-to-action.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently.

---

## Phase 4: User Story 2 - Cognitive Planning Chapter (Priority: P2)

**Goal**: As a learner, I want to learn prompt engineering for "Chain-of-Robotic-Thought" to map commands to Nav2 goals.

**Independent Test**: A user can follow the chapter's instructions to create prompts that translate a high-level command into a sequence of ROS 2 actions for Nav2.

### Implementation for User Story 2

- [x] T006 [US2] Create `frontend/docs/module-4/11-chapter11-cognitive-planning.md` with frontmatter (id: chapter11, title: "Chapter 11: Cognitive Planning (LLMs & ROS 2)", sidebar_position: 11)
- [x] T007 [US2] Add content about "Chain-of-Robotic-Thought" prompt engineering to `frontend/docs/module-4/11-chapter11-cognitive-planning.md`
- [x] T008 [US2] Add content about mapping commands to Nav2 goals to `frontend/docs/module-4/11-chapter11-cognitive-planning.md`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently.

---

## Phase 5: User Story 3 - Capstone Project Chapter (Priority: P3)

**Goal**: As a learner, I want to follow an end-to-end walkthrough that combines voice, planning, navigation, and vision to perform a "grab" task.

**Independent Test**: A user can follow the capstone project steps to create a system that takes a voice command and completes a simple "see and grab" task.

### Implementation for User Story 3

- [x] T009 [US3] Create `frontend/docs/module-4/12-chapter12-capstone-project.md` with frontmatter (id: chapter12, title: "Chapter 12: Capstone Project", sidebar_position: 12)
- [x] T010 [US3] Add content for the end-to-end walkthrough (Voice -> Plan -> Navigate -> See -> Grab) to `frontend/docs/module-4/12-chapter12-capstone-project.md`

**Checkpoint**: All user stories should now be independently functional.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [x] T011 [P] Validate Docusaurus frontmatter in all generated markdown files, ensuring titles with colons are quoted.
- [x] T012 Run `npm run build` in the `frontend` directory to verify site integrity.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)**: No dependencies.
- **User Stories (Phase 3-5)**: Depend on Setup completion. Can proceed in priority order.
- **Polish (Phase N)**: Depends on all user stories being complete.

### User Story Dependencies
- **User Story 1 (P1)**: Can start after Setup.
- **User Story 2 (P2)**: Can start after Setup.
- **User Story 3 (P3)**: Can start after Setup.

The user stories are independent content chapters and can be worked on in parallel.

## Implementation Strategy

### Incremental Delivery
1. Complete Phase 1: Setup.
2. Complete Phase 3: User Story 1 -> Test independently (MVP).
3. Complete Phase 4: User Story 2 -> Test independently.
4. Complete Phase 5: User Story 3 -> Test independently.
5. Complete Phase N: Polish.
