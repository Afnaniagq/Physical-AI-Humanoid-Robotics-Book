# Tasks: Docusaurus Book UI and Navigation Implementation

**Input**: Design documents from `/specs/001-docusaurus-book-ui/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Paths shown below assume the `frontend` directory is the root for development.

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Ensure the Docusaurus project is set up correctly.

- [ ] T001 Verify that the Docusaurus project is initialized in the `frontend/` directory.
- [ ] T002 Install all necessary dependencies by running `npm install` in the `frontend/` directory.

---

## Phase 2: User Story 1 - Landing Page Experience (Priority: P1) 🎯 MVP

**Goal**: Create a clear and inviting landing page with a two-column layout.

**Independent Test**: The landing page can be deployed and tested to ensure the layout, thematic image, and navigation elements are present and functional, even without any chapter content.

### Implementation for User Story 1

- [ ] T003 [US1] Create the custom layout component file `frontend/src/theme/CustomLayout.js`.
- [ ] T004 [US1] Implement the two-column layout structure (e.g., using CSS Flexbox or Grid) within `frontend/src/theme/CustomLayout.js`.
- [ ] T005 [P] [US1] Create the thematic image component `frontend/src/components/ThematicImage.js`.
- [ ] T006 [P] [US1] Create a placeholder component for the Book Title, e.g., `frontend/src/components/BookTitle.js`.
- [ ] T007 [P] [US1] Create a placeholder component for Feature Buttons (Login/Auth, Translation), e.g., `frontend/src/components/FeatureButtons.js`.
- [ ] T008 [US1] Integrate the `ThematicImage`, `BookTitle`, and `FeatureButtons` components into `frontend/src/theme/CustomLayout.js`.

**Checkpoint**: User Story 1 should be functional. The landing page should display a two-column layout with all the placeholder components.

---

## Phase 3: User Story 2 - Start the Course (Priority: P1)

**Goal**: Allow users to start the course with a single click from the landing page.

**Independent Test**: The "Start Course" button can be tested to ensure it navigates to the correct URL for the first chapter.

### Implementation for User Story 2

- [ ] T009 [P] [US2] Create the `StartCourseButton` component in `frontend/src/components/StartCourseButton.js`.
- [ ] T010 [US2] Add the `StartCourseButton` component to the `frontend/src/theme/CustomLayout.js`.
- [ ] T011 [US2] Configure the button's `onClick` handler or `href` to navigate to the first chapter of Module 1.

**Checkpoint**: User Stories 1 and 2 should now be complete and testable.

---

## Phase 4: User Story 3 - Chapter-Level UI Elements (Priority: P2)

**Goal**: Support localization by adding a UI element for translation to each chapter.

**Independent Test**: A sample chapter page can be checked to verify that the UI element for the translation toggle is present and correctly positioned.

### Implementation for User Story 3

- [ ] T012 [P] [US3] Create a placeholder component for the "Localization Principle" button, e.g., `frontend/src/components/TranslationToggle.js`.
- [ ] T013 [US3] Swizzle the `DocItem` component to create `frontend/src/theme/DocItem/index.js`.
- [ ] T014 [US3] Inject the `TranslationToggle` component into the swizzled `DocItem` component at the top of the content area.

**Checkpoint**: All user stories should now be implemented.

---

## Phase 5: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [ ] T015 Apply styling to all new components (`ThematicImage`, `StartCourseButton`, etc.) to match the "modern, academic, and clean aesthetic".
- [ ] T016 Implement the responsive design for the landing page, ensuring the two columns stack on mobile and tablet devices.
- [ ] T017 Implement the fallback behavior for the thematic image (a solid background color) in `frontend/src/components/ThematicImage.js`.
- [ ] T018 Run `quickstart.md` validation to ensure the project runs as expected.

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: Must be completed first.
- **User Stories (Phases 2-4)**: Can begin after Setup. User stories are largely independent and can be worked on in parallel.
- **Polish (Phase 5)**: Depends on the completion of all user stories.

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies on other stories.
- **User Story 2 (P1)**: Depends on User Story 1 (the landing page layout).
- **User Story 3 (P2)**: No dependencies on other stories.

### Parallel Opportunities

- Within User Story 1, tasks T005, T006, and T007 can be done in parallel.
- User Story 3 can be worked on in parallel with User Stories 1 and 2.

## Implementation Strategy

### MVP First (User Stories 1 & 2)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: User Story 1.
3.  Complete Phase 3: User Story 2.
4.  **STOP and VALIDATE**: Test the complete landing page experience.
5.  Deploy/demo the MVP.
