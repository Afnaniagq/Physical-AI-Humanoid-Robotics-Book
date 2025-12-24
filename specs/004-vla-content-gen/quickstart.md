# Quickstart: Module 4: Vision-Language-Action (VLA) Content Generation

This guide provides the steps to review and test the generated content for the new module.

## 1. Review Generated Content

The following files will be created:

-   `frontend/docs/module-4/_category_.json`
-   `frontend/docs/module-4/10-chapter10-voice-to-action.md`
-   `frontend/docs/module-4/11-chapter11-cognitive-planning.md`
-   `frontend/docs/module-4/12-chapter12-capstone-project.md`

Please review these files for accuracy, clarity, and adherence to the requirements in the feature specification.

## 2. Run the Docusaurus Site Locally

To verify that the new content does not break the site, run the local development server.

```bash
cd frontend
npm install
npm start
```

This will start a local development server and open up a browser window. Navigate to the new module (Module 4) to see the generated chapters.

## 3. Build the Docusaurus Site

As a final verification step, run the production build command to ensure everything is working correctly.

```bash
cd frontend
npm run build
```

The build should complete successfully without any errors.
