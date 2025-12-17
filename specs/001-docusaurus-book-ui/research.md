# Research: Docusaurus Custom Layout Implementation

**Date**: 2025-12-16

## Objective

Identify the most efficient and maintainable method to implement a custom two-column layout for the landing page and to inject custom components (like a translation button) into chapter pages within the Docusaurus framework.

## Investigation

Two primary methods were considered:

1.  **Creating a Custom Page**: This involves creating a standalone React page in the `src/pages` directory. This method provides complete control over the page's layout and content, but it disconnects the page from the Docusaurus theme and layout context.

2.  **Swizzling Theme Components**: Docusaurus' "swizzling" feature allows for replacing core theme components with custom implementations. This is the recommended approach for customizing the look and feel of the site while staying within the framework's ecosystem. For our use case, we could swizzle the `Layout` component to create our two-column structure for the homepage, and other components like `DocItem` to inject elements into chapter pages.

## Decision

**Decision**: Use the **Docusaurus "swizzle" theme component method**.

**Rationale**:
- **Maintainability**: Swizzling keeps customizations within the Docusaurus theme structure, making them easier to manage and update.
- **Integration**: This method ensures that our custom layout and components remain integrated with the Docusaurus theme context, including features like dark mode, navigation, and other global styles.
- **Flexibility**: It provides the necessary flexibility to override specific components for specific pages (like the homepage) while leaving others intact. It is the most robust way to inject components like the translation button into the `DocItem` layout.

**Alternatives Considered**:
- **Custom Page**: Rejected because it would require manually recreating much of the Docusaurus layout and context, leading to code duplication and maintenance overhead.
- **CSS-only Overrides**: Rejected as it would be too brittle and would not allow for the structural changes needed for the two-column layout or for injecting React components.
