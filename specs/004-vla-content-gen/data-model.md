# Data Model: Module 4: Vision-Language-Action (VLA) Content Generation

This feature involves the creation of static content files and does not have a traditional data model. The data is represented by the structure of the Docusaurus Markdown files.

## Content Entities

### `_category_.json`

This file defines the metadata for the module sidebar in Docusaurus.

-   **`label`** (string, required): The display name for the module in the sidebar. e.g., `"Module 4: Vision-Language-Action (VLA)"`
-   **`position`** (integer, required): The order of the module in the sidebar. e.g., `4`

### Chapter Markdown File (`.md`)

Each chapter is a Markdown file with the following frontmatter:

-   **`id`** (string, required): A unique identifier for the document.
-   **`title`** (string, required): The title of the chapter displayed on the page. Titles with colons must be quoted.
-   **`sidebar_position`** (integer, required): The order of the chapter within the module sidebar. (e.g., `10`, `11`, `12`)
