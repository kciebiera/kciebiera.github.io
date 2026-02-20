# Lab 3: CSS — Making Pages Look Good

## Introduction

HTML gives structure; CSS gives appearance. In this lab you will style the three-page site from Lab 2 using a single external stylesheet. You will go from plain text to a polished layout using the box model, Flexbox, and CSS Grid.

The Goal: A visually consistent, responsive site where layout is controlled entirely by CSS — zero inline styles.

### The Theory

The browser applies styles in a predictable order called the **cascade**. Three key concepts:

- **Specificity**: `#id` beats `.class` beats `tag`. More specific rules win.
- **Inheritance**: text properties (color, font) flow down the tree; layout properties do not.
- **The Box Model**: every element is a rectangle — `content + padding + border + margin`.

```
┌──────────── margin ─────────────┐
│  ┌───────── border ──────────┐  │
│  │  ┌──── padding ────────┐  │  │
│  │  │      content        │  │  │
│  │  └─────────────────────┘  │  │
│  └───────────────────────────┘  │
└─────────────────────────────────┘
```

## Setup

Create `public/style.css`. Link it in the `<head>` of **all three** HTML files from Lab 2:

```html
<link rel="stylesheet" href="/style.css">
```

Update `read_file` in your server to also serve `.css` files with the correct `Content-Type`:

```python
def read_file(path):
    try:
        with open("public" + path, "rb") as f:
            content = f.read()
        if path.endswith(".css"):
            content_type = "text/css"
        else:
            content_type = "text/html"
        return content, "200 OK", content_type
    except FileNotFoundError:
        return b"<h1>404</h1>", "404 Not Found", "text/html"
```

(Update `generate_response` accordingly to accept a `content_type` argument.)

## Phase 1: Typography and Color

Open DevTools → Elements, click on your `<h1>`. Notice the **Computed** panel — it shows every CSS property that ended up applied, even defaults.

Add to `style.css`:

```css
/* TODO: Set a font-family on body (try: 'Segoe UI', system-ui, sans-serif)
   and a default font-size of 16px */

/* TODO: Style h1, h2, h3 — pick sizes, a color, and a bottom margin */

/* TODO: Give <a> tags a color and remove the underline on hover */

/* TODO: Set a max-width of 900px on body and center it with margin: 0 auto */
```

🧪 After each block, save and hard-refresh (Ctrl+Shift+R) to see the change.

## Phase 2: The Box Model

Add visible spacing and borders so you can *see* the box model in action.

```css
/* TODO: Give <header> a background color, padding: 1rem 2rem,
   and a bottom border */

/* TODO: Give <footer> a top border, padding, and center-aligned text */

/* TODO: Give <article> a right margin of 2rem */

/* TODO: Give <aside> a left border, padding-left, and a light background */

/* TODO: Style the projects <table>:
   - border-collapse: collapse on the table
   - padding: 0.5rem 1rem on th and td
   - a bottom border on each tr
   - a different background on thead */
```

🧪 Open DevTools → Elements, hover over any element, and watch the coloured box model overlay in the page. Identify the margin, border, and padding regions by colour.

## Phase 3: Flexbox Navigation

Flexbox makes one-dimensional layouts trivial. Restyle your `<nav>`:

```css
nav ul {
    /* TODO: display: flex */
    /* TODO: Remove default list-style and padding */
    /* TODO: gap between items */
}

nav ul li a {
    /* TODO: display: block, padding, and a hover background */
}
```

Now make the `<main>` section — which contains `<article>` and `<aside>` — a two-column flex layout:

```css
main {
    /* TODO: display: flex */
    /* TODO: align-items: flex-start so the aside doesn't stretch */
}

article {
    /* TODO: flex: 1 so it takes remaining space */
}

aside {
    /* TODO: width: 220px, flex-shrink: 0 */
}
```

🧪 Resize the browser window. Notice the layout breaks on narrow screens. You will fix that in Phase 4.

## Phase 4: Responsive Design with Media Queries

A **media query** lets you apply styles only when a condition is true (e.g., viewport is narrow).

```css
@media (max-width: 600px) {
    /* TODO: Stack main's children vertically (flex-direction: column) */

    /* TODO: Make aside full width */

    /* TODO: Make the nav links stack vertically */
}
```

🧪 In DevTools, toggle Device Toolbar (Ctrl+Shift+M) and set width to 375px (iPhone). The layout should adapt without horizontal scrolling.

## Phase 5: Grid for the Projects Page

CSS Grid is ideal for two-dimensional layouts. On `projects.html`, instead of a raw `<table>`, optionally try building the same data as a card grid.

Add a `<div class="card-grid">` wrapping several `<div class="card">` elements:

```css
.card-grid {
    /* TODO: display: grid */
    /* TODO: grid-template-columns: repeat(auto-fill, minmax(200px, 1fr)) */
    /* TODO: gap: 1rem */
}

.card {
    /* TODO: border, border-radius: 8px, padding, box-shadow */
}

.card:hover {
    /* TODO: transform: translateY(-4px), transition: transform 0.2s */
}
```

🧪 Resize the window — the grid should automatically reflow columns.

## Submission

Final checks:

1. All three pages share one stylesheet — no `style=""` attributes anywhere.
2. The layout is usable at 375px and 1280px widths.
3. Open the **Computed** tab in DevTools and find a property where inheritance is visible (a color or font flowing from `body` down to a `<p>`).

**Exploration:** Add a CSS custom property (variable): `--primary-color: #2c7be5;` on `:root`, and use `var(--primary-color)` everywhere you currently hardcode that color. Change it once, see everything update.
