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

Update `generate_response` to accept a `content_type` argument and inject a `Cache-Control` header:

```python
def generate_response(content, status, content_type="text/html"):
    if isinstance(content, str):
        content = content.encode()
    response_line    = f"HTTP/1.1 {status}\r\n"
    response_headers = (
        f"Content-Type: {content_type}\r\n"
        f"Cache-Control: no-store, must-revalidate\r\n"
        f"Content-Length: {len(content)}\r\n\r\n"
    )
    return response_line.encode() + response_headers.encode() + content
```

> **Why `Cache-Control: no-store`?**  Browsers aggressively cache `.css` files. Without this header, editing your stylesheet and refreshing the page appears to do nothing — the browser keeps serving its stale copy. This header forces it to always fetch a fresh version from your server.

## Phase 1: Typography and Color

Open DevTools → Elements, click on your `<h1>`. Notice the **Computed** panel — it shows every CSS property that ended up applied, even defaults.

Start `style.css` with a **reset** and **design tokens** — every professional stylesheet begins here:

```css
/* ── Reset ────────────────────────────────────────── */
*, *::before, *::after {
    box-sizing: border-box;
    margin: 0;
    padding: 0;
}

/* ── Design tokens ─────────────────────────────────── */
:root {
    --primary-color: #2c7be5;
    --text-color:    #333;
    --bg-color:      #f5f7fa;
    --font-main:     'Segoe UI', system-ui, sans-serif;
    --spacing-md:    1rem;
    --spacing-lg:    2rem;
}
```

> **Why `box-sizing: border-box`?**  By default, `width: 200px` means 200 px of *content only* — padding and borders push the element wider. `border-box` makes `width` include padding and borders, so the maths becomes predictable.

> **Why CSS variables?**  Instead of repeating `#2c7be5` throughout your file, you define it once and reference it with `var(--primary-color)`. To retheme the entire site, you change one line.

Now add typography rules using your variables:

```css
/* TODO: Set font-family: var(--font-main) and color: var(--text-color) on body.
   Add a background: var(--bg-color) and font-size: 16px */

/* TODO: Style h1, h2, h3 — use var(--primary-color) for color,
   pick sizes, and add a bottom margin using var(--spacing-md) */

/* TODO: Give <a> tags color: var(--primary-color) and remove the underline on hover */

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
    display: flex;
    list-style: none;
    padding: 0;
    gap: var(--spacing-md);
}

nav ul li a {
    display: block;
    padding: 0.5rem 1rem;
    border-radius: 4px;
}

nav ul li a:hover {
    background: var(--primary-color);
    color: #fff;
}
```

Now make the `<main>` section — which contains `<article>` and `<aside>` — a two-column flex layout:

```css
main {
    display: flex;
    align-items: flex-start;
    gap: var(--spacing-lg);
    padding: var(--spacing-lg);
}

article {
    flex: 1;              /* grows to fill all remaining space */
}

aside {
    flex: 0 0 250px;      /* fixed 250 px, never grows or shrinks */
}
```

> **`flex: 1` vs `flex: 0 0 250px`:**  The first value is `flex-grow`. `flex: 1` tells the article to absorb all leftover space. `flex: 0 0 250px` tells the aside to stay exactly 250 px and never participate in space distribution.

🧪 Resize the browser window. Notice the layout breaks on narrow screens. You will fix that in Phase 4.

## Phase 4: Responsive Design with Media Queries

A **media query** lets you apply styles only when a condition is true (e.g., viewport is narrow).

```css
@media (max-width: 768px) {
    main {
        flex-direction: column;   /* stack article + aside vertically */
    }

    aside {
        flex: 0 0 auto;
        width: 100%;
    }

    nav ul {
        flex-direction: column;
        align-items: center;      /* center links on narrow screens */
    }
}
```

🧪 In DevTools, toggle Device Toolbar (Ctrl+Shift+M) and set width to 375px (iPhone). The layout should adapt without horizontal scrolling.

## Phase 5: Grid for the Projects Page

CSS Grid is ideal for two-dimensional layouts. Replace the raw `<table>` on `projects.html` with a card grid. Swap the `<table>` markup for:

```html
<div class="card-grid">
    <!-- One .card div per project -->
    <div class="card">
        <h2>Project Name</h2>
        <p>Language: Python</p>
        <p>Status: Active</p>
        <a href="#" target="_blank">View →</a>
    </div>
    <!-- TODO: Add at least 2 more .card divs -->
</div>
```

Add to `style.css`:

```css
.card-grid {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
    gap: var(--spacing-md);
    padding: var(--spacing-lg);
}

.card {
    border: 1px solid #ddd;
    border-radius: 8px;
    padding: var(--spacing-md);
    background: #fff;
    box-shadow: 0 2px 4px rgba(0,0,0,.06);
    transition: transform 0.2s;
}

.card:hover {
    transform: translateY(-4px);
}
```

> **`auto-fit` + `minmax(250px, 1fr)`:**  Each column is *at least* 250 px wide. When there is room for more than one, the grid creates extra columns automatically; when the viewport is narrow, it collapses to one column — no media query needed.

🧪 Resize the browser window — the grid should automatically reflow between 1, 2, and 3 columns.

## Submission

Final checks:

1. All three pages share one stylesheet — no `style=""` attributes anywhere.
2. The layout is usable at 375px and 1280px widths.
3. Open the **Computed** tab in DevTools and find a property where inheritance is visible (a color or font flowing from `body` down to a `<p>`).

**Exploration:** Open any real website, press F12, and find a CSS custom property in the **Computed** tab. What does changing its value in DevTools do to the page?
