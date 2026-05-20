# Lab 3: CSS — Styling the Site

## Introduction

HTML gives structure; CSS controls layout and appearance. In this lab you will style the three-page site from Lab 2 using one external stylesheet.

**Goal:** Style the Lab 2 site with `public/style.css`. The site must work on desktop and mobile, and the HTML must not use inline styles.

### CSS Basics

The browser applies CSS through the **cascade**. Three rules matter most in this lab:

- **Specificity**: `#id` beats `.class` beats `tag`. More specific rules win.
- **Inheritance**: text properties (color, font) flow down the tree; layout properties do not.
- **The Box Model**: every element is a rectangle — `content + padding + border + margin`.

### Specificity in Practice

Every CSS selector has a specificity score: `(IDs, classes, tags)`. When multiple rules set the same property on the same element, the highest score applies.

| Selector | IDs | Classes | Tags | Score |
|---|---|---|---|---|
| `p` | 0 | 0 | 1 | 0-0-1 |
| `.intro` | 0 | 1 | 0 | 0-1-0 |
| `p.intro` | 0 | 1 | 1 | 0-1-1 |
| `#hero` | 1 | 0 | 0 | 1-0-0 |
| `#hero p.intro` | 1 | 1 | 1 | 1-1-1 |

**Example — which rule applies?**

```html
<p id="hero" class="intro">Hello</p>
```

```css
p            { color: black; }   /* 0-0-1 */
.intro       { color: blue;  }   /* 0-1-0 */
#hero        { color: green; }   /* 1-0-0 */
```

The text is green because the `#id` selector has the highest specificity.

**Example — combining selectors raises the score:**

```css
.card a          { color: blue;   }   /* 0-1-1 */
.card .nav a     { color: orange; }   /* 0-2-1 */
```

A link inside `.card .nav` is orange because that selector has more classes.

**Example — same score: later rule applies:**

```css
.btn { background: blue; }
.btn { background: red;  }   /* same score, declared later */
```

Avoid `!important` in this lab. If a rule is not applying, inspect the element in DevTools and adjust the selector.

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

`.css` files are already handled if your Lab 2 `MIME_TYPES` dict includes `".css": "text/css"`.

Update `generate_response` to inject a `Cache-Control` header (add one line to what you wrote in Lab 2):

```python
def generate_response(content, status, mime="text/html"):
    if isinstance(content, str):
        content = content.encode()
    response_line    = f"HTTP/1.1 {status}\r\n"
    response_headers = (
        f"Content-Type: {mime}\r\n"
        f"Cache-Control: no-store, must-revalidate\r\n"
        f"Content-Length: {len(content)}\r\n\r\n"
    )
    return response_line.encode() + response_headers.encode() + content
```

Browsers cache CSS files. `Cache-Control: no-store` makes refreshes less confusing while you are editing `style.css`.

## Phase 1: Typography and Color

Open DevTools → Elements, click on your `<h1>`, and check the **Computed** panel. It shows the final CSS values after the cascade.

Start `style.css` with a reset and variables:

```css
*, *::before, *::after {
    box-sizing: border-box;
    margin: 0;
    padding: 0;
}

:root {
    --primary-color: #2c7be5;
    --text-color:    #333;
    --bg-color:      #f5f7fa;
    --font-main:     'Segoe UI', system-ui, sans-serif;
    --spacing-md:    1rem;
    --spacing-lg:    2rem;
}
```

`box-sizing: border-box` makes width calculations simpler: padding and border are included in the element's declared width.

CSS variables avoid repeating the same values. For example, `var(--primary-color)` uses the value defined in `:root`.

Now add typography rules using your variables:

```css
/* TODO: Set font-family: var(--font-main) and color: var(--text-color) on body.
   Add a background: var(--bg-color) and font-size: 16px */

/* TODO: Style h1, h2, h3 — use var(--primary-color) for color,
   pick sizes, and add a bottom margin using var(--spacing-md) */

/* TODO: Give <a> tags color: var(--primary-color) and remove the underline on hover */

/* TODO: Set a max-width of 900px on body and center it with margin: 0 auto */
```

**Check:** Save and hard-refresh (Ctrl+Shift+R). The page should use your font, colors, and link styles.

## Phase 2: The Box Model

Add spacing and borders:

```css
/* TODO: Give <header> a background color, padding: 1rem 2rem,
   and a bottom border */

/* TODO: Give <footer> a top border, padding, and center-aligned text */

/* TODO: Give <article> a right margin of 2rem */

/* TODO: Give <aside> a left border, padding-left, and a light background */

/* TODO: Give <section> elements bottom margin */

/* TODO: Give <details> padding and a border */
```

**Check:** Open DevTools → Elements and hover over `header`, `main`, `article`, `aside`, and `footer`. The highlighted boxes should match the spacing you added.

## Phase 3: Flexbox Navigation

Use Flexbox for the navigation:

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

Make `<main>` a two-column layout:

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
    flex: 0 0 250px;      /* fixed 250px, never grows or shrinks */
}
```

`flex: 1` lets the article take the remaining space. `flex: 0 0 250px` keeps the aside at 250px.

**Check:** Resize the browser window. On narrow screens the layout will be cramped; fix that in Phase 4.

## Phase 4: Responsive Design with Media Queries

A media query applies CSS only when a condition is true, such as a narrow viewport.

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

**Check:** In DevTools, toggle Device Toolbar (Ctrl+Shift+M) and set the width to 375px. The page should not require horizontal scrolling.

## Phase 5: Grid for the Projects Page

Use CSS Grid for the projects page. Replace the `<table>` from Lab 2 with a card grid:

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

`auto-fit` and `minmax(250px, 1fr)` create as many columns as fit. On narrow screens, the grid collapses to one column.

**Check:** Resize the browser window. The grid should reflow between 1, 2, and 3 columns.

## Phase 6: Form Styling

Style the contact form from Lab 2:

```css
form {
    padding: var(--spacing-lg);
}

fieldset {
    border: 1px solid #ddd;
    padding: var(--spacing-md);
}

label {
    display: block;
    margin-top: var(--spacing-md);
}

input,
textarea,
select {
    width: 100%;
    max-width: 480px;
    padding: 0.5rem;
    margin-top: 0.25rem;
}

button {
    margin-top: var(--spacing-md);
    padding: 0.6rem 1rem;
    border: 0;
    border-radius: 4px;
    background: var(--primary-color);
    color: #fff;
}
```

**Check:** Open `contact.html` at desktop and mobile widths. Labels, inputs, and the submit button should be readable and aligned.

## Submission

Submit the complete project from Labs 1, 2, and 3.

Your project must include:

1. `server.py`
2. `public/index.html`
3. `public/projects.html`
4. `public/contact.html`
5. `public/style.css`

Server requirements:

1. Starts with `uv run server.py`.
2. Listens on `localhost:8000`.
3. Serves files from `public/`.
4. Serves `/` as `public/index.html`.
5. Sends correct HTTP status lines, `Content-Type`, and `Content-Length`.
6. Includes `Cache-Control: no-store, must-revalidate`.
7. Handles missing files with `404 Not Found`.
8. Handles `POST /submit` and returns a confirmation page with the submitted name.
9. Handles multiple clients using threads, as described in Lab 1.

HTML requirements:

1. The site has three pages: Home, Projects, and Contact.
2. All pages can be reached by clicking links, starting from `http://localhost:8000`.
3. The pages use semantic HTML: `header`, `nav`, `main`, `section`, and `footer` where appropriate.
4. The contact form uses labels connected to inputs.
5. Any images have `alt` text.

CSS requirements:

1. All styling is in `public/style.css`.
2. Every HTML page links to the same stylesheet.
3. There are no `style=""` attributes in the HTML.
4. The site uses Flexbox for navigation or page layout.
5. The projects page uses CSS Grid for project cards.
6. The contact form is styled and readable.
7. The layout works at 375px and 1280px widths without horizontal scrolling.

Before submitting, test:

1. Open `http://localhost:8000`.
2. Navigate to every page using links only.
3. Submit the contact form.
4. Refresh after changing `style.css` and confirm the browser loads the new styles.
5. Open DevTools → Computed and identify one inherited property, such as `font-family` or `color`.
