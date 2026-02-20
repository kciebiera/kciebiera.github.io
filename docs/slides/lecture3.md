---
marp: true
theme: default
paginate: true
style: |
  section { font-size: 1.4rem; }
  pre { font-size: 0.85rem; }
  h1 { color: #0a7c59; }
  h2 { color: #0fa87a; }
  .columns { display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; }
  code { background: #f5f5f5; padding: 0.1em 0.3em; border-radius: 3px; }
---

# Lecture 3
## CSS — Making the Web Look Good

**WWW 25/26**
Cascade · Box Model · Flexbox · Grid · Responsive Design

---

# What is CSS?

**Cascading Style Sheets** — the language that controls the *appearance* of HTML.

HTML answers: **what is this?**  
CSS answers: **how should it look?**

```css
p {
  color: navy;
  font-size: 1.2rem;
  line-height: 1.6;
}
```

A CSS **rule** = a **selector** (which elements) + a **declaration block** (what to change).  
A **declaration** = a **property** + a **value**, separated by `:`.

---

# Three Ways to Add CSS

**1. External stylesheet (always preferred):**
```html
<link rel="stylesheet" href="/static/style.css">
```

**2. Embedded in `<style>` tag:**
```html
<head>
  <style>
    p { color: red; }
  </style>
</head>
```

**3. Inline style (avoid except for dynamic JS):**
```html
<p style="color: red;">Text</p>
```

**Rule:** Always use external stylesheets. Inline styles are impossible to override and make maintenance a nightmare.

---

# The Cascade

CSS = **C**ascading Style Sheets — "cascading" means rules from multiple sources combine.

Sources in **ascending priority order** (last wins):
1. Browser default stylesheet (user-agent styles)
2. External stylesheets (`<link>`)
3. `<style>` block in `<head>`
4. Inline `style=""` attribute
5. `!important` declarations

```css
p { color: blue; }         /* lower priority */
p { color: red; }          /* same specificity: last rule wins */
```

When two rules have the **same specificity**, the one that appears **later** in the source wins.

---

# Specificity

When two rules target the same element, **specificity** decides which wins.

| Selector type | Points |
|--------------|--------|
| Inline style | 1,0,0,0 |
| `#id` | 0,1,0,0 |
| `.class`, `[attr]`, `:pseudo-class` | 0,0,1,0 |
| `element`, `::pseudo-element` | 0,0,0,1 |
| `*` (universal) | 0,0,0,0 |

```css
p              { color: black; }    /* 0,0,0,1 */
.intro         { color: blue;  }    /* 0,0,1,0 — wins */
#main p        { color: green; }    /* 0,1,0,1 — wins over .intro */
```

Higher points win; points don't "carry over" across columns.

---

# Inheritance

Some CSS properties **inherit** — child elements get the parent's value automatically.

```css
body { font-family: sans-serif; color: #333; }
/* Every element inherits font-family and color */
```

**Inherited by default:** `color`, `font-*`, `line-height`, `text-align`, `cursor`

**Not inherited:** `background`, `border`, `margin`, `padding`, `width`, `display`

You can force inheritance:
```css
button { font-family: inherit; }   /* buttons don't inherit by default */
```

---

# CSS Selectors — The Basics

```css
/* Type selector — all paragraphs */
p { }

/* Class selector — all elements with class="intro" */
.intro { }

/* ID selector — the element with id="hero" */
#hero { }

/* Universal selector — everything */
* { }

/* Attribute selector */
input[type="email"] { }
a[href^="https"] { }    /* href starts with "https" */
a[href$=".pdf"]  { }    /* href ends with ".pdf" */
```

---

# Combinators

```css
/* Descendant: any <p> inside .container (any depth) */
.container p { }

/* Child: direct <p> children of .container only */
.container > p { }

/* Adjacent sibling: <p> immediately after an <h2> */
h2 + p { }

/* General sibling: all <p> after an <h2> (same parent) */
h2 ~ p { }
```

```html
<div class="container">
  <p>← .container p matches this</p>
  <section>
    <p>← .container p matches this too (descendant)</p>
    <p>← .container > p does NOT match this</p>
  </section>
</div>
```

---

# Pseudo-classes

Pseudo-classes select elements based on **state** or **position**.

```css
a:hover  { text-decoration: underline; }   /* mouse over it */
a:focus  { outline: 2px solid blue; }      /* keyboard focused */
a:active { opacity: 0.7; }                 /* being clicked */
a:visited { color: purple; }

input:invalid { border-color: red; }       /* fails HTML validation */
input:required { border-left: 3px solid orange; }

li:first-child  { font-weight: bold; }
li:last-child   { margin-bottom: 0; }
li:nth-child(2) { background: #f0f0f0; }
li:nth-child(odd) { background: #fafafa; }
```

---

# Pseudo-elements

Pseudo-elements let you style **parts** of an element or inject content.

```css
/* First letter of every paragraph */
p::first-letter {
  font-size: 2em;
  float: left;
}

/* First line of every paragraph */
p::first-line { font-weight: bold; }

/* Insert content before/after — no HTML needed */
.required::after {
  content: " *";
  color: red;
}

/* Selected text */
::selection { background: yellow; }
```

---

# The Box Model

**Every element is a rectangular box:**

```
┌───────────────── margin ──────────────────┐
│  ┌──────────────── border ──────────────┐  │
│  │  ┌─────────── padding ─────────────┐ │  │
│  │  │                                 │ │  │
│  │  │         content area            │ │  │
│  │  │                                 │ │  │
│  │  └─────────────────────────────────┘ │  │
│  └──────────────────────────────────────┘  │
└────────────────────────────────────────────┘
```

- **content** — where text and child elements live
- **padding** — space inside the border (has the element's background)
- **border** — visible line around the padding
- **margin** — space outside the border (transparent)

---

# Box Model in CSS

```css
div {
  width: 300px;
  height: 150px;

  padding: 20px;             /* all four sides */
  padding: 10px 20px;        /* top/bottom  left/right */
  padding: 10px 20px 30px 40px;  /* top right bottom left */

  border: 2px solid #333;
  border-radius: 8px;        /* rounded corners */

  margin: 1rem auto;         /* top/bottom=1rem, left/right=auto (centres) */
}
```

**Gotcha:** By default, `width` refers to the content area only. Padding and border are **added on top**.

---

# `box-sizing: border-box`

The default (`content-box`) is confusing:

```css
/* content-box: total width = 300 + 20*2 + 2*2 = 344px */
div {
  width: 300px;
  padding: 20px;
  border: 2px solid black;
}
```

**Fix:** Use `border-box` globally — width includes padding and border:

```css
*, *::before, *::after {
  box-sizing: border-box;
}
/* Now: width: 300px means the total rendered width is 300px */
```

This is the first rule in almost every real-world stylesheet.

---

# The `display` Property

```css
/* Block: starts on new line, takes full width */
display: block;

/* Inline: flows with text, no width/height */
display: inline;

/* Inline-block: flows with text, but respects width/height */
display: inline-block;

/* Flexbox container — children become flex items */
display: flex;

/* Grid container — children become grid items */
display: grid;

/* Hide element completely (removed from layout) */
display: none;
```

---

# Colors

```css
/* Named */
color: red;
color: cornflowerblue;

/* Hex (most common) */
color: #ff0000;       /* red */
color: #f00;          /* shorthand for #ff0000 */
color: #2c7be5;

/* RGB */
color: rgb(44, 123, 229);
color: rgba(44, 123, 229, 0.5);   /* with alpha (transparency) */

/* HSL — Hue Saturation Lightness (most intuitive to tweak) */
color: hsl(214, 77%, 54%);
color: hsla(214, 77%, 54%, 0.8);

/* Modern: hsl without comma */
color: hsl(214 77% 54% / 80%);
```

---

# Typography

```css
body {
  font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
  font-size: 16px;          /* base size */
  font-weight: 400;         /* 100–900; 400=normal, 700=bold */
  font-style: italic;
  line-height: 1.6;         /* unitless: 1.6 × font-size */
}

h1 {
  font-size: 2.5rem;        /* 2.5 × root font-size */
  letter-spacing: -0.02em;  /* tight for large headings */
  text-transform: uppercase;
}

p {
  text-decoration: underline;
  text-align: justify;
  max-width: 65ch;           /* ch = width of "0" — great for readability */
}
```

---

# CSS Units

| Unit | Meaning | Use when |
|------|---------|----------|
| `px` | Screen pixels | Borders, shadows, images |
| `%` | Relative to parent | Widths, heights in flow |
| `em` | Relative to element's own `font-size` | Padding, margin around text |
| `rem` | Relative to **root** `font-size` | Font sizes, spacing |
| `vw` | 1% of viewport **width** | Hero sections, full-width |
| `vh` | 1% of viewport **height** | Full-screen sections |
| `ch` | Width of "0" character | Text column widths |

**Recommendation:** Use `rem` for font sizes, `em` or `rem` for spacing. Avoid `px` for font sizes (respects user's browser zoom).

---

# Positioning

```css
/* Default: in document flow */
position: static;

/* Offset from its normal position; still in flow */
position: relative;
top: 10px; left: 20px;

/* Removed from flow; positioned relative to nearest
   non-static ancestor */
position: absolute;

/* Like absolute but relative to the viewport */
position: fixed;

/* Normal flow, but sticks when scrolled past */
position: sticky;
top: 0;    /* sticks to top when scrolling */
```

---

# `z-index` and Stacking

When elements overlap, `z-index` controls which is on top.

```css
.modal-backdrop {
  position: fixed;
  z-index: 100;
}

.modal {
  position: fixed;
  z-index: 101;   /* modal appears above backdrop */
}

.tooltip {
  position: absolute;
  z-index: 200;   /* tooltips above modals */
}
```

`z-index` only works on positioned elements (`relative`, `absolute`, `fixed`, `sticky`). Has no effect on `static`.

---

# Flexbox — The Concept

**Flexbox** solves one-dimensional layout: a row **or** a column.

```css
.container {
  display: flex;     /* makes children into flex items */
}
```

```
FLEX CONTAINER
┌──────────────────────────────────────────┐
│  ┌──────┐  ┌──────┐  ┌──────┐  ┌──────┐ │  ← main axis (row)
│  │item 1│  │item 2│  │item 3│  │item 4│ │
│  └──────┘  └──────┘  └──────┘  └──────┘ │
└──────────────────────────────────────────┘
                                    ↕ cross axis
```

The **main axis** runs along the `flex-direction`.  
The **cross axis** is perpendicular.

---

# Flexbox — Container Properties

```css
.container {
  display: flex;

  /* Direction of main axis */
  flex-direction: row;            /* default */
  flex-direction: column;
  flex-direction: row-reverse;

  /* Alignment on main axis */
  justify-content: flex-start;   /* default */
  justify-content: center;
  justify-content: space-between;
  justify-content: space-around;
  justify-content: flex-end;

  /* Alignment on cross axis */
  align-items: stretch;          /* default */
  align-items: center;
  align-items: flex-start;

  /* Wrap to next line if overflow */
  flex-wrap: wrap;

  gap: 1rem;                     /* space between items */
}
```

---

# Flexbox — Item Properties

```css
.item {
  /* How much does this item grow relative to siblings? */
  flex-grow: 1;      /* 0 = don't grow (default) */

  /* How much can this item shrink? */
  flex-shrink: 0;    /* 0 = don't shrink */

  /* Starting size before grow/shrink */
  flex-basis: 200px;

  /* Shorthand: grow shrink basis */
  flex: 1 0 200px;
  flex: 1;            /* 1 1 0% — common: "take equal share" */

  /* Override align-items for this item only */
  align-self: center;

  /* Reorder visually (does not change DOM order) */
  order: 2;
}
```

---

# Flexbox — Common Patterns

```css
/* Navigation: items spread across the bar */
nav { display: flex; justify-content: space-between; align-items: center; }

/* Sidebar layout: sidebar fixed, content grows */
main { display: flex; gap: 2rem; }
aside { flex: 0 0 240px; }    /* don't grow or shrink */
article { flex: 1; }          /* take remaining space */

/* Center anything (horizontally + vertically) */
.hero {
  display: flex;
  justify-content: center;
  align-items: center;
  min-height: 100vh;
}

/* Card row that wraps to next line */
.cards { display: flex; flex-wrap: wrap; gap: 1rem; }
.card  { flex: 1 1 250px; }   /* minimum 250px, then grow */
```

---

# CSS Grid — The Concept

**CSS Grid** solves two-dimensional layout: rows **and** columns simultaneously.

```css
.container {
  display: grid;
  grid-template-columns: 250px 1fr 1fr;  /* 3 columns */
  grid-template-rows: auto 1fr auto;     /* 3 rows */
  gap: 1rem;
}
```

```
┌─────────┬─────────────┬─────────────┐
│ sidebar │    main     │    extra    │
│ 250px   │    1fr      │    1fr      │
├─────────┴─────────────┴─────────────┤
│              footer                  │
└─────────────────────────────────────┘
```

`1fr` = "one fraction of available space" — like `flex: 1`.

---

# CSS Grid — Placing Items

```css
/* Items automatically fill cells left-to-right, top-to-bottom.
   Override with explicit placement: */

.header {
  grid-column: 1 / -1;    /* span from col 1 to last column */
  grid-row: 1;
}

.sidebar {
  grid-column: 1;
  grid-row: 2 / 4;         /* span rows 2–3 */
}

.main {
  grid-column: 2 / -1;
  grid-row: 2;
}

/* Named areas (more readable) */
grid-template-areas:
  "header header header"
  "sidebar main   main  "
  "footer  footer footer";
```

---

# CSS Grid — auto-fill and auto-fit

These let the browser decide how many columns to create:

```css
/* Create as many 200px columns as will fit */
.gallery {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(200px, 1fr));
  gap: 1rem;
}
```

```
viewport 800px → 4 columns of ~175px
viewport 500px → 2 columns of ~225px
viewport 300px → 1 column
```

**`auto-fill`** vs **`auto-fit`**: `auto-fit` collapses empty tracks; `auto-fill` keeps them. With `minmax` the difference rarely matters.

---

# When to Use Flexbox vs Grid

| Use | Flexbox | Grid |
|-----|---------|------|
| Navigation bar | ✅ | |
| Row of buttons | ✅ | |
| Centring content | ✅ | |
| Page layout (header/sidebar/content/footer) | | ✅ |
| Photo gallery | | ✅ |
| Card grid with equal heights | | ✅ |
| Any 2D layout | | ✅ |
| Stacking items in one direction | ✅ | |

They are **complementary**, not competing. A Grid can contain Flex items, and vice versa.

---

# Responsive Design

A responsive site works well on any screen size.

**Three techniques:**

1. **Fluid layout** — use `%`, `fr`, `max-width` instead of fixed `px`
2. **Media queries** — apply different styles at different viewport widths
3. **Flexible images** — `img { max-width: 100%; height: auto; }`

The **viewport meta tag** in HTML is mandatory:
```html
<meta name="viewport" content="width=device-width, initial-scale=1.0">
```
Without it, mobile browsers pretend they have a 980px screen and scale down.

---

# Media Queries

```css
/* Applies only when viewport ≤ 600px */
@media (max-width: 600px) {
  main { flex-direction: column; }
  aside { width: 100%; }
  nav ul { flex-direction: column; }
}

/* Applies only when viewport ≥ 1024px */
@media (min-width: 1024px) {
  .container { max-width: 1200px; }
}

/* Combine conditions */
@media (min-width: 600px) and (max-width: 1024px) {
  /* tablet only */
}

/* Dark mode preference */
@media (prefers-color-scheme: dark) {
  body { background: #1a1a1a; color: #f0f0f0; }
}
```

---

# Mobile-First Approach

Write **base styles for mobile** first, then add complexity for larger screens with `min-width` queries.

```css
/* Mobile styles (default) */
.card-grid {
  display: grid;
  grid-template-columns: 1fr;   /* 1 column */
  gap: 1rem;
}

/* Tablet and up */
@media (min-width: 640px) {
  .card-grid { grid-template-columns: 1fr 1fr; }
}

/* Desktop and up */
@media (min-width: 1024px) {
  .card-grid { grid-template-columns: repeat(3, 1fr); }
}
```

Mobile-first keeps the baseline simple. `max-width` overrides get messy quickly.

---

# CSS Custom Properties (Variables)

Variables store values you reuse across many rules.

```css
:root {
  --color-primary:    #2c7be5;
  --color-background: #ffffff;
  --color-text:       #1a1a2e;
  --spacing-base:     1rem;
  --border-radius:    8px;
  --font-sans:        'Segoe UI', system-ui, sans-serif;
}

button {
  background: var(--color-primary);
  border-radius: var(--border-radius);
  padding: calc(var(--spacing-base) * 0.5) var(--spacing-base);
}
```

Change `--color-primary` in one place → every button, link, and heading updates. Essential for theming and dark mode.

---

# CSS Transitions

Make property changes **animate smoothly** instead of jumping.

```css
button {
  background: #2c7be5;
  transform: scale(1);
  transition: background 0.2s ease,
              transform  0.15s ease;
}

button:hover {
  background: #1a5fbf;
  transform: scale(1.05);
}
```

`transition: property duration timing-function`

Common timing functions: `ease` (default), `ease-in`, `ease-out`, `ease-in-out`, `linear`

**Rule:** Keep transitions under 300ms for interactive elements — anything longer feels sluggish.

---

# CSS Animations

For more control than transitions, use `@keyframes`:

```css
@keyframes spin {
  from { transform: rotate(0deg); }
  to   { transform: rotate(360deg); }
}

.spinner {
  animation: spin 1s linear infinite;
}

@keyframes fade-in {
  from { opacity: 0; transform: translateY(10px); }
  to   { opacity: 1; transform: translateY(0); }
}

.hero-text {
  animation: fade-in 0.5s ease-out both;
}
```

---

# Background Properties

```css
div {
  /* Solid colour */
  background-color: #f0f4f8;

  /* Image */
  background-image: url('/images/pattern.svg');
  background-repeat: no-repeat;
  background-position: center top;
  background-size: cover;     /* fills element, may crop */
  background-size: contain;   /* fits inside element */

  /* Linear gradient */
  background: linear-gradient(135deg, #2c7be5 0%, #0a3d62 100%);

  /* Shorthand */
  background: url('/bg.jpg') center/cover no-repeat #1a1a2e;
}
```

---

# Overflow and Scroll

```css
/* Default: content spills outside the box */
overflow: visible;

/* Hide overflowing content (can hide scrollbar) */
overflow: hidden;

/* Show scrollbars when needed */
overflow: auto;

/* Always show scrollbars */
overflow: scroll;

/* Control axes separately */
overflow-x: auto;
overflow-y: hidden;

/* Long words in narrow containers */
overflow-wrap: break-word;   /* or word-break: break-all */
```

---

# CSS Reset and Normalise

Browsers have different default stylesheets. Resets give you a blank slate:

```css
/* Minimal modern reset */
*, *::before, *::after { box-sizing: border-box; }
* { margin: 0; padding: 0; }
body { line-height: 1.5; -webkit-font-smoothing: antialiased; }
img, picture, video, canvas, svg { display: block; max-width: 100%; }
input, button, textarea, select { font: inherit; }
```

Popular libraries: **Normalize.css** (preserves useful defaults), **Modern CSS Reset** (Josh Comeau).

Don't invent your own reset — use a well-tested one.

---

# Browser DevTools — Styles Panel

F12 → Elements → select an element → Styles panel:

- See all rules applied to this element (in specificity order)
- Strikethrough = overridden
- Checkbox next to each declaration — toggle to see the effect
- Click on a colour value → colour picker appears
- Add new declarations at the top (in `element.style`)
- **Computed** tab → final resolved values after cascade

**Tip:** Live-edit CSS in DevTools, then copy the final rule to your file. Faster than edit-save-reload cycling.

---

# Common CSS Mistakes

| Mistake | Fix |
|---------|-----|
| Using `px` for font size | Use `rem` |
| Not resetting `box-sizing` | Add `* { box-sizing: border-box; }` |
| Using `!important` to fix specificity | Refactor selectors |
| Deeply nested selectors | Keep selectors shallow (1–3 levels) |
| Absolute positioning for layout | Use Flexbox or Grid |
| Styling the same thing in 10 places | Use a CSS variable |
| No `max-width` on text blocks | `max-width: 65ch` for readability |

---

# Summary

CSS controls appearance through a **cascade** of rules resolved by **specificity** and **source order**.

Key ideas:
- The **box model** describes every element as content + padding + border + margin
- **Flexbox** — one-dimensional layout (nav bars, rows, columns)
- **CSS Grid** — two-dimensional layout (page structure, galleries)
- **Media queries** — different styles at different viewport widths
- **CSS variables** — centralize design tokens
- **Transitions** — make interactions feel polished

---

# Lab 3 Preview

**What you'll build:**
- A single `style.css` shared across all three Lab 2 pages
- Responsive two-column layout (article + aside) using Flexbox
- A card grid on the projects page using CSS Grid with `auto-fill`
- Mobile layout at 375px (single column, stacked nav)
- CSS custom property for the primary colour

**Check your understanding:**
1. What is the difference between `justify-content` and `align-items`?
2. Why does `box-sizing: border-box` make layouts easier?
3. What does `1fr` mean in a grid template?

---

# Questions?

**Next lecture:** Django — a framework that handles HTTP, routing, templates, and databases for you.

Recommended tools before Lab 3:
- Firefox DevTools → Grid inspector (highlights grid lines)
- Chrome DevTools → Flexbox inspector
- CSS-Tricks Flexbox Guide: css-tricks.com/snippets/css/a-guide-to-flexbox
- CSS-Tricks Grid Guide: css-tricks.com/snippets/css/complete-guide-grid

---
