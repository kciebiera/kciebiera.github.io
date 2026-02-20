---
marp: true
theme: default
paginate: true
style: |
  section { font-size: 1.4rem; }
  pre { font-size: 0.85rem; }
  h1 { color: #c47a00; }
  h2 { color: #e67e00; }
  .columns { display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; }
  code { background: #f5f5f5; padding: 0.1em 0.3em; border-radius: 3px; }
---

# Lecture 2
## HTML — Structuring the Web

**WWW 25/26**
Elements · Semantics · Forms · Accessibility

---

# What is HTML?

**HyperText Markup Language** — the skeleton of every web page.

- **HyperText** — text that links to other text
- **Markup** — instructions embedded in the content describing its *structure*
- **Language** — a formal system with defined rules

HTML is **not** a programming language. It has no variables, no loops, no conditions. It describes *what* content is — not *how* to display it or *what to do* with it.

---

# A Very Brief History

| Year | Version | Notable |
|------|---------|---------|
| 1991 | HTML 1.0 | 18 elements, invented at CERN |
| 1997 | HTML 4.0 | Tables, frames, CSS link |
| 2000 | XHTML 1.0 | Strict XML rules |
| 2008 | HTML5 (draft) | Video, canvas, semantic elements |
| 2014 | HTML5 (standard) | Living standard from WHATWG |
| Now | HTML Living Standard | Continuously updated by WHATWG |

**Key shift in HTML5:** The spec describes *browser behaviour*, not just syntax. Browsers should handle broken HTML gracefully, not reject it.

---

# HTML and the Browser

You write HTML text. The browser turns it into a **DOM tree** and renders it.

```
HTML text                    DOM tree
─────────                    ────────
<html>                       Document
  <body>               →       └── html
    <h1>Hi</h1>                      └── body
    <p>There</p>                           ├── h1 "Hi"
  </body>                                  └── p "There"
</html>
```

The DOM (Document Object Model) is a live tree in memory.
- CSS reads it to apply styles.
- JavaScript reads and modifies it.
- Screen readers traverse it to read content aloud.

---

# Every HTML Document

```html
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Page Title</title>
  </head>
  <body>
    <!-- Visible content goes here -->
  </body>
</html>
```

These lines are **not optional boilerplate** — each does something real:
- `<!DOCTYPE html>` — tells browser: standards mode, not quirks mode
- `lang="en"` — helps screen readers and translation tools
- `charset="UTF-8"` — without this, accented chars may break
- `viewport` — prevents mobile browsers from zooming out

---

# Elements, Tags, and Attributes

```html
<a href="https://example.com" target="_blank">Click here</a>
│  │                                          │           │
│  └── attribute (name="value")               └── content └── closing tag
└── opening tag
```

- An **element** = opening tag + content + closing tag
- A **tag** is the `< >` markers
- **Attributes** modify behaviour; always in the opening tag
- Attribute values go in quotes (`"` or `'` — be consistent)

---

# Void Elements

Some elements have **no content** and no closing tag:

```html
<img src="photo.jpg" alt="A cat">
<br>
<hr>
<input type="text" name="email">
<meta charset="UTF-8">
<link rel="stylesheet" href="style.css">
```

Old XHTML required a self-closing slash: `<br />`. In HTML5 the slash is optional and usually omitted.

---

# Nesting Rules

Elements must be **properly nested** — close in reverse order of opening.

```html
<!-- ✅ Correct -->
<p>This is <strong>important</strong> text.</p>

<!-- ❌ Wrong — overlapping tags -->
<p>This is <strong>important</p></strong>
```

Browsers will *try* to fix bad HTML, but the result is unpredictable. Different browsers fix it differently.

**Rule of thumb:** If element B opens inside element A, it must close inside element A.

---

# Block vs Inline

**Block elements** start on a new line and take full width:
`<div>`, `<p>`, `<h1>`–`<h6>`, `<ul>`, `<table>`, `<section>`, …

**Inline elements** flow within text, take only as much width as needed:
`<span>`, `<a>`, `<strong>`, `<em>`, `<img>`, `<code>`, …

```html
<p>
  This paragraph is block-level.
  The word <strong>bold</strong> is inline —
  it doesn't break the flow.
</p>
```

CSS can change this with `display: block / inline / flex / grid`.

---

# Headings

Six levels: `<h1>` (most important) → `<h6>` (least important)

```html
<h1>Page Title</h1>
  <h2>Section</h2>
    <h3>Subsection</h3>
```

**Rules:**
- One `<h1>` per page (the main topic)
- Don't skip levels — don't jump from `<h2>` to `<h4>`
- Use headings for *structure*, not for visual size (that's CSS's job)

Screen readers let users **jump between headings** — your heading hierarchy is the page's table of contents.

---

# Text Content

```html
<p>A paragraph. The browser collapses all whitespace to a single space.</p>

<strong>Bold / important</strong>   <!-- semantic: important -->
<b>Bold / stylistic</b>             <!-- no semantic meaning -->

<em>Italic / emphasis</em>          <!-- semantic: stress emphasis -->
<i>Italic / stylistic</i>           <!-- no semantic meaning -->

<code>inline code</code>
<pre><code>
  preserved whitespace
  for multi-line code
</code></pre>

<blockquote>A long quotation.</blockquote>
```

---

# Links

```html
<!-- Absolute URL -->
<a href="https://example.com">External site</a>

<!-- Relative URL (same site) -->
<a href="/about">About page</a>
<a href="../index.html">Parent directory</a>
<a href="#section-2">Jump to section on same page</a>

<!-- Open in new tab -->
<a href="https://example.com" target="_blank"
   rel="noopener noreferrer">Opens in new tab</a>
```

`rel="noopener noreferrer"` with `target="_blank"` is a **security requirement** — without it, the opened page can access `window.opener`.

---

# Images

```html
<img src="photo.jpg"
     alt="A brown tabby cat sitting on a keyboard"
     width="400"
     height="300">
```

- `src` — path to the image (relative or absolute URL)
- `alt` — **required**; describes the image for:
  - Screen readers
  - Slow connections (shown while loading)
  - Search engines
- `width` / `height` — prevents layout shift while loading

**If an image is purely decorative:** `alt=""` (empty, not omitted).

---

# Lists

```html
<!-- Unordered (bullet points) -->
<ul>
  <li>HTML</li>
  <li>CSS</li>
  <li>JavaScript</li>
</ul>

<!-- Ordered (numbered) -->
<ol>
  <li>Boil water</li>
  <li>Add pasta</li>
  <li>Wait 10 minutes</li>
</ol>

<!-- Description list (term–definition pairs) -->
<dl>
  <dt>HTTP</dt>
  <dd>HyperText Transfer Protocol</dd>
</dl>
```

---

# Why Semantic HTML?

Compare:

```html
<!-- Non-semantic -->
<div class="big-text">News</div>
<div class="container">
  <div class="box">Article content</div>
  <div class="sidebar">Related links</div>
</div>

<!-- Semantic -->
<h1>News</h1>
<main>
  <article>Article content</article>
  <aside>Related links</aside>
</main>
```

The semantic version:
✅ Is understood by screen readers  
✅ Is indexed better by search engines  
✅ Is easier to style and maintain  
✅ Works even when CSS fails

---

# Semantic Landmarks

```html
<body>
  <header>               ← site header, logo, top nav
    <nav>                ← primary navigation
      <ul>…</ul>
    </nav>
  </header>

  <main>                 ← the unique content of this page (one per page)
    <article>            ← self-contained content (blog post, news item)
      <section>…</section>
    </article>
    <aside>              ← related but not essential (sidebar)
  </main>

  <footer>               ← site footer, copyright, links
</body>
```

---

# `<article>` vs `<section>`

**`<article>`** — self-contained, could be syndicated independently:
- A blog post
- A news article
- A product card
- A comment

**`<section>`** — thematic group of content within a page or article:
- The "Introduction" section of an article
- The "Skills" section of a profile

**Mnemonic:** Would this make sense on its own, outside this page?  
→ Yes: `<article>`. → No: `<section>`.

---

# `<div>` and `<span>`

These are the **non-semantic** elements. Use them only when no semantic element fits.

```html
<!-- div: block-level container, no meaning -->
<div class="card">
  <span class="badge">New</span>
  <p>Content here</p>
</div>
```

Before reaching for `<div>`:
1. Is it the main content? → `<main>`
2. Is it a section with a heading? → `<section>`
3. Is it independently useful? → `<article>`
4. Is it navigational? → `<nav>`
5. None of the above? → `<div>`

---

# Tables

Tables are for **tabular data** — rows and columns with a meaningful relationship.

```html
<table>
  <thead>
    <tr>
      <th scope="col">Name</th>
      <th scope="col">Score</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td>Alice</td>
      <td>95</td>
    </tr>
  </tbody>
  <tfoot>
    <tr><td>Average</td><td>95</td></tr>
  </tfoot>
</table>
```

**Never use tables for layout.** That was a 1990s practice. Use CSS Grid/Flexbox.

---

# Forms — Overview

Forms let users send data to the server.

```html
<form action="/submit" method="POST">
  <!-- inputs go here -->
  <button type="submit">Send</button>
</form>
```

- `action` — which URL receives the data (defaults to current URL)
- `method` — `GET` (query string) or `POST` (request body)

The browser serialises all `<input>` values in the form and sends them as a `key=value&key=value` string.

---

# `<input>` — The Swiss Army Element

```html
<input type="text"     name="username" placeholder="Your name">
<input type="email"    name="email"    required>
<input type="password" name="pwd">
<input type="number"   name="age"      min="0" max="120">
<input type="date"     name="birthday">
<input type="checkbox" name="agree"    value="yes">
<input type="radio"    name="colour"   value="red">
<input type="file"     name="avatar"   accept="image/*">
<input type="hidden"   name="csrf"     value="abc123">
<input type="submit"   value="Send">
```

The `type` attribute controls the browser's validation, keyboard (mobile), and UI widget — not just the appearance.

---

# `<label>` — Accessibility Essential

```html
<!-- Method 1: for + id (preferred) -->
<label for="email">Email address</label>
<input type="email" id="email" name="email">

<!-- Method 2: wrapping -->
<label>
  Email address
  <input type="email" name="email">
</label>
```

**Why labels matter:**
- Clicking the label focuses the input (larger click target)
- Screen readers announce the label when the input is focused
- Without a label, a screen reader user hears "edit field, blank" — useless

---

# `<textarea>` and `<select>`

```html
<!-- Multi-line text input -->
<label for="msg">Message</label>
<textarea id="msg" name="message" rows="5" cols="40">
  Default text here
</textarea>

<!-- Dropdown menu -->
<label for="colour">Favourite colour</label>
<select id="colour" name="colour">
  <option value="">-- Please choose --</option>
  <option value="red">Red</option>
  <option value="blue" selected>Blue</option>
  <optgroup label="Muted">
    <option value="grey">Grey</option>
  </optgroup>
</select>
```

---

# GET vs POST Forms

**GET form** — data in the URL:
```html
<form action="/search" method="GET">
  <input type="search" name="q">
  <button>Search</button>
</form>
```
URL becomes: `/search?q=python`  
✅ Bookmarkable, shareable, cacheable  
❌ Not for passwords or large data

**POST form** — data in the request body:
```html
<form action="/login" method="POST">
  <input type="password" name="pwd">
</form>
```
✅ Not in URL (no browser history)  
✅ Can send large data, file uploads  
❌ Not bookmarkable

---

# Form Validation Attributes

HTML5 validation — no JavaScript needed:

```html
<input type="email" required>         <!-- must be present + valid email -->
<input type="number" min="1" max="10"><!-- range constraint -->
<input type="text" minlength="2" maxlength="50">
<input type="text" pattern="[A-Z]{3}[0-9]{3}"
       title="Three letters then three digits">
<input type="url" required>
```

The browser shows native error UI and **blocks the form submission**.

**Important:** Browser validation is a UX convenience, not a security measure. Always validate on the server. Users can bypass browser validation with DevTools or `curl`.

---

# The `<button>` Element

```html
<!-- Submit form (default behaviour inside <form>) -->
<button type="submit">Send</button>

<!-- Reset form to default values -->
<button type="reset">Clear</button>

<!-- Does nothing by itself (use with JavaScript) -->
<button type="button">Click Me</button>

<!-- Disabled state -->
<button type="submit" disabled>Processing…</button>
```

`<button>` is preferred over `<input type="submit">` — it can contain HTML (icons, spans), is easier to style, and has cleaner semantics.

---

# Linking CSS and JavaScript

In the `<head>`:

```html
<!-- CSS: browser downloads and applies before rendering -->
<link rel="stylesheet" href="/static/style.css">
```

Before `</body>`:

```html
<!-- JS at end of body: doesn't block HTML parsing -->
<script src="/static/app.js"></script>

<!-- Or in <head> with defer: runs after HTML is parsed -->
<script src="/static/app.js" defer></script>
```

The order matters. If `<script>` is in `<head>` without `defer`, the browser pauses HTML parsing to download and execute it — slowing down the page.

---

# Useful `<meta>` Tags

```html
<!-- Character encoding — always first in <head> -->
<meta charset="UTF-8">

<!-- Responsive viewport — essential for mobile -->
<meta name="viewport" content="width=device-width, initial-scale=1.0">

<!-- Page description (search engine snippet) -->
<meta name="description"
      content="A course on web development from first principles.">

<!-- Disable phone number detection on iOS -->
<meta name="format-detection" content="telephone=no">

<!-- Open Graph — how the page appears when shared on social media -->
<meta property="og:title" content="My Page">
<meta property="og:image" content="/preview.png">
```

---

# Accessibility Basics

**WCAG** (Web Content Accessibility Guidelines) defines four principles:

| Principle | Meaning |
|-----------|---------|
| **Perceivable** | All content can be perceived (alt text, captions) |
| **Operable** | All UI works with keyboard and assistive tech |
| **Understandable** | Clear language, predictable behaviour |
| **Robust** | Works with current and future browsers/tools |

**Quick wins in HTML:**
- Every `<img>` has `alt`
- Every form field has a `<label>`
- Logical heading hierarchy (`h1` → `h2` → `h3`)
- Interactive elements are focusable (buttons, links, inputs are by default)

---

# Tab Order and Focus

Users who can't use a mouse navigate with **Tab** (forward) and **Shift+Tab** (back).

```html
<!-- Native elements are focusable by default -->
<a href="…">Link</a>        ✅
<button>Button</button>     ✅
<input type="text">         ✅

<!-- Divs are NOT focusable -->
<div onclick="…">Click me</div>   ❌ (keyboard user can't reach it)

<!-- Fix: use a real button, or add tabindex -->
<div tabindex="0" role="button" onclick="…">Click me</div>
```

**Best advice:** Use the correct semantic element and focus management is free.

---

# Browser DevTools — Elements Panel

Open any page → F12 → **Elements** tab

You can:
- Hover elements in the tree → they highlight on the page
- Double-click any attribute to edit it live
- Right-click an element → "Force State" → `:hover`, `:focus`
- See the **Computed** styles panel on the right
- Use the **Accessibility** panel to view the accessibility tree

**Exercise:** Find an `<h1>` on a real website. Is there only one? What does the accessibility tree show?

---

# HTML Comments

```html
<!-- This is a comment. The browser ignores it. -->

<!-- 
  Multi-line comments work too.
  Use them sparingly — they bloat the HTML.
  Never put passwords or secrets in comments:
  the browser receives them even if invisible!
-->

<!-- TODO: replace this placeholder image -->
<img src="placeholder.png" alt="Product photo">
```

Comments are visible in the page source (`Ctrl+U`). Never embed secrets.

---

# Paths: Relative vs Absolute

```html
<!-- Absolute: always works regardless of current page -->
<img src="https://example.com/images/photo.jpg">
<a href="/about">About</a>           <!-- absolute path, same origin -->

<!-- Relative: resolved from the current document's URL -->
<img src="images/photo.jpg">         <!-- ./images/photo.jpg -->
<img src="../images/photo.jpg">      <!-- one directory up -->
<a href="contact.html">Contact</a>   <!-- same directory -->
```

**Preference:**
- Use **absolute paths** (`/images/photo.jpg`) for site assets — safer when URLs change.
- Use **relative paths** for file-based projects without a server.

---

# Common HTML Mistakes

| Mistake | Fix |
|---------|-----|
| Missing `alt` on `<img>` | Always add `alt` (or `alt=""` for decorative) |
| `<br>` for spacing | Use CSS `margin` or `padding` |
| Tables for layout | Use CSS Grid or Flexbox |
| `<b>` / `<i>` for meaning | Use `<strong>` / `<em>` |
| Non-unique `id` | Each `id` must be unique per page |
| Missing `<label>` | Every input needs a visible label |
| `onclick` on `<div>` | Use `<button>` |
| Skipping heading levels | Follow h1 → h2 → h3 order |

---

# Validating HTML

The W3C Markup Validation Service checks for errors:

**Online:** validator.w3.org

```bash
# Or use the html-validate npm package locally:
npx html-validate index.html
```

Common errors it catches:
- Unclosed tags
- Missing required attributes (`alt`, `src`)
- Duplicate `id` values
- Invalid nesting
- Obsolete elements

Get into the habit of validating — it catches bugs that look fine visually but break assistive tech.

---

# HTML5 Media

```html
<!-- Video -->
<video src="demo.mp4" controls width="640" height="480">
  <source src="demo.mp4" type="video/mp4">
  <source src="demo.webm" type="video/webm">
  Your browser does not support video.    <!-- fallback text -->
</video>

<!-- Audio -->
<audio src="music.mp3" controls>
  <source src="music.mp3" type="audio/mpeg">
</audio>
```

Multiple `<source>` elements let the browser pick the format it supports. The fallback text is shown only if video/audio is unsupported entirely.

---

# Putting It All Together

A complete real-world page structure:

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Blog — My Site</title>
  <link rel="stylesheet" href="/static/style.css">
</head>
<body>
  <header>
    <nav aria-label="Main navigation">
      <ul>
        <li><a href="/">Home</a></li>
        <li><a href="/blog" aria-current="page">Blog</a></li>
      </ul>
    </nav>
  </header>
  <main>
    <h1>Latest Posts</h1>
    <article>…</article>
  </main>
  <footer><p>© 2025</p></footer>
  <script src="/static/app.js" defer></script>
</body>
</html>
```

---

# Summary

HTML provides **structure and meaning**. Not appearance.

Key ideas:
- Every element communicates *what* the content is
- Semantic elements (`<nav>`, `<article>`, `<main>`) > generic `<div>`
- Forms collect user input; always validate server-side
- `<label>` + `<input>` pairs are mandatory for accessibility
- The DOM is a live tree — CSS and JavaScript act on it

**What HTML cannot do:** control layout, colors, fonts. That is CSS.

---

# Lab 2 Preview

**What you'll build:**
- Three linked HTML pages served by your Lab 1 server
- Uses semantic elements throughout
- A projects table with proper `<thead>` / `<tbody>`
- A contact form with multiple input types, labels, and validation attributes
- Server parses POST body and returns a confirmation page

**Check your understanding:**
1. What is the difference between `<article>` and `<section>`?
2. Why is `alt=""` (empty) different from omitting `alt`?
3. What happens when you submit a GET form?

---

# Questions?

**Next lecture:** CSS — controlling how all this structure looks.

Recommended reading before Lab 2:
- MDN Web Docs → "Introduction to HTML"
- MDN → "HTML elements reference" (bookmark this)
- validator.w3.org — validate your Lab 2 HTML

---
