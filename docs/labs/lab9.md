# Lab 9: TypeScript and the DOM — Interactive Pages

## Introduction

In Lab 7 you wrote raw JavaScript inside a `<script>` tag to load posts from the API. That works, but there is no type safety — you could mistype `post.titel` and only discover the bug in production. TypeScript can type the DOM too.

The Goal: Build an interactive blog browser — filter, search, and expand posts — entirely in TypeScript, compiled to a single `.js` file and served by Django.

### The Theory

The browser exposes the DOM as a JavaScript API. TypeScript ships type definitions for all standard DOM types (`HTMLElement`, `HTMLInputElement`, `Event`, etc.). When you call `document.getElementById("search")`, TypeScript knows the return type is `HTMLElement | null` — it forces you to handle the `null` case.

```typescript
const input = document.getElementById("search");
input.value;  // ❌ Error: Object is possibly null

const input = document.getElementById("search") as HTMLInputElement;
input.value;  // ✅ TypeScript trusts the cast
```

## Preparation: Django API search support

Before writing TypeScript, make sure your Django API supports the `?search=` query parameter. In your `PostListView.get()` from Lab 7, add filtering:

```python
# blog/api.py — inside PostListView.get()
def get(self, request):
    posts = Post.objects.all()
    query = request.GET.get("search", "")
    if query:
        posts = posts.filter(title__icontains=query)
    return JsonResponse({"posts": [post_to_dict(p) for p in posts]})
```

Test it:

```bash
curl "http://127.0.0.1:8000/api/posts/?search=django"
```

You should see only posts whose title contains "django". The TypeScript code in this lab will call this endpoint.

## Setup

You already know the build pipeline from Lab 8 — `tsc` checks types, `esbuild` bundles for the browser. Now we point it at Django's static files directory.

In your Django project root:

```bash
npm init -y
npm install --save-dev typescript esbuild
```

Create `tsconfig.json`:

```json
{
  "compilerOptions": {
    "target": "ES2020",
    "module": "ES2020",
    "moduleResolution": "bundler",
    "strict": true,
    "noEmit": true,
    "sourceMap": true
  },
  "include": ["pages/static/pages/ts/**/*"]
}
```

Add npm scripts to `package.json`:

```json
{
  "scripts": {
    "check": "tsc --noEmit",
    "build": "npm run check && esbuild pages/static/pages/ts/main.ts --bundle --outfile=pages/static/pages/js/main.js --sourcemap",
    "watch": "esbuild pages/static/pages/ts/main.ts --bundle --outfile=pages/static/pages/js/main.js --sourcemap --watch"
  }
}
```

Run `npm run watch` in one terminal, `python manage.py runserver` in another.

## Phase 1: Type-Safe DOM Selection

Create `pages/static/pages/ts/main.ts`:

```typescript
// A helper that throws if the element is missing —
// better than silently failing at runtime.
function getElement<T extends HTMLElement>(id: string): T {
    const el = document.getElementById(id);
    if (el === null) {
        throw new Error(`Element #${id} not found`);
    }
    return el as T;
}

// TODO: Use getElement to grab references to:
//   - #search-input  (HTMLInputElement)
//   - #post-list     (HTMLDivElement)
//   - #status-bar    (HTMLParagraphElement)

const searchInput = getElement<HTMLInputElement>("search-input");
const postList    = getElement<HTMLDivElement>("post-list");
const statusBar   = getElement<HTMLParagraphElement>("status-bar");
```

Create `pages/templates/pages/blog_browser.html`:

```html
{ % extends "pages/base.html" %}
{ % load static %}
{ % block title %}Browse Posts{ % endblock %}

{ % block content %}
<h1>Browse Posts</h1>

<input type="text" id="search-input" placeholder="Search…" autocomplete="off">
<p id="status-bar"></p>
<div id="post-list"></div>

<script src="{ % static 'pages/js/main.js' %}"></script>
{ % endblock %}
```

Add a route `path("browse/", views.blog_browser, name="blog-browser")` and a trivial view.

🧪 Run `npm run build`, then visit `/browse/`. Open DevTools Console — you should see no errors. In Sources tab, confirm you can see `main.ts` (not just `main.js`).

## Phase 2: Fetching, State, and Rendering

Modern UIs follow a simple pattern: **state → render**. You keep all data in a single state object, and whenever it changes, you re-render the UI from scratch. This is the mental model behind React, Vue, and every modern framework — but you can use it with raw DOM too.

### The state machine

An async page has four possible states. Model them explicitly:

```typescript
type PageStatus = "idle" | "loading" | "success" | "error";
type SortField  = "title" | "date" | "category";

interface AppState {
    posts:   Post[];
    query:   string;
    status:  PageStatus;
    error:   string;
    sortBy:  SortField;
}

let state: AppState = {
    posts:  [],
    query:  "",
    status: "idle",
    error:  "",
    sortBy: "date",
};
```

Defining `PageStatus` as a union of literal strings means TypeScript will catch typos: `state.status = "laoding"` is a compile error.

### Fetching data

Define the API response shape:

```typescript
interface Post {
    id:       number;
    title:    string;
    slug:     string;
    body:     string;
    pub_date: string;    // ISO string from Django
    category: string | null;
}

interface PostsResponse {
    posts: Post[];
}

async function fetchPosts(query: string = ""): Promise<Post[]> {
    const url = query
        ? `/api/posts/?search=${encodeURIComponent(query)}`
        : "/api/posts/";
    const res = await fetch(url);
    if (!res.ok) {
        throw new Error(`API error: ${res.status}`);
    }
    const data: PostsResponse = await res.json();
    return data.posts;
}
```

### Sorting

```typescript
// TODO: Implement sortPosts. Sort by:
//   "title"    → alphabetically
//   "date"     → by pub_date (newest first)
//   "category" → alphabetically by category name (null last)
function sortPosts(posts: Post[], by: SortField): Post[] {
    const sorted = [...posts];
    // TODO: implement
    return sorted;
}
```

### Rendering from state

The `render` function reads state and builds the entire UI. It never reads the DOM for data — state is the single source of truth.

```typescript
function renderPost(post: Post): HTMLElement {
    const article = document.createElement("article");
    article.dataset.id = String(post.id);

    const heading = document.createElement("h2");
    heading.textContent = post.title;

    const meta = document.createElement("small");
    // TODO: Set meta.textContent to "Category: <category or 'None'> | <pub_date first 10 chars>"

    const excerpt = document.createElement("p");
    // TODO: Set excerpt.textContent to the first 120 characters of post.body + "…"

    article.append(heading, meta, excerpt);
    return article;
}

function render(): void {
    postList.innerHTML = "";

    if (state.status === "loading") {
        statusBar.textContent = "Loading…";
        return;
    }
    if (state.status === "error") {
        statusBar.textContent = `Error: ${state.error}`;
        return;
    }

    const sorted = sortPosts(state.posts, state.sortBy);
    statusBar.textContent = `${sorted.length} post(s) found`;
    for (const post of sorted) {
        postList.appendChild(renderPost(post));
    }
}
```

### Updating state and re-rendering

Every user action follows the same cycle: update state → re-render.

```typescript
async function loadPosts(query: string = ""): Promise<void> {
    state = { ...state, status: "loading", query };
    render();

    try {
        const posts = await fetchPosts(query);
        state = { ...state, posts, status: "success" };
    } catch (err) {
        state = { ...state, status: "error", error: String(err) };
    }

    render();
}

loadPosts();
```

> **Why `{ ...state, status: "loading" }` instead of `state.status = "loading"`?**
> Creating a new object makes each state transition explicit and traceable. It also prevents accidental partial updates. This is the same pattern React's `setState` and Redux use.

### Exercise: sort control

```typescript
// TODO: Add a <select id="sort-select"> to the HTML template with options:
//   <option value="date">Date</option>
//   <option value="title">Title</option>
//   <option value="category">Category</option>
//
// In main.ts, listen for the "change" event:
//   sortSelect.addEventListener("change", () => {
//       state = { ...state, sortBy: sortSelect.value as SortField };
//       render();
//   });
//
// The page should re-sort posts immediately when the user changes the select.
```

🧪 The page should load and display all posts. Open Network tab — confirm the single fetch to `/api/posts/`.

## Phase 3: Live Search with Debounce

Calling the API on every keypress is wasteful. A **debounce** waits until the user stops typing.

```typescript
function debounce<T extends (...args: unknown[]) => void>(
    fn: T,
    delay: number
): (...args: Parameters<T>) => void {
    let timer: ReturnType<typeof setTimeout>;
    return (...args) => {
        clearTimeout(timer);
        timer = setTimeout(() => fn(...args), delay);
    };
}

const debouncedLoad = debounce((query: string) => {
    loadPosts(query);
}, 300);

searchInput.addEventListener("input", () => {
    debouncedLoad(searchInput.value.trim());
});
```

🧪 Type in the search box. Network requests should only fire ~300ms after you stop typing.

### Exercise: highlight matching text

```typescript
// TODO: When the user searches, highlight matching text in post titles.
// Instead of setting heading.textContent = post.title, build HTML that
// wraps matched substrings in <mark> tags.
//
// Example: query = "type", title = "Hello TypeScript"
//          → "Hello <mark>Type</mark>Script"
//
// Write a function:
//   function highlightMatch(text: string, query: string): string
// that returns the HTML string with <mark> around case-insensitive matches.
// Use innerHTML (not textContent) for the heading when a query is active.
//
// Hint: use a RegExp with the "gi" flag and String.replace().
// Be careful with HTML injection — what if the user types "<script>"?
// Escape the query before building the RegExp.
```

## Phase 4: Expand/Collapse with Events

Make post excerpts expandable. Update `renderPost`:

```typescript
function renderPost(post: Post): HTMLElement {
    const article = document.createElement("article");

    const heading = document.createElement("h2");
    heading.textContent = post.title;

    const excerpt = document.createElement("p");
    excerpt.textContent = post.body.slice(0, 120) + "…";

    const fullBody = document.createElement("p");
    fullBody.textContent = post.body;
    fullBody.hidden = true;

    const toggle = document.createElement("button");
    toggle.textContent = "Read more";

    toggle.addEventListener("click", () => {
        const isExpanded = !fullBody.hidden;
        fullBody.hidden    = isExpanded;
        excerpt.hidden     = !isExpanded;
        toggle.textContent = isExpanded ? "Read more" : "Show less";
    });

    // TODO: Add a keyboard handler: when the article receives a keydown
    // event with key === "Enter", trigger toggle.click()
    article.tabIndex = 0;

    article.append(heading, excerpt, fullBody, toggle);
    return article;
}
```

🧪 Click "Read more" — the full text should appear. Click "Show less" — it collapses. Tab to a post and press Enter — same behaviour.

### Exercise: keyboard navigation

```typescript
// TODO: Add keyboard navigation to the post list.
// Add focusedIndex to AppState (type: number, initial: -1).
//
// Listen for "keydown" on the document:
//   ArrowDown → increment focusedIndex (wrap at end)
//   ArrowUp   → decrement focusedIndex (wrap at start)
//   Enter     → toggle expand/collapse on the focused post
//   Escape    → collapse all posts, reset focusedIndex to -1
//
// In the render() function, add a CSS class "focused" to the article
// at state.focusedIndex. Define .focused in your CSS:
//   .focused { outline: 2px solid steelblue; outline-offset: 4px; }
//
// Call article.scrollIntoView({ block: "nearest" }) when focus changes
// so the focused post is always visible.
```

## Phase 5: localStorage Persistence

Save the user's preferences so they survive page reloads.

```typescript
// TODO: Define the shape of persisted data:
interface PersistedState {
    query:  string;
    sortBy: SortField;
}

// TODO: Write functions:
//   function savePrefs(prefs: PersistedState): void
//     → JSON.stringify and save to localStorage under key "blog-prefs"
//
//   function loadPrefs(): PersistedState | null
//     → Read from localStorage, parse JSON, validate the shape using a
//       type guard isPersistedState(data: unknown): data is PersistedState.
//       Return null if missing, corrupt, or invalid.
//
// On page load: call loadPrefs(). If valid, use its values for initial state.
// On every state change: call savePrefs({ query: state.query, sortBy: state.sortBy }).
//
// Why a type guard? localStorage is stringly typed — anyone (or a bug) could
// write garbage there. Never trust external data without validation.
```

🧪 Search for something, change the sort, reload the page — your preferences should be restored.

## Submission

Final checks:

1. `npm run check` has zero errors.
2. No `any` types in `main.ts`.
3. Search debounces correctly (verify in Network tab).
4. Matching text is highlighted with `<mark>` tags during search.
5. Expand/collapse works with both mouse and keyboard.
6. Arrow keys navigate between posts with a visible focus indicator.
7. Sort preference and search query persist across page reloads via `localStorage`.
8. Open DevTools → Sources — you can see and set breakpoints in your `.ts` file (source maps).

**Exploration:** Add a `<select>` element for category filtering. Fetch all posts and filter client-side by the selected category using `Array.filter()`. Update `state` with the filtered results and call `render()`. This means the sort, search highlight, and category filter must all compose correctly.
