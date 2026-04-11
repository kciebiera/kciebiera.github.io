# Lab 8: TypeScript — Types, Interfaces, and the Browser

## Introduction

JavaScript is the only language browsers execute natively — but it has no type system. TypeScript adds a static type layer on top: you write `.ts` files, the compiler (`tsc`) checks them, and emits plain `.js` that the browser can run.

The Goal: Learn TypeScript's type system by building interactive pages that compile and run in the browser. See type errors caught before your code ever executes.

### The Theory

TypeScript adds types to JavaScript at **author time**, not run time. The compiled JavaScript has no type information — it is stripped out entirely. TypeScript is a development tool, not a runtime.

```
you write:   const name: string = "Alice";
tsc emits:   const name = "Alice";
```

This means TypeScript cannot prevent runtime errors from external data (e.g. a JSON API response). You need to *validate* external data and *assert* its shape — TypeScript ensures you use it consistently after that.

## Setup: The Build Pipeline

A browser cannot execute `.ts` files directly. You need a **pipeline** that transforms your TypeScript into JavaScript the browser can load:

```
  TypeScript source       Type checking        Bundle for browser
  ──────────────────  →  ──────────────  →  ────────────────────
  src/main.ts             tsc --noEmit        esbuild → dist/main.js
  src/blog.ts             (catches errors)     (one file, browser-ready)
```

**Why two tools?** `tsc` is the best type checker but is slow as a bundler. `esbuild` is an extremely fast bundler that also compiles TypeScript — but it does *not* check types. Use both: `tsc` for safety, `esbuild` for output.

```bash
npm init -y
npm install --save-dev typescript esbuild
npx tsc --init   # creates tsconfig.json
```

Open `tsconfig.json` and set:

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
  "include": ["src/**/*"]
}
```

Notice `"noEmit": true` — `tsc` only checks types; it does not produce output files. `esbuild` handles that.

### npm scripts — the conventional way to run project commands

Open `package.json` and add a `scripts` section:

```json
{
  "scripts": {
    "check": "tsc --noEmit",
    "build": "npm run check && esbuild src/main.ts --bundle --outfile=dist/main.js --sourcemap",
    "dev": "esbuild src/main.ts --bundle --outfile=dist/main.js --sourcemap --servedir=. --serve"
  }
}
```

Every project defines its commands this way — a new contributor reads `package.json` to learn how to build and run the project.

Create `src/` for TypeScript source, `dist/` for compiled output, and `index.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>TypeScript Lab</title>
    <style>
        body { font-family: system-ui, sans-serif; max-width: 800px; margin: 2rem auto; }
        #output { border: 1px solid #ccc; padding: 1rem; min-height: 100px; }
        .error { color: red; }
    </style>
</head>
<body>
    <h1>TypeScript Lab 8</h1>
    <div id="output"></div>
    <script src="dist/main.js"></script>
</body>
</html>
```

Run `npm run dev` — esbuild starts a local server with live reload. Open the URL it prints (usually `http://localhost:8000`). Every time you save a `.ts` file, esbuild recompiles and the browser refreshes automatically.

> **Source maps:** The `--sourcemap` flag generates a `.js.map` file. Open DevTools → Sources — you will see your original `.ts` files, with breakpoints and variable inspection. Without source maps, you would debug compiled JavaScript.

## Phase 1: Primitive Types and Functions

Create `src/main.ts`:

```typescript
// A helper to display results in the page instead of the console.
function show(label: string, value: unknown): void {
    const output = document.getElementById("output")!;
    const line = document.createElement("p");
    line.textContent = `${label}: ${String(value)}`;
    output.appendChild(line);
}

// TODO: Write a function greet(name: string, times: number): string
// that returns "Hello, <name>! ".repeat(times).trim()
// Try calling it with greet("Alice", "3") — TypeScript should error.

function greet(name: string, times: number): string {
    // TODO: implement
    return "";
}

// TODO: Write a function clamp(value: number, min: number, max: number): number
// that returns value clamped to [min, max].

show("greet", greet("World", 2));
show("clamp(15, 0, 10)", clamp(15, 0, 10));   // → 10
show("clamp(-5, 0, 10)", clamp(-5, 0, 10));   // → 0
```

Save the file — the browser should refresh and display the results.

🧪 Deliberately introduce a type error (pass a string where a number is expected). Run `npm run check` — it should report the error. The `dev` server still compiles (esbuild ignores type errors), but `npm run check` catches them. **Always run `check` before submitting.**

### Exercise: formatDuration

```typescript
// TODO: Write a function formatDuration(totalSeconds: number): string
// that converts seconds to a human-readable string.
// Examples:
//   formatDuration(0)     → "0s"
//   formatDuration(62)    → "1m 2s"
//   formatDuration(3661)  → "1h 1m 1s"
//   formatDuration(86400) → "24h 0m 0s"
// Do not show hours if totalSeconds < 3600.
// Do not show minutes if totalSeconds < 60.

// Render a table of test cases in the page:
const testCases: Array<[number, string]> = [
    [0, "0s"], [5, "5s"], [62, "1m 2s"],
    [3661, "1h 1m 1s"], [86400, "24h 0m 0s"],
];

const table = document.createElement("table");
table.innerHTML = "<tr><th>Input</th><th>Expected</th><th>Got</th><th>✓</th></tr>";
for (const [input, expected] of testCases) {
    const got = formatDuration(input);
    const pass = got === expected;
    const row = document.createElement("tr");
    row.innerHTML = `<td>${input}</td><td>${expected}</td><td>${got}</td>
                     <td>${pass ? "✅" : "❌"}</td>`;
    if (!pass) row.classList.add("error");
    table.appendChild(row);
}
document.getElementById("output")!.appendChild(table);
```

## Phase 2: Promises and async/await

JavaScript (and TypeScript) is **single-threaded** but handles slow operations (network requests, timers) using **Promises**. A `Promise<T>` is a value that will arrive in the future.

```typescript
// src/async.ts

function delay(ms: number): Promise<void> {
    return new Promise((resolve) => setTimeout(resolve, ms));
}

// async/await makes Promise code read top-to-bottom:
async function countdown(element: HTMLElement): Promise<void> {
    for (let i = 5; i >= 0; i--) {
        element.textContent = `${i}…`;
        await delay(1000);
    }
    element.textContent = "Go!";
}
```

Import and use in `main.ts`:

```typescript
import { delay, countdown } from "./async";

const counter = document.createElement("h2");
document.getElementById("output")!.appendChild(counter);
countdown(counter);
```

Save — you should see a live countdown in the browser.

### Fetching data with types

The browser's `fetch()` API retrieves data from any URL. Type the response so TypeScript knows the shape:

```typescript
// src/async.ts (continued)

interface Todo {
    userId:    number;
    id:        number;
    title:     string;
    completed: boolean;
}

async function fetchTodo(id: number): Promise<Todo> {
    const res = await fetch(`https://jsonplaceholder.typicode.com/todos/${id}`);
    if (!res.ok) {
        throw new Error(`HTTP ${res.status}`);
    }
    const data: Todo = await res.json();
    return data;
}

export { delay, countdown, fetchTodo, Todo };
```

In `main.ts`, fetch and render:

```typescript
import { fetchTodo, Todo } from "./async";

async function renderTodos(): Promise<void> {
    const container = document.createElement("div");
    container.innerHTML = "<h3>Todos from API</h3>";
    document.getElementById("output")!.appendChild(container);

    for (const id of [1, 2, 3]) {
        const todo = await fetchTodo(id);
        const p = document.createElement("p");
        p.textContent = `${todo.completed ? "✅" : "⬜"} ${todo.title}`;
        container.appendChild(p);
    }
}

renderTodos();
```

🧪 Change the `Todo` interface (e.g., rename `title` to `name`) — `tsc` should catch every place that accesses the wrong field. This is the core value of TypeScript: **the compiler knows the shape of your data**.

### Exercise: fetchWithTimeout

```typescript
// TODO: Write a function fetchWithTimeout<T>(url: string, ms: number): Promise<T>
// that rejects if the request takes longer than `ms` milliseconds.
// Hint: use Promise.race() with delay() that throws after the timeout.
//
// Test it:
//   fetchWithTimeout<Todo>("https://jsonplaceholder.typicode.com/todos/1", 5000)
//     → should succeed
//   fetchWithTimeout<Todo>("https://jsonplaceholder.typicode.com/todos/1", 1)
//     → should reject with a timeout error
//
// Display "✅ Succeeded" or "❌ Timed out" in the page for each test.
```

### Exercise: parallel fetching

```typescript
// TODO: Fetch todos 1 through 10 in parallel using Promise.all().
// Render them as a checklist in the page. Show the total time taken
// (use performance.now() before and after).
// Compare with fetching them sequentially (one after another) — show both times.
```

## Phase 3: Interfaces and Object Types

An **interface** describes the shape of an object. It is erased at compile time.

> **`type` vs `interface` — when to use which?**
> Both can describe object shapes. Use `interface` for objects you expect others to extend (`interface Animal { ... }`). Use `type` for unions, intersections, and aliases (`type ID = string | number`). In practice the difference is small — pick one convention and be consistent. This course prefers `interface` for objects and `type` for everything else.

Create `src/blog.ts`:

```typescript
interface Category {
    id:   number;
    name: string;
    slug: string;
}

interface Post {
    id:       number;
    title:    string;
    slug:     string;
    body:     string;
    pubDate:  string;
    category: Category | null;   // a field that is either a Category or nothing — union types are introduced in Phase 4
}

interface Comment {
    id:      number;
    author:  string;
    body:    string;
    created: string;
}

// TODO: Write a function summarise(post: Post): string that returns
// "<title> (<category name or 'Uncategorised'>) — <first 50 chars of body>..."

function summarise(post: Post): string {
    // TODO: implement
    return "";
}

// TODO: Write a function filterByCategory(posts: Post[], categoryName: string): Post[]
// that returns only posts whose category.name matches (case-insensitive).

export { Post, Comment, Category, summarise, filterByCategory };
```

Import and render in `main.ts`:

```typescript
import { Post, summarise, filterByCategory } from "./blog";

const posts: Post[] = [
    {
        id: 1, title: "Hello TypeScript", slug: "hello-ts",
        body: "TypeScript is JavaScript with types. It compiles to plain JS.",
        pubDate: "2025-01-01",
        category: { id: 1, name: "Tech", slug: "tech" },
    },
    {
        id: 2, title: "CSS Grid", slug: "css-grid",
        body: "CSS Grid is a two-dimensional layout system for the web.",
        pubDate: "2025-01-15",
        category: { id: 2, name: "Frontend", slug: "frontend" },
    },
    {
        id: 3, title: "Django REST", slug: "django-rest",
        body: "Build a REST API with Django and serve JSON to any client.",
        pubDate: "2025-02-01",
        category: { id: 1, name: "Tech", slug: "tech" },
    },
];

// Render each post as a card
function renderPostCard(post: Post): HTMLElement {
    const card = document.createElement("article");
    card.innerHTML = `<h3>${post.title}</h3><p>${summarise(post)}</p>`;
    return card;
}

const output = document.getElementById("output")!;
for (const post of posts) {
    output.appendChild(renderPostCard(post));
}

// Show filtered results
const techPosts = filterByCategory(posts, "tech");
const filteredSection = document.createElement("div");
filteredSection.innerHTML = `<h3>Tech posts: ${techPosts.map(p => p.title).join(", ")}</h3>`;
output.appendChild(filteredSection);
```

### Exercise: sortPosts with live selector

```typescript
// TODO: Write a function sortPosts(posts: Post[], by: "title" | "date" | "category"): Post[]
// that returns a new sorted array. Sort by:
//   "title"    → alphabetically by title
//   "date"     → by pubDate (newest first)
//   "category" → alphabetically by category name (null last)
//
// Add a <select> element to the page with these three options.
// When the user changes the selection, re-sort and re-render the post cards.
// Hint: listen to the "change" event on the <select>.
```

## Phase 4: Union Types, Type Guards, and Enums

```typescript
// src/status.ts

// An enum compiles to a real JavaScript object.
enum PostStatus {
    Draft     = "draft",
    Published = "published",
    Archived  = "archived",
}

// A union type: the value must be one of these strings.
type HttpMethod = "GET" | "POST" | "PUT" | "PATCH" | "DELETE";

interface ApiRequest {
    method:  HttpMethod;
    path:    string;
    body?:   unknown;    // ? makes the field optional
}

// A type guard narrows a wide type to a specific one.
function isPost(value: unknown): value is Post {
    return (
        typeof value === "object" &&
        value !== null &&
        "title" in value &&
        "slug"  in value
    );
}

// TODO: Write a function describeRequest(req: ApiRequest): string
// that returns e.g. "GET /api/posts/" or "POST /api/posts/ (has body)"

export { PostStatus, HttpMethod, ApiRequest, isPost };
```

### Exercise: discriminated unions and SVG

```typescript
// TODO: Define a discriminated union type Shape:
//
// interface Circle    { kind: "circle";    radius: number; }
// interface Rectangle { kind: "rectangle"; width: number; height: number; }
// interface Triangle  { kind: "triangle";  base: number; height: number; }
// type Shape = Circle | Rectangle | Triangle;
//
// 1. Write a function area(shape: Shape): number
//    Use a switch on shape.kind — TypeScript narrows the type in each branch.
//    Try removing a case: TypeScript should warn about unhandled variants.
//
// 2. Write a function renderShape(shape: Shape): SVGElement
//    that creates an SVG element for each shape:
//    - Circle    → <circle cx="50" cy="50" r="<radius>" fill="steelblue" />
//    - Rectangle → <rect width="<w>" height="<h>" fill="coral" />
//    - Triangle  → <polygon points="..." fill="seagreen" />
//
// 3. Create an array of shapes and render them as an SVG gallery in the page.
//    Below each shape, display "Area: <value>".
//
// Hint: SVG elements must be created with document.createElementNS, and their
// attributes must be set with setAttribute (not direct property assignment):
//
//   // Use this helper to save boilerplate:
//   function svgEl(
//       tag: string,
//       attrs: Record<string, string | number>
//   ): SVGElement {
//       const el = document.createElementNS("http://www.w3.org/2000/svg", tag);
//       for (const [k, v] of Object.entries(attrs)) {
//           el.setAttribute(k, String(v));
//       }
//       return el as SVGElement;
//   }
//
//   // Wrap shapes in an <svg> container with an explicit viewBox:
//   const svg = document.createElementNS("http://www.w3.org/2000/svg", "svg");
//   svg.setAttribute("width", "120");
//   svg.setAttribute("height", "120");
//   svg.setAttribute("viewBox", "0 0 120 120");
//   svg.appendChild(svgEl("circle", { cx: 60, cy: 60, r: 50, fill: "steelblue" }));
```

### Exercise: robust type guard

```typescript
// TODO: Write a function isValidPost(data: unknown): data is Post
// that validates ALL fields — not just "title" and "slug".
// Check that:
//   - data is an object (not null)
//   - id is a number
//   - title, slug, body, pubDate are strings
//   - category is null or an object with id (number), name (string), slug (string)
//
// Add a <textarea> to the page. When the user pastes JSON and clicks a
// "Validate" button:
//   - Parse the JSON (handle parse errors)
//   - Run isValidPost on the result
//   - Display "✅ Valid Post" and render the card, or "❌ Invalid: <reason>"
```

> **That was painful, right?** Manually validating every field, including a nested object, produces a lot of repetitive, error-prone code. In practice, developers reach for **schema validation libraries** that do this automatically:
>
> - **[Zod](https://zod.dev)** — TypeScript-first, infers types from schemas: `z.object({ id: z.number(), title: z.string(), ... })`
> - **[Valibot](https://valibot.dev)** — similar API, smaller bundle
> - **[Yup](https://github.com/jquense/yup)** — popular with form libraries (Formik, React Hook Form)
>
> These libraries generate both the runtime validator *and* the TypeScript type from a single schema definition. No duplicated logic.

## Phase 5: Generics

Generics let you write functions that work with any type while keeping type safety.

```typescript
// src/utils.ts

// A generic identity function — T is inferred from the argument.
function identity<T>(value: T): T {
    return value;
}

// TODO: Write a generic function first<T>(arr: T[]): T | undefined
// that returns the first element of an array, or undefined if empty.

// TODO: Write a generic function groupBy<T>(
//     items: T[],
//     keyFn: (item: T) => string
// ): Record<string, T[]>
// that groups items into an object by the string key returned by keyFn.

// Example usage (should work without any type annotations at the call site):
// groupBy(posts, p => p.category?.name ?? "none")
// → { "Tech": [post1, post3], "Frontend": [post2] }

export { identity, first, groupBy };
```

Render grouped posts as categorised sections in the page — one `<section>` per group with a heading showing the category name and the post cards inside.

### Exercise: pipe

```typescript
// TODO: Write a generic pipeline combinator:
// function pipe<T>(...fns: Array<(arg: T) => T>): (arg: T) => T
//
// Note: this signature requires every function to accept and return the SAME
// type T. That's a deliberate simplification — it works perfectly for pipelines
// over a single type (like Post[]). A true variadic pipe where each step can
// change the type (A → B → C) requires complex overloads and is beyond this lab.
// Libraries like fp-ts and Ramda implement the full version.
//
// It returns a new function that applies each function in sequence:
//   pipe(f, g, h)(x)  ===  h(g(f(x)))
//
// Use it to build a post-processing pipeline:
//   const process = pipe<Post[]>(
//       posts => posts.filter(p => p.category !== null),
//       posts => posts.filter(p => p.pubDate >= "2025-01-10"),
//       posts => posts.sort((a, b) => a.title.localeCompare(b.title)),
//       posts => posts.slice(0, 5),
//   );
//
// Render the pipeline input and output side by side in the page.
```

### Exercise: memoize

```typescript
// TODO: Write a generic memoization function:
// function memoize<A extends PropertyKey, R>(fn: (arg: A) => Promise<R>): (arg: A) => Promise<R>
//
// Note: A extends PropertyKey (= string | number | symbol) so the cache key
// can be used as an object property. This allows memoizing fetchTodo(id: number).
//
// It caches the result of the first call for each argument.
// Subsequent calls with the same argument return the cached value instantly.
//
// Use it to memoize fetchTodo:
//   const cachedFetch = memoize(fetchTodo);
//
// Call cachedFetch(1) twice. The first call should take ~100ms (network).
// The second call should be instant.
// Display "Cache MISS (120ms)" or "Cache HIT (0ms)" next to each result in the page.
// Use performance.now() to measure.
```

## Submission

Final checks:

1. `npm run check` produces zero errors.
2. All functions have explicit return types. There are no `any` types.
3. The page loads in the browser and displays results from all phases.
4. The `<select>` sort control (Phase 3) re-sorts posts live.
5. The JSON validator (Phase 4) correctly accepts valid posts and rejects invalid ones.
6. `fetchWithTimeout` (Phase 2) times out on a 1ms deadline.

Run:

```bash
npm run check
```

**Exploration:** Open DevTools → Sources. Set a breakpoint in `fetchTodo`. Reload the page. Step through the code — notice you're stepping through `.ts` files, not compiled `.js`. This is source maps at work.
