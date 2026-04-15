---
marp: true
theme: default
paginate: true
style: |
  section { font-size: 1.4rem; }
  pre { font-size: 0.85rem; }
  h1 { color: #092e20; }
  h2 { color: #44b78b; }
  .columns { display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; }
  code { background: #f5f5f5; padding: 0.1em 0.3em; border-radius: 3px; }
---

# Lecture 8

## JavaScript & TypeScript — The Browser’s Programming Model

**WWW 25/26**
JavaScript · DOM · async/await · TypeScript · types · tooling

---

# What This Lecture Is About

You already know how to program. This lecture explains **how JavaScript works in the browser** and why **TypeScript** is useful once projects get larger.

Today’s goals:

- Understand where JavaScript runs
- Understand how JavaScript interacts with a web page
- Understand the browser’s event-driven execution model
- Understand why asynchronous code exists
- Understand what TypeScript adds on top of JavaScript

> The goal is not to memorise syntax. The goal is to build the runtime model:
> **page + DOM + events + async + types**.

---

# The Three-Layer Web Stack

Every web page is built from three languages with distinct roles:

```
┌─────────────────────────────────────────────────┐
│  HTML — structure                               │
│  <h1>Hello</h1>  <p>Some text</p>               │
│  What is on the page                            │
├─────────────────────────────────────────────────┤
│  CSS — appearance                               │
│  h1 { color: red; font-size: 2rem; }            │
│  What it looks like                             │
├─────────────────────────────────────────────────┤
│  JavaScript — behaviour                         │
│  document.getElementById("btn").onclick = ...   │
│  What it does when the user interacts           │
└─────────────────────────────────────────────────┘
```

**JavaScript is the only *scripting* language that runs natively in the browser.**
JavaScript is the browser’s main programming language for interactivity and dynamic behaviour.
(WebAssembly, covered later, is the second native language — a compilation target for C++, Rust, and others.)

---

# JavaScript in HTML — The `<script>` Tag

JavaScript is embedded in (or linked from) HTML:

```html
<!DOCTYPE html>
<html>
<head>
  <title>My Page</title>
</head>
<body>
  <h1 id="greeting">Hello</h1>
  <button id="btn">Click me</button>
  <script>
    document.getElementById("btn").addEventListener("click", function () {
      document.getElementById("greeting").textContent = "You clicked!";
    });
  </script>
  <!-- Or load from an external file (preferred for real projects) -->
  <!-- <script src="app.js"></script> -->
</body>
</html>
```
- The browser loads HTML from top to bottom
- When it reaches a `<script>`, it executes JavaScript
- That code can read and modify the page through the DOM
In real projects, JavaScript usually lives in separate files and is loaded with `<script src="...">`.
---

# JavaScript for C++ Developers

If you know C++, JavaScript syntax will feel familiar. The differences are in semantics.

<div class="columns">
<div>

**C++**

```cpp
int add(int a, int b) {
    return a + b;
}

int x = 10;
const int y = 20;

for (int i = 0; i < 5; i++) {
    if (i % 2 == 0) {
        std::cout << i << std::endl;
    }
}
```

</div>
<div>

**JavaScript**

```javascript
function add(a, b) {
    return a + b;
}

let x = 10;
const y = 20;

for (let i = 0; i < 5; i++) {
    if (i % 2 === 0) {
        console.log(i);
    }
}
```

</div>
</div>

Braces, semicolons, `if`/`for`/`while`, `return` — all the same. Key difference: **no type declarations**.
The syntax looks familiar. The important differences are not syntax but runtime behaviour: dynamic typing, garbage collection, objects, and the event-driven browser environment.

---

# Key Differences from C++

JavaScript differs from C++ in three big ways:

1. Execution model
   - Runs inside a browser environment
   - Event-driven
   - Usually one main thread for page code
2. Language model
   - Dynamic typing
   - Garbage collection
   - Objects and functions are lightweight, flexible values
3. Safety model
   - No manual memory management
   - No dangling pointers or buffer overflows in user code
   - Errors are more often logic/type errors than memory errors

> JavaScript is **safe** (no buffer overflows, no dangling pointers) but **loose** (no type errors until runtime).

---

# Variables and Values

JavaScript has **dynamic types** — a variable can hold any value at any time:

```javascript
let x = 42;           // number (all numbers are 64-bit floats)
x = "hello";          // now it's a string — perfectly legal
x = true;             // now it's a boolean

const PI = 3.14159;   // const: can't reassign the binding

// Primitive types:
typeof 42           // "number"
typeof "hi"         // "string"
typeof true         // "boolean"
typeof null         // "object"  ← famous historical bug in JS
typeof undefined    // "undefined"
```

**The coercion trap** — JavaScript silently converts between types:

```javascript
"5" + 1    // "51"  — number coerced to string (+ prefers strings)
"5" - 1    // 4     — string coerced to number (- is always numeric)
"5" == 5   // true  — loose equality coerces types (avoid!)
"5" === 5  // false — strict equality, no coercion (always prefer ===)
```

---

# `null` vs. `undefined` — Two Kinds of Nothing

C++ has one concept (null pointer). JavaScript has **two** distinct "nothing" values:

<div class="columns">
<div>

## `null`

**Intentional** absence — the programmer explicitly set it.

```javascript
let user = null;      // "no user yet"
let category = null;  // "post has no category"
```

Think of it as: "I looked, there is nothing here."

</div>
<div>

## `undefined`

**Unintentional** absence — value was never assigned.

```javascript
let x;             // declared but not assigned
obj.missingField   // property doesn't exist
function f() {}    // f() returns undefined
```

Think of it as: "Nobody ever set this."

</div>
</div>

```javascript
// Both are falsy, but not strictly equal — different types:
null == undefined    // true  (loose equality coerces types — avoid!)
null === undefined   // false (strict equality — null and undefined are distinct)

// Safe "either one" check with ?. and ?? (optional chaining + nullish coalescing):
const name = user?.name ?? "Anonymous";  // user?.name is undefined if user is null/undefined
```

> **Rule:** use `null` when you deliberately clear a value; never assign `undefined` yourself.

---

# Objects — Dynamic Structs

Objects in JS are **dynamic structs** (V8 optimises them like C++ hidden classes). Use `Map` when you need non-string keys (`std::unordered_map` equivalent).

```javascript
const post = {
    id: 1,
    title: "Hello World",
    published: true,
};

post.title          // "Hello World"
post["title"]       // same — bracket == dot notation
post.body           // undefined — missing field, no error
post.author = "Alice";  // add a new field at any time
```

---

# Arrays

Arrays are **dynamic lists** — zero-indexed, resizable, mixed types allowed:

```javascript
const nums = [10, 20, 30];
nums.push(40);          // [10, 20, 30, 40]
nums.length             // 4
nums[10]                // undefined — no bounds-check error

// Common higher-order methods (like std::transform / std::copy_if):
nums.map(x => x * 2)        // [20, 40, 60, 80]
nums.filter(x => x > 20)    // [30, 40]
nums.reduce((acc, x) => acc + x, 0)  // 150
```

---

# `const` Does Not Mean Immutable

Coming from C++, `const` looks familiar — but it means something narrower in JavaScript.

<div class="columns">
<div>

**C++ `const`**

```cpp
const User u = { "Alice", 42 };
u.name = "Bob";   // compile error
// the whole object is immutable
```

</div>
<div>

**JavaScript `const`**

```javascript
const user = { name: "Alice", age: 42 };
user.name = "Bob";   // ✅ perfectly legal!
user.age  = 99;      // ✅ also fine

user = {};           // ❌ Error: Assignment to
                     //    constant variable
```

</div>
</div>

`const` in JavaScript means: **the binding (variable name) can't be reassigned** — it doesn't freeze the object the variable points to.

```javascript
const nums = [1, 2, 3];
nums.push(4);     // ✅ mutates the array — allowed
nums = [5, 6];    // ❌ rebinds the variable — not allowed
```

> To deeply freeze an object, use `Object.freeze(obj)` — but this is shallow and rarely needed. **The habit:** use `const` by default for everything, `let` only when you need to reassign.

---

# Functions as First-Class Values

In JavaScript, functions are **values** — store them, pass them, return them:

```javascript
// Regular function declaration
function greet(name) {
    return "Hello, " + name + "!";
}

// Arrow function — shorter syntax, very common in modern code
const greetArrow = (name) => "Hello, " + name + "!";

// Functions passed as arguments (callbacks):
const numbers = [3, 1, 4, 1, 5, 9];
const evens   = numbers.filter(n => n % 2 === 0);    // [4]
const doubled = numbers.map(n => n * 2);              // [6, 2, 8, 2, 10, 18]
const sum     = numbers.reduce((acc, n) => acc + n, 0);  // 23
```

**Closures** — a function remembers variables from its enclosing scope:

```javascript
function makeCounter() {
    let count = 0;
    return () => ++count;   // arrow function closes over `count`
}
const counter = makeCounter();
counter();  // 1
counter();  // 2  — count persists between calls
```

---

# The DOM — JavaScript's Bridge to the Page

The browser parses HTML into a **live tree of objects** called the **DOM**:

```
HTML source:                  DOM tree (live objects in memory):
<body>                        document
  <h1 id="title">Hi</h1>       └── body
  <ul>                                ├── h1#title  .textContent = "Hi"
    <li>Item 1</li>                   └── ul
    <li>Item 2</li>                       ├── li  .textContent = "Item 1"
  </ul>                                  └── li  .textContent = "Item 2"
</body>
```

```javascript
// Read
const h = document.getElementById("title");
console.log(h.textContent);   // "Hi"

// Modify — change is immediately visible in the browser
h.textContent = "Hello, world!";
h.style.color = "steelblue";

// Create and insert new elements
const item = document.createElement("li");
item.textContent = "Item 3";
document.querySelector("ul").appendChild(item);
```

---

# Event-Driven Programming

GUI programs are **event-driven**: nothing happens until the user does something.

```javascript
const button = document.getElementById("submit-btn");

button.addEventListener("click", function (event) {
    console.log("Clicked!", event.target);
});

// Live search — fires on every keystroke:
const search = document.getElementById("search");
search.addEventListener("input", () => filterPosts(search.value));
```

This model differs from sequential programs:

```
Sequential (C++):             Event-driven (JavaScript):
──────────────────            ───────────────────────────────
read input                    register handlers
process                       ...wait...
output                        user clicks → run click handler
done                          user types  → run input handler
                              ...wait...
```

Common events: `"click"`, `"input"`, `"submit"`, `"keydown"`, `"load"`, `"mouseover"`

---

# JavaScript Is Single-Threaded

JavaScript runs on **one thread**. Only one piece of code executes at a time.

```
┌───────────────────────┐      ┌──────────────────────────────┐
│   Call Stack          │      │   Event Queue                │
│   (running now)       │◄─────│   click handler (pending)    │
│                       │      │   fetch response (pending)   │
│   fetchPosts()        │      │   timer callback (pending)   │
└───────────────────────┘      └──────────────────────────────┘
          ▲
          └── Event loop: "Is the stack empty? Run next event."
```

**Consequences:**

- Long-running code **freezes the page** — no clicks, no rendering, nothing
- No mutexes or locks needed — only one thing runs at a time
- Slow operations (network, timers) must be **asynchronous** — they yield the thread

> `setTimeout(fn, 1000)` doesn't block for 1 second — it registers a callback and returns immediately. The callback runs 1 second later when the stack is free.

---

# Era 1: Callbacks — The Original Async Pattern

Before Promises, async code used **nested callbacks**. Every step is a function passed to the previous one:

```javascript
// XMLHttpRequest — the original way to fetch data (pre-2015)
function loadPosts(onSuccess, onError) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/api/posts/");
    xhr.onload = function () {
        if (xhr.status !== 200) {
            onError("HTTP " + xhr.status);
            return;
        }
        var data = JSON.parse(xhr.responseText);
        onSuccess(data.posts);
    };
    xhr.onerror = function () { onError("Network error"); };
    xhr.send();
}

// Now load a post, then load its comments — nested callbacks:
loadPosts(function (posts) {
    loadComments(posts[0].id, function (comments) {
        loadUser(comments[0].author, function (user) {
            render(posts[0], comments, user);  // finally!
        }, onError);
    }, onError);
}, onError);
// ↑ "Callback hell" — each level adds indentation; error handling repeats
```

Every async step nests one level deeper. With 4–5 steps this becomes unreadable — nicknamed **"callback hell"** or the **"pyramid of doom"**.

---

# Async: Promises → async/await

A blocking HTTP request would freeze the browser. JavaScript's solution: **don't wait**.

```javascript
// Era 2: Promises — chainable, composable (ES2015)
fetch("/api/posts/")
    .then(res => res.json())
    .then(data => render(data.posts))
    .catch(err => showError(err));

// Era 3: async/await — reads like synchronous code (ES2017)
async function loadPosts() {
    try {
        const res  = await fetch("/api/posts/");   // pauses here
        const data = await res.json();              // pauses here
        render(data.posts);
    } catch (err) {
        showError(err);
    }
}
```

`await` pauses **only the current async function** — not the whole thread. Other events (clicks, timers) still fire while the fetch is in progress.

---

# How `await` Actually Works

For C++ developers: `await` is similar to a **coroutine suspension point** — the function pauses and returns control to the event loop, which can run other callbacks. It resumes when the Promise resolves.

```
Time ──────────────────────────────────────────────────────────►

Thread:  [loadPosts starts]  [event loop free]  [loadPosts resumes]
                   │                │  │                │
         await fetch(...)           │  │       data = await res.json()
                   │         click! │  │timer  │
                   │         handler│  │fires  │
                   │         runs ✅│  │  ✅   │
                   │                │          │
                  fetch in progress (OS/network, not JS)
```

```javascript
async function loadPosts() {
    // Point A — running on the thread
    const res = await fetch("/api/posts/");
    // Thread is FREE here — other code runs (button clicks, timers, etc.)
    // Point B — resumes when fetch completes
    const data = await res.json();
    // Thread is FREE again while JSON is parsed
    // Point C — resumes with parsed data
    render(data.posts);
}
```

> Contrast with C++ threads: no new thread is created. The OS handles I/O; JS only runs when there's a result to process.

---

# The JavaScript Ecosystem

<div class="columns">
<div>

**Node.js (2009)**
JavaScript on the server — same language, no browser.

```bash
node server.js   # run a web server
node script.js   # run any script
```

Used for: backend services, CLI tools, build systems, test runners.

**npm — Node Package Manager**

```bash
npm install lodash
npm install --save-dev typescript esbuild
```

3 million+ packages. Dependencies listed in `package.json`; the actual code in `node_modules/` is never committed to git.

`package.json` is effectively your **CMakeLists.txt + vcpkg/Conan**: it declares project metadata, dependencies with version constraints, and build scripts (`npm run build`, `npm test`, …).

</div>
<div>

**Modules**

```javascript
// math.js — export
export function add(a, b) {
    return a + b;
}
export const PI = 3.14159;

// main.js — import
import { add, PI } from "./math.js";
```

One file per concern; explicit dependencies. No header files, no `#include`.

**ECMAScript** — the official standard. New features shipped yearly: `async/await` (ES2017), optional chaining `?.` (ES2020), `structuredClone` (ES2022), …

</div>
</div>

---

# WebAssembly — A Second Native Language

JavaScript isn't the only language the browser can run natively. **WebAssembly** (Wasm, 2017) is a binary instruction format designed as a compilation target:

```
C++ / Rust / Go source
        │
        ▼ compile (emcc / wasm-pack / tinygo)
  module.wasm   ← binary, runs at near-native speed in the browser
        │
        ▼ loaded via JavaScript
const wasm = await WebAssembly.instantiateStreaming(fetch("module.wasm"));
wasm.instance.exports.add(1, 2);   // call Wasm function from JS
```

**What it is and isn't:**

| | WebAssembly | JavaScript |
|--|--|--|
| **Written by hand?** | Rarely — it's a compiler target | Yes |
| **Speed** | Near-native (no GC pauses) | Fast (JIT), but GC overhead |
| **DOM access** | Only via JavaScript bridge | Direct |
| **Use cases** | Heavy compute, game engines, codecs, existing C++ libraries | UI, logic, APIs |

**Real uses you've seen:** Figma (rendering engine in C++), Google Meet (video codec), AutoCAD in the browser, Python/Ruby interpreters compiled to Wasm.

> As a C++ developer you can compile existing C++ code to Wasm with **Emscripten** — your algorithms run in the browser with no rewrite.

---

# JavaScript at Scale — The Problem

JavaScript works for small scripts. For large applications, dynamic typing becomes painful:

```javascript
// Typo — no error until this code path runs in production:
function processUser(user) {
    return user.profil.email;   // "profil" should be "profile"
}

// What does this function expect?
function renderPost(post) { ... }
// Is post.pubDate a string? A Date object? A Unix timestamp number?
// You must read the whole function to find out.

// Rename a field across the codebase:
// grep for "pub_date", hope you got every occurrence.
// The IDE can't help — it's all strings.
```

**Real costs:** bugs appear in production; no autocomplete on object fields; refactoring is manual and risky; new team members must read source to understand data shapes.

> This is the same problem C++ solved with type declarations. JavaScript skipped it — and is now adding it back via TypeScript.

---

# Enter TypeScript

**TypeScript** is JavaScript with a compile-time type checker bolted on top.

- Created at Microsoft by Anders Hejlsberg (also created C# and Delphi/Turbo Pascal)
- Open source since 2012; 100M+ weekly npm downloads
- A **strict superset**: every valid `.js` file is also valid `.ts`
- Types are checked at compile time, then **erased** — the browser sees plain JavaScript

```
Source                          tsc checks             Browser runs
──────────────────────────────  ─────────────────────  ──────────────
function greet(name: string) {  Error: 'number' is     (error caught;
  return "Hello, " + name;       not assignable to      nothing runs)
}                                'string'.
greet(42);

function greet(name: string) {  OK — types match       function greet(name) {
  return "Hello, " + name;                              return "Hello, " + name;
}                               (types erased) ──►     }
greet("Alice");                                         greet("Alice");
```

---

# The TypeScript Build Pipeline

A browser can't run `.ts` files. A two-tool pipeline handles compilation:

```
  TypeScript source         Type checking              Bundle for browser
  ──────────────────  ──►  ──────────────────  ──►   ──────────────────────
  src/main.ts               tsc --noEmit               esbuild → dist/main.js
  src/blog.ts               (reports type errors)      (one file, browser-ready)
```

**Why two tools?**

- **`tsc`** — official TypeScript compiler. Best type checker. Slow bundler.
- **`esbuild`** — extremely fast bundler. Strips types but skips checking.

```json
{
  "scripts": {
    "check": "tsc --noEmit",
    "build": "npm run check && esbuild src/main.ts --bundle --outfile=dist/main.js",
    "dev":   "esbuild src/main.ts --bundle --outfile=dist/main.js --sourcemap --serve"
  }
}
```

**Source maps** (`--sourcemap`): DevTools shows original `.ts` source when debugging. Set breakpoints in TypeScript, step through TypeScript — not compiled JavaScript.

---

# Type Annotations and Inference

TypeScript adds optional type annotations to JavaScript syntax:

```typescript
// Annotate function parameters and return types (recommended)
function clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
}

// TypeScript infers types from context — no annotations needed everywhere
let x = 42;               // inferred: number
let y = "hello";          // inferred: string
let z = clamp(5, 0, 10);  // inferred: number

// Errors caught at compile time:
clamp("five", 0, 10);
// Error: Argument of type 'string' is not assignable to parameter of type 'number'

// With strict: true, null is a distinct type:
const el = document.getElementById("btn");
el.click();  // Error: 'el' is possibly 'null'  (getElementById can fail)

const el = document.getElementById("btn")!;  // ! asserts non-null
el.click();  // OK
```

> **Rule of thumb:** annotate function signatures; let inference handle local variables.

---

# Interfaces — Describing Object Shapes

An **interface** names the shape of an object. TypeScript uses **structural typing** — if an object has all the required fields, it matches the interface.

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
    category: Category | null;   // Category object OR null — union types in Phase 4
}

function summarise(post: Post): string {
    return post.title + " — " + post.body.slice(0, 60) + "…";
}

// No explicit "implements Post" needed — structural compatibility:
const p = { id: 1, title: "Hello", body: "World",
            pubDate: "2025-01-01", category: null };
summarise(p);   // ✅

summarise({ id: 1, title: "Hi" });
// ❌ Error: missing 'body', 'pubDate', 'category'
```

> Interfaces are **erased at compile time** — zero bytes in the JavaScript output.

---

# Union Types and Type Guards

A **union type** says a value can be one of several types:

```typescript
type ID = number | string;

function getUser(id: ID): User { ... }
getUser(42);        // ✅
getUser("abc-123"); // ✅
getUser(true);      // ❌ Error: boolean not assignable to ID
```

> **TypeScript has `enum`** (C++ developers: you'll see it in Lab 8). But the community prefers union types because string literal unions compile to **zero JavaScript** — they're erased completely. `enum` compiles to a real JS object and adds runtime overhead. Use unions unless you have a specific reason for `enum`.

---

# Types at Trust Boundaries

TypeScript checks consistency **inside** your program. External data is always untyped:

```
                     TRUST BOUNDARY
                          │
   External World         │         Your TypeScript Code
   (untyped)              │         (type-checked)
                          │
   fetch() response  ─────┤──► parse + validate ──► typed Post
   User form input   ─────┤──► check fields     ──► trusted string
   localStorage      ─────┤──► type guard       ──► or throw error
```

```typescript
// response.json() returns Promise<any> — TypeScript trusts you blindly!
const data: Post = await response.json();  // no runtime check

// Safe approach: validate at the boundary with a type guard
function isPost(x: unknown): x is Post {
    return typeof x === "object" && x !== null
        && typeof (x as any).title === "string";
}
const raw = await response.json();
if (!isPost(raw)) throw new Error("unexpected API shape");
// TypeScript now knows raw is Post ✅
```

> In practice, libraries like **Zod** generate these guards automatically from a schema.

---

# Summary

JavaScript is the scripting language of the web — alongside WebAssembly, the only languages that run natively in browsers.

**JavaScript:**

- Syntax is C-like; semantics differ — dynamic, garbage-collected, single-threaded
- Objects are hash maps; functions are first-class values; closures capture scope
- The DOM is the live tree of HTML objects JavaScript reads and modifies
- All slow operations are async — `async/await` makes them readable
- npm gives access to a huge ecosystem; `package.json` tracks dependencies

**TypeScript:**

- Strict superset of JavaScript — any JS is valid TS; no new runtime behaviour
- Types are erased at compile time; the browser sees plain JavaScript
- `strict: true` is the baseline; enables `strictNullChecks` and more
- Interfaces describe object shapes (structural typing — matches by shape, not name)
- Union types model "one of several alternatives" with exhaustive checking
- Generics write one reusable function for any type
- `tsc` checks types; `esbuild` bundles; source maps enable TypeScript debugging

**The rule:** validate at trust boundaries — external data is always `unknown`.
