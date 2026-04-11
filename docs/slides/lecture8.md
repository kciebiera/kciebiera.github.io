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

## JavaScript & TypeScript — The Web's Language

**WWW 25/26**
JavaScript · DOM · async/await · TypeScript · types · tooling

---

# What This Lecture Is About

You know C++. You know how to write programs. This lecture introduces **JavaScript** — the language that runs in every browser — and **TypeScript**, which adds a type system on top.

- What is JavaScript and why does it exist?
- How does it fit into a web page?
- How does it differ from C++?
- What problem does TypeScript solve?
- How does the build toolchain work?

> The lab will cover TypeScript syntax exercises. Today we build the **mental model**: what JavaScript is, where it runs, and why TypeScript makes it better.

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

**JavaScript is the only programming language that runs natively in the browser.**
Everything interactive on the web — forms, menus, animations, API calls — is JavaScript.

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

  <!-- Inline script — runs immediately when the browser reaches this tag -->
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

Open any web page, press **F12** → **Console** tab. You have a live JavaScript interpreter.

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

---

# Key Differences from C++

JavaScript was designed for **quick scripts in a browser**, not systems programming:

| Feature | C++ | JavaScript |
|---------|-----|------------|
| **Types** | Static, declared | Dynamic, checked at runtime |
| **Memory** | Manual (`new`/`delete`) | Garbage collected |
| **Compilation** | To machine code | Interpreted / JIT-compiled |
| **Threads** | Multi-threaded | Single-threaded (event loop) |
| **Undefined behaviour** | Yes — dangerous | No — surprising, but defined |
| **Null access** | Crashes (segfault) | `null`/`undefined` — distinct values |
| **Numbers** | `int`, `long`, `float`, … | Just `number` (64-bit float) |
| **Strings** | `std::string` / `char*` | Built-in, immutable, Unicode |
| **Code sharing** | `.h` header files | `import` / `export` |

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
// Both are falsy, but not equal to each other:
null == undefined    // true  (loose equality)
null === undefined   // false (strict equality — different types)

// Safe "either one" check with ?? (nullish coalescing):
const name = user.name ?? "Anonymous";  // uses "Anonymous" if null OR undefined
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

> **JSON** is literally JavaScript object syntax — `JSON.stringify(post)` / `JSON.parse(text)`.

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
const greet = (name) => "Hello, " + name + "!";

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

# Closure Capture — By Reference, Not Value

C++ lambdas capture by value (`[=]`) or reference (`[&]`). JS closures always capture the **variable itself** — they see its value *when called*, not when created.

> **`var` vs `let`**: `var` is function-scoped (old JS); `let` is block-scoped like C++. This is why `var` causes the bug.

```javascript
// Classic loop closure bug — var is shared across all iterations:
const fns = [];
for (var i = 0; i < 3; i++) {
    fns.push(() => console.log(i));
}
fns[0]();  // 3  ← not 0!  (all share the same i, now 3)

// Fix: use let — creates a fresh i per iteration:
for (let i = 0; i < 3; i++) {
    fns.push(() => console.log(i));
}
fns[0]();  // 0 ✅   fns[1]();  // 1 ✅   fns[2]();  // 2 ✅
```

> **Rule:** always use `let` / `const`, never `var`.

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

# Async: Callbacks → Promises → async/await

A blocking HTTP request would freeze the browser. JavaScript's solution: **don't wait**.

```javascript
// Era 1: Callbacks — works but nests badly
fetch("/api/posts/")
    .then(res => res.json())
    .then(data => render(data.posts))
    .catch(err => showError(err));

// Era 2: Promises — same but chainable, composable (ES2015)

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
interface Post {
    id:       number;
    title:    string;
    body:     string;
    pubDate:  string;
    category: string | null;   // string OR null — covered more in Phase 4
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

# `type` vs `interface` — Which One?

Both describe types, but they have different roles:

<div class="columns">
<div>

**`interface`** — for object shapes

```typescript
interface User {
    id:   number;
    name: string;
}

// Can be extended later:
interface AdminUser extends User {
    role: "admin";
}
// Analogous to a C++ struct or
// abstract base class declaration
```

</div>
<div>

**`type`** — an alias for any type expression

```typescript
// Union of primitives:
type ID = number | string;

// Function signature:
type Handler = (e: Event) => void;

// Alias for an object:
type Point = { x: number; y: number };

// Analogous to C++:
//   using ID = int;
//   typedef void(*Handler)(Event);
```

</div>
</div>

> **Rule of thumb:** use `interface` for object shapes (especially in APIs); use `type` for unions, primitives, and function signatures. When in doubt, `interface` is the conventional choice for objects.

---

# Structural Typing — Compatibility by Shape

C++ developers are used to **nominal typing** — types are compatible only if they share an explicit base class or interface. TypeScript uses **structural typing** — any object with the right shape is compatible, no declaration needed.

<div class="columns">
<div>

**Nominal (C++/Java)**

```cpp
// Must explicitly inherit:
class Post : public Printable {
    std::string title;
};

void print(Printable& p) { ... }

struct Article { std::string title; };
Article a;
print(a);   // ❌ Compile error:
            // Article is not Printable
```

</div>
<div>

**Structural (TypeScript)**

```typescript
interface Printable {
    title: string;
}

function print(p: Printable) { ... }

const article = { title: "Hi", views: 42 };
print(article);   // ✅ — article has 'title'
                  // Extra fields are fine
                  // No "implements Printable"
                  // declaration needed
```

</div>
</div>

> This mirrors **duck typing** in Python/Ruby ("if it has a `title`, it's printable") — but enforced statically by the compiler, not discovered at runtime.

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

**Discriminated unions** — a `kind` field lets TypeScript narrow the type in each branch:

```typescript
type Shape =
    | { kind: "circle";    radius: number }
    | { kind: "rectangle"; width: number; height: number };

function area(s: Shape): number {
    switch (s.kind) {
        case "circle":    return Math.PI * s.radius ** 2;
        case "rectangle": return s.width * s.height;
        // Add "triangle" to Shape without a case here:
        // TypeScript: "Function lacks ending return statement" — exhaustiveness check
    }
}
```

> **TypeScript has `enum`** (C++ developers: you'll see it in Lab 8). But the community prefers union types because string literal unions compile to **zero JavaScript** — they're erased completely. `enum` compiles to a real JS object and adds runtime overhead. Use unions unless you have a specific reason for `enum`.

---

# Generics — One Function, Any Type

**Generics** write one function that works with any type while preserving full type safety:

```typescript
// Without generics — one function per type, lots of duplication
function firstString(arr: string[]): string | undefined { return arr[0]; }
function firstNumber(arr: number[]): number | undefined { return arr[0]; }

// With generics — T is inferred from the argument at the call site
function first<T>(arr: T[]): T | undefined {
    return arr[0];
}

first([1, 2, 3]);     // T = number, returns number | undefined  ✅
first(["a", "b"]);    // T = string, returns string | undefined  ✅

// Useful for collections over domain types:
function groupBy<T>(
    items: T[],
    keyFn: (item: T) => string
): Record<string, T[]> {
    const groups: Record<string, T[]> = {};
    for (const item of items) {
        const k = keyFn(item);
        (groups[k] ??= []).push(item);
    }
    return groups;
}
groupBy(posts, p => p.category ?? "none");  // T inferred as Post
```

---

# The Toolchain in Practice

A TypeScript project has a few configuration files:

```json
// tsconfig.json — tells tsc how to check your code
{
  "compilerOptions": {
    "target": "ES2020",   // output JS version
    "strict": true,       // enable all strict checks — always use this
    "noEmit": true,       // only type-check; esbuild handles output
    "sourceMap": true     // generate .js.map for DevTools
  },
  "include": ["src/**/*"]
}
```

**`strict: true`** enables the most important checks:

- `strictNullChecks` — `null` and `undefined` are distinct types; must be handled explicitly
- `noImplicitAny` — every value must have a known type (no silent `any`)

```typescript
// Without strictNullChecks: getElementById returns HTMLElement
// With strictNullChecks:    getElementById returns HTMLElement | null

// You must handle the null case — or the compiler won't let you proceed.
```

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

JavaScript is the language of the web — the only language that runs natively in browsers.

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
