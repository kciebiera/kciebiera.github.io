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

## JavaScript & TypeScript - The Browser's Programming Model

**WWW 25/26**
JavaScript - DOM - events - async/await - TypeScript

---

# What This Lecture Is About

You already know how to program. Today is about understanding **how the browser thinks**.

We will build one mental model:

- the page is a data structure
- JavaScript reacts to events
- slow work must be asynchronous
- TypeScript makes larger JavaScript programs safer

> The goal is not to memorise syntax.
> The goal is to understand **page + DOM + events + async + types**.

---

# The Three-Layer Web Stack

```text
+---------------------------------------------+
| HTML - structure                            |
| <h1>Hello</h1>  <button>Save</button>       |
| What exists on the page                     |
+---------------------------------------------+
| CSS - appearance                            |
| h1 { color: red; }                          |
| What it looks like                          |
+---------------------------------------------+
| JavaScript - behaviour                      |
| button.onclick = ...                        |
| What it does over time                      |
+---------------------------------------------+
```

JavaScript is the browser's built-in programming language.
It reads the page, changes the page, and reacts to what the user does.

---

# What the Browser Actually Does

When you open a page, the browser roughly does this:

1. downloads HTML
2. builds a **DOM** tree in memory
3. downloads CSS and applies styles
4. executes JavaScript
5. renders pixels to the screen

After that, the page stays alive and reacts to events.

---

# The `<script>` Tag and Timing

```html
<!DOCTYPE html>
<html>
<body>
  <h1 id="greeting">Hello</h1>
  <button id="btn">Click me</button>

  <script>
    document.getElementById("btn").addEventListener("click", () => {
      document.getElementById("greeting").textContent = "You clicked!";
    });
  </script>
</body>
</html>
```

The browser reads HTML from top to bottom.
When it reaches a `<script>`, it runs JavaScript.
That is why script placement or `defer` matters.

---

# JavaScript for C++ Developers

<div class="columns">
<div>

**C++**

```cpp
int add(int a, int b) {
    return a + b;
}
```

</div>
<div>

**JavaScript**

```javascript
function add(a, b) {
    return a + b;
}
```

</div>
</div>

The syntax looks familiar.
The important differences are in the **runtime model**:

- dynamic typing
- garbage collection
- event-driven execution
- usually one main thread for page code

---

# Values Have Types, Variables Do Not

JavaScript is **dynamically typed**.

```javascript
let x = 42;
x = "hello";
x = true;
```

The variable name `x` is not tied to one declared type.
The **current value** has a type.

```javascript
typeof 42        // "number"
typeof "hi"      // "string"
typeof true      // "boolean"
typeof undefined // "undefined"
typeof null      // "object"  // historical quirk
```

---

# JavaScript Will Coerce for You

JavaScript often converts values automatically:

```javascript
"5" + 1    // "51"
"5" - 1    // 4
"5" == 5   // true
"5" === 5  // false
```

Mental model:

- JavaScript tries hard to keep going
- that is convenient for small scripts
- that is dangerous in larger programs

> Prefer `===` and `!==`.

---

# Objects, Arrays, and Bindings

```javascript
const post = {
  id: 1,
  title: "Hello",
  published: true,
};

const nums = [10, 20, 30];
```

Mental model:

- **object** = a bag of named properties
- **array** = an ordered list of values
- `const` protects the **binding**, not the contents

```javascript
post.body   // undefined
nums[10]    // undefined
post.title = "Updated";  // allowed
```

---

# Functions and Closures

Functions are values you can store and pass around.

```javascript
const greet = name => "Hello, " + name;
const doubled = [1, 2, 3].map(n => n * 2);
```

Closures let functions remember variables from where they were created:

```javascript
function makeCounter() {
  let count = 0;
  return () => ++count;
}
```

This is why callbacks and event handlers feel natural in JavaScript.

---

# `null` and `undefined`

JavaScript has two common "missing value" states.

```javascript
let user = null;   // deliberately empty
let x;             // undefined: never assigned
```

```javascript
null == undefined   // true
null === undefined  // false
```

Use modern operators to handle absence safely:

```javascript
const name = user?.profile?.name ?? "Anonymous";
```

---

# The DOM - The Page as Data

The browser turns HTML text into a **DOM**: a live tree of objects in memory.

```text
document
  body
    h1#title
    ul
      li
      li
```

Mental model:

- HTML is text
- DOM is the in-memory object model JavaScript works with

---

# Reading and Writing the DOM

```javascript
const title = document.getElementById("title");

if (title) {
  console.log(title.textContent);
  title.textContent = "Hello, world!";
  title.style.color = "steelblue";
}
```

When you change the DOM, you change what the browser will render.

---

# Creating New Elements

```javascript
const item = document.createElement("li");
item.textContent = "Item 3";

document.querySelector("ul")?.appendChild(item);
```

DOM nodes are regular objects.
You can create them, modify them, move them, and remove them.

---

# Events Drive the Program

In browser programming, most code does not run once from top to bottom.
It mostly waits.

```javascript
const button = document.getElementById("save-btn");
button?.addEventListener("click", event => {
  console.log("Saved", event.target);
});
```

Common events: `click`, `input`, `submit`, `keydown`, `load`

---

# Sequential vs Event-Driven Thinking

```text
Sequential program:
read input -> process -> output -> done

Browser program:
register handlers -> wait
user clicks -> run click handler
user types  -> run input handler
network returns -> run callback
wait again
```

This is the biggest mindset shift from many C++ console programs.

---

# One Main Thread and the Event Loop

For normal page code, JavaScript runs on the browser's **main thread**.
Only one piece of page JavaScript runs at a time.

```text
+-------------------+      +--------------------------+
| Call stack        |      | Pending work queue       |
| code running now  | <--- | click handler            |
|                   |      | timer callback           |
|                   |      | fetch continuation       |
+-------------------+      +--------------------------+

Event loop:
if the stack is empty, run the next pending task
```

Long-running code freezes the page.

---

# Why Async Exists

Some operations are slow:

- network requests
- timers
- file access
- waiting for user input

If JavaScript blocked while waiting, the whole page would become unresponsive.
So browser APIs start work now and continue later.

---

# `async` / `await`

```javascript
async function loadPosts() {
  const res = await fetch("/api/posts/");
  const data = await res.json();
  render(data.posts);
}
```

Mental model:

- start asynchronous work
- pause **this function**
- let the browser do other work
- resume when the result is ready

---

# What Happens During `await`

```text
loadPosts starts
  -> await fetch(...)
  -> function pauses
  -> event loop can run other handlers
  -> network reply arrives
  -> loadPosts resumes
```

`await` does not freeze the whole page.
It suspends the current async function.

---

# Modules and Code Organisation

Real applications are not one giant script.
Code is split into files with explicit imports.

```javascript
// math.js
export function add(a, b) {
  return a + b;
}

// main.js
import { add } from "./math.js";
```

Each file states what it needs and what it provides.

---

# Why Plain JavaScript Gets Hard at Scale

JavaScript is convenient, but large programs need stronger guarantees.

```javascript
function processUser(user) {
  return user.profil.email;
}
```

This typo may survive until runtime.
Other problems: unclear data shapes, risky refactors, weaker autocomplete.

---

# TypeScript in One Sentence

TypeScript is **JavaScript plus a static checker**.

```typescript
function greet(name: string): string {
  return "Hello, " + name;
}
```

The checker finds mistakes before runtime.
The browser still runs plain JavaScript.

---

# TypeScript Does Not Change the Runtime

```text
TypeScript source -> type checking -> JavaScript output -> browser
```

Mental model:

- types help the developer and the editor
- types are removed before execution
- TypeScript improves correctness, not browser capabilities

---

# Annotations and Inference

```typescript
function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value));
}

let x = 42;       // inferred as number
let y = "hello"; // inferred as string

clamp("five", 0, 10); // error
```

Annotate function boundaries; let inference handle many local variables.

---

# Types Describe Shapes

```typescript
interface Post {
  id: number;
  title: string;
  slug: string;
  body: string;
  pubDate: string;
}

function summarise(post: Post): string {
  return post.title + " - " + post.body.slice(0, 60) + "...";
}
```

TypeScript lets you state what kind of object a function expects.

---

# Types Stop at the Boundary

TypeScript checks your code, but it does not magically validate external data.

Untrusted inputs include:

- HTTP responses
- user form input
- localStorage
- third-party APIs

Inside your program, types are promises.
At the boundary, you still need validation.

---

# A Safer Boundary Pattern

```typescript
interface Post {
  title: string;
}

function isPost(x: unknown): x is Post {
  return typeof x === "object" && x !== null
      && typeof (x as { title?: unknown }).title === "string";
}

const raw: unknown = await response.json();
if (!isPost(raw)) throw new Error("Unexpected API shape");
```

Treat outside data as `unknown` until checked.

---

# Minimal Toolchain Picture

In a typical project:

- `tsc` checks types
- a bundler prepares browser-ready JavaScript
- source maps let DevTools point back to your original code

You do not need to memorise the tooling today.
The key idea is: **TypeScript is checked first, then turned into JavaScript**.

---

# Summary

Mental model of modern frontend JavaScript:

- the browser turns HTML into a live DOM tree
- JavaScript reads and changes that tree
- programs mostly wait for events
- the event loop schedules work on one main thread
- async APIs keep the page responsive
- TypeScript adds static checking for larger codebases

> JavaScript is not just a language.
> It is the browser's event-driven programming model.
