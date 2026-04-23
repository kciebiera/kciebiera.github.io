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

# Lecture 9

## Event-Driven Pages and Event Streams

**WWW 25/26**
browser runtime · DOM · events · delays · scheduling · SSE

---

# What This Lecture Is About

Lecture 8 introduced the browser's programming model.
This lecture focuses on what happens after the page is loaded.

We will focus on ideas:

- why the browser is fundamentally **event-driven**
- where delays come from
- how one action can trigger many reactions
- how user input, timers, network responses, and server push fit one model
- why **Server-Sent Events (SSE)** are conceptually simple

> The goal is not API memorisation.
> The goal is to understand how an interactive page lives over time.

---

# Three Actors

```text
User    -> clicks, types, scrolls, waits
Browser -> DOM, timers, rendering, event loop
Server  -> HTML, JSON, streamed events
```

An interactive page is a conversation between these three actors over time.

- the user produces intentions
- the server produces data
- the browser mediates the whole exchange

---

# The Browser Is a Host Environment

JavaScript the language is only part of the story.
In the browser, your program runs inside a **host environment** that provides extra objects:

| Provided by browser | Purpose |
|---------------------|---------|
| `document` | access the DOM |
| `window` | global browser context |
| `console` | debugging |
| `fetch` | HTTP requests |
| `setTimeout` | timers |
| `requestAnimationFrame` | schedule work before repaint |
| `EventTarget` | event subscription model |
| `EventSource` | SSE connection |

In Node.js, many of these are different or historically absent.

> Same language, different runtime.

---

# The Browser Connection in One Diagram

```text
HTML ──► DOM tree ──► JavaScript reads/writes DOM
                      ▲
                      │
                your program code
                      ▲
                      │
                 source files
```

So the real chain is:

1. you write code
2. tools prepare it for the browser
3. the browser runs that JavaScript
4. that code talks to browser APIs
5. browser APIs connect your code to the page, the user, and the network

---

# The Browser Is an Event Machine

After page load, the browser mostly waits.

```text
wait for something to happen
    │
    ├─ user clicks
    ├─ user types
    ├─ timer fires
    ├─ fetch completes
    ├─ SSE message arrives
    └─ browser repaints
```

Your code does not run continuously like a physics simulation loop by default.
It runs in **small reaction steps** when events happen.

This is the key mental shift from batch programming:

- not "run algorithm from top to bottom"
- but "register reactions, then let the browser wake them up"

---

# The Browser Is an Event Machine: Example

Example page lifecycle:

1. page loads
2. user types `"dj"`
3. browser fires `input`
4. code starts `fetch("/api/posts/?search=dj")`
5. server replies later
6. browser wakes your continuation
7. UI updates

The important point is that steps 3 and 6 happen at different times.

---

# Events Are the Unifying Abstraction

At first these look unrelated:

- button click
- keyboard input
- timer callback
- HTTP response
- incoming SSE message

But conceptually they are the same:

> **Something happened outside your current function, so the runtime notifies you later.**

That is what event-driven programming means.

---

# Events Are the Unifying Abstraction

You describe:

- what to listen to
- what handler to run
- how to update program state when it happens

```javascript
button.addEventListener("click", () => console.log("clicked"));
input.addEventListener("input", () => console.log("typed"));
setTimeout(() => console.log("timer fired"), 1000);
```

Different APIs, same idea: register work now, run it later.

---

# Event Sources Around the Page

```text
User ───────────────► click / input / keydown
Browser ────────────► load / resize / visibilitychange
Timer system ───────► timeout / interval callback
HTTP stack ─────────► fetch promise resolves
Server push ────────► SSE message event
```

Different source, same shape:

```text
source produces signal
        ▼
browser/runtime queues work
        ▼
your handler runs
        ▼
you update state / DOM
```

---

# Event Sources Around the Page: Examples

Concrete examples:

- `click` on "Load posts" button
- `input` in a search field
- `visibilitychange` when user switches tabs
- `fetch(...).then(...)` after server response
- `message` from `EventSource` when backend pushes update

---

# Event Loop: The Traffic Controller

JavaScript in the page is usually **single-threaded**.
The browser handles concurrency by scheduling work through the event loop.

```text
external world -> browser APIs -> queue -> handler runs -> queue -> handler runs
```

This gives a powerful illusion:

- many things happen "at once"
- your code still runs one handler at a time

That simplifies reasoning:

- no shared-memory races in normal page code
- but long handlers block everything else

> Responsiveness depends on handlers being short.

---

# Event Loop: Blocking Example

Example:

```javascript
button.addEventListener("click", () => {
  for (let i = 0; i < 1_000_000_000; i++) {}
  console.log("done");
});
```

While this loop runs, the page cannot respond smoothly to typing, clicks, or repainting.

---

# Where Delays Come From

In browser programs, "later" has many causes:

- the user pauses before acting again
- a timer waits on purpose
- the network takes unpredictable time
- the browser waits to repaint
- your own code blocks the main thread

> Delay is normal in an interactive system, not an exception.

---

# `setTimeout(fn, 0)` Still Means "Later"

```javascript
console.log("A");
setTimeout(() => console.log("B"), 0);
console.log("C");
```

Output:

```text
A
C
B
```

The callback is queued and runs only after current work finishes.

---

# Events Carry Data Across Boundaries

An event is not just "something happened."
It usually carries information from the outside world.

Examples:

- `input` event: current text field contents
- `click` event: target element, mouse coordinates
- `submit` event: form submission intent
- resolved `fetch`: response bytes from server
- SSE `message` event: text payload from the server

So events are how the outside world injects **new facts** into your program.

---

# Events Carry Data Across Boundaries: Example

```javascript
input.addEventListener("input", event => {
  console.log(event.target.value);
});
```

The event is the carrier; the new text is the payload you care about.

---

# One Click, Multiple Targets

One physical click can trigger logic at several levels of the DOM tree.

```html
<div id="card">
  <button id="delete-btn">Delete</button>
</div>
```

```javascript
card.addEventListener("click", () => console.log("card clicked"));
deleteBtn.addEventListener("click", () => console.log("button clicked"));
document.body.addEventListener("click", () => console.log("body clicked"));
```

If the user clicks the button, all three handlers may run.

Why?

- the **target** is the button
- the event then **bubbles upward**
- parent elements can react too

---

# One Click, Multiple Targets: Why It Matters

This is useful, but it can also surprise you.

Examples:

- button handler deletes an item
- card handler opens the detail view
- page-level handler records analytics

So one click may mean:

```text
button action + parent action + global logging
```

This is why frontend code often checks:

- `event.target`
- `event.currentTarget`
- whether propagation should continue

> A click is not just a point in space.
> It travels through the DOM as an event.

---

# The Core UI Pattern

The browser gives you events.
Your job is to turn them into state changes and visible output.

```text
event -> interpret -> update state -> render UI
```

Examples:

- input event -> update `query` -> re-render filtered list
- click event -> toggle `expandedPostId` -> re-render article
- fetch result -> store `posts` -> render loaded page
- SSE message -> append notification -> render feed

---

# The Core UI Pattern: Example

Mini example:

```javascript
input.addEventListener("input", event => {
  state.query = event.target.value;
  render();
});
```

This is why events and state belong together conceptually.

---

# Delays Create Ordering Problems

The hardest part of async code is often not waiting, but **ordering**.

```text
User types:        d -------- dj
Requests sent:     A -------- B
Responses arrive:           B -------- A
```

If you blindly render every response, old data can overwrite new data.

> Later request does not guarantee later response.

---

# Debounce and Throttle

Delay is not always a bug.
Sometimes delay is a **tool**.

<div class="columns">
<div>

**Debounce**

- wait until activity stops
- useful for search boxes
- "send one request 300ms after typing stops"

</div>
<div>

**Throttle**

- run at most once per interval
- useful for scroll and resize
- "update at most every 100ms"

</div>
</div>

Designing good interactive systems often means choosing **which delays are useful**.

---

# A Useful Distinction: Pull vs. Push

How does the browser learn that the server has new data?

Two broad models:

<div class="columns">
<div>

**Pull**

Client asks:

```text
"Anything new?"
```

Examples:

- page reload
- `fetch()`
- polling every 5 seconds

</div>
<div>

**Push**

Server tells client:

```text
"Here is new data."
```

Examples:

- SSE
- WebSockets
- push notifications

</div>
</div>

This distinction matters more than any specific API.

---

# Polling: Simulating Events by Repeated Questions

Polling means:

```text
every N seconds:   ask server for updates
```

Conceptually, polling turns time into a fake event source:

```text
timer fires -> fetch -> maybe new data
```

Why it is attractive:

- easy to understand
- plain HTTP
- easy to debug

Why it is unsatisfying:

- many requests return nothing new
- latency depends on interval
- server and battery cost grow with frequency

---

# Polling: Concrete Example

```javascript
setInterval(async () => {
  const res = await fetch("/api/notifications");
  const data = await res.json();
  updateUI(data);
}, 5000);
```

Mental model:

```text
timer fires -> fetch -> maybe new data
```

---

# SSE: Turning Server Updates Into Browser Events

**Server-Sent Events** let the server keep an HTTP connection open and send messages over time.

From the browser's point of view:

```text
open connection once
        │
server sends message
        │
browser fires "message" event
        │
your handler runs
```

That is the key idea:

> SSE makes server updates look like ordinary browser events.

This is why SSE feels conceptually elegant.

---

# SSE vs. Traditional Fetch

```text
traditional fetch:
client asks once -> server answers once

SSE:
client connects once -> server answers many times
```

---

# Why SSE Fits the Browser So Well

SSE is a good conceptual match for many web applications because it preserves the browser's existing model:

- still HTTP
- still text messages
- still event listeners
- still one-way from outside world into handlers

```javascript
const source = new EventSource("/stream");
source.onmessage = event => {
  // treat incoming server data like any other event
};
```

---

# Why SSE Fits the Browser So Well

Another example:

```javascript
const source = new EventSource("/api/build-log");

source.addEventListener("message", event => {
  console.log("new log line:", event.data);
});
```

Mentally, this is close to:

```javascript
button.addEventListener("click", ...)
input.addEventListener("input", ...)
source.addEventListener("message", ...)
```

---

# SSE Is Not "Real-Time Magic"

SSE does not change the programming model.
It only adds a **new event source**.

Before SSE:

```text
user events + timers + fetch completions
```

After SSE:

```text
user events + timers + fetch completions + server message events
```

So the important idea is not the API name.
The important idea is that the browser can unify many asynchronous sources under one event-driven model.

---

# Typical SSE Use Cases

SSE works best when the server mainly needs to **announce updates**.

Examples:

- notification feed
- build logs
- stock prices
- monitoring dashboard
- job progress updates
- AI text streaming

These all share one pattern:

```text
server produces a sequence of facts
client displays them as they arrive
```

More concrete examples:

- CI server streams test output line by line
- blog admin panel streams "new comment arrived"
- background import job streams progress: `10%`, `20%`, `30%`
- AI assistant streams generated tokens one chunk at a time

That is naturally a stream.

---

# SSE vs. WebSockets

The conceptual difference is simple:

| Technology | Direction | Best mental model |
|-----------|-----------|-------------------|
| `fetch()` | request -> response | one question, one answer |
| Polling | repeated request -> response | repeated questions |
| **SSE** | server -> client stream | event feed |
| WebSocket | both directions, ongoing | conversation |

If the client mostly listens, SSE is often enough.
If both sides speak freely at any time, WebSockets fit better.

> Do not choose the most powerful tool first.
> Choose the simplest model that matches the communication pattern.

---

# The Deeper Pattern: Everything Becomes a Stream

A mature frontend eventually treats many things as streams of values over time:

- mouse positions
- search queries
- route changes
- HTTP results
- SSE messages
- authentication state

```text
value now, then later another value, then later another value
```

This is the bridge to ideas from RxJS, reactive UI, and functional reactive programming.

You do not need those frameworks to understand the core insight:

> interactive programs are about **data changing over time**.

---

# The Real Architectural Boundary

When building browser applications, there are really three layers:

```text
outside world
  user / browser / server
        │
        ▼
events and data arrive
        │
        ▼
your program logic interprets them
        │
        ▼
state and DOM are updated
```

This suggests a clean design rule:

- treat the outside world as **untrusted input**
- translate it into explicit internal state
- render from that state

That rule works for clicks, forms, JSON, and SSE messages alike.

---

# Common Misunderstandings

1. "`setTimeout(fn, 0)` runs immediately."
   No. It only schedules work for later.

2. "A click only belongs to one element."
   No. Events can bubble through multiple levels of the DOM.

3. "The newest request always returns last."
   No. Network delays can reorder responses.

4. "SSE is a completely different paradigm."
   No. It is just another event source.

---

# Design Heuristics

- Model the page as a set of event sources.
- Keep handlers short and focused.
- Make delays explicit: timers, retries, debouncing.
- Assume network responses can arrive out of order.
- Convert outside input into explicit internal state.
- Prefer simple event flows over scattered DOM mutation.
- Start with `fetch()` or polling.
- Move to SSE when the server mainly needs to push updates.
- Use WebSockets only when true two-way communication matters.

---

# Summary

- The browser is an **event-driven runtime**
- Interactive pages are built from **event sources** and delayed reactions
- One action can trigger multiple handlers through **propagation**
- Delays come from timers, CPU work, rendering, and the network
- Polling simulates events; SSE gives you server-to-client event streams

---

# Questions?
