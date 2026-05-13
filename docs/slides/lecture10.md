---
marp: true
theme: default
paginate: true
style: |
  section { font-size: 1.35rem; }
  pre { font-size: 0.78rem; }
  h1 { color: #092e20; }
  h2 { color: #44b78b; }
  .columns { display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; }
  .wide-code pre { font-size: 0.68rem; }
  code { background: #f5f5f5; padding: 0.1em 0.3em; border-radius: 3px; }
---
{% raw %}

# Lecture 10

## Vue as a Runtime for Interactive Programs

**WWW 25/26**
state - components - reactivity - routing - framework tradeoffs

---

# What This Lecture Is About

We will use Vue to study ideas that appear in every modern frontend framework.

- What is **state**?
- How do page elements interact without global chaos?
- How can JavaScript update the page without manually calling `appendChild` everywhere?
- How can a framework "take over" buttons and URLs?
- What do frameworks actually buy us, and what do they cost?

> Vue is the example. The concepts transfer to React, Svelte, Solid, Angular, and most UI runtimes.

---

# Sample Application: Task Board

We will reason about a small app:

```text
/                 dashboard
/tasks            all tasks
/tasks/42         task details
/settings         user preferences
```

Features:

- list tasks
- filter by status
- mark a task as done
- open a task detail view without full page reload
- keep selected filter in the URL
- render from one source of truth

This is small enough to fit in one lecture but rich enough to expose the machinery.

Live demo for the lecture:

```text
docs/vue-task-board/
```

---

# The Core Problem

HTML describes a starting tree.
CSS describes appearance.

But an application changes over time:

```text
user clicks button
      |
      v
program state changes
      |
      v
visible UI must change consistently
```

Without a framework, you must decide manually which DOM nodes to update after every possible event.

---

# Manual DOM Programming Does Not Scale Nicely

```javascript
button.addEventListener("click", () => {
  task.done = true;

  row.classList.add("done");
  button.disabled = true;
  counter.textContent = String(countDone(tasks));
  emptyMessage.hidden = tasks.length > 0;
  detailsPanel.textContent = task.title;
});
```

This is not "wrong".

The problem is that the same state is now encoded in many places:

- JavaScript objects
- classes on DOM nodes
- text content
- disabled flags
- hidden panels

---

# Framework Mental Model

Vue asks you to write:

```text
state -> template -> DOM
```

Instead of:

```text
event -> find node -> mutate node -> remember all consequences
```

Your job:

- define the state
- define how UI is derived from state
- define events that change state

Vue's job:

- track what depends on what
- update the DOM when dependencies change
- keep components and routing connected

---

# What Is State?

**State** is the minimum data needed to answer:

> If I redraw the UI now, what should it show?

For the task board:

```typescript
type Task = {
  id: number;
  title: string;
  done: boolean;
};

type AppState = {
  tasks: Task[];
  filter: "all" | "open" | "done";
  selectedTaskId: number | null;
};
```

State is memory that affects future rendering.

---

# State vs Derived Data

Do not store what you can compute.

```typescript
const tasks = ref<Task[]>([]);
const filter = ref<"all" | "open" | "done">("all");

const visibleTasks = computed(() => {
  if (filter.value === "open") return tasks.value.filter(t => !t.done);
  if (filter.value === "done") return tasks.value.filter(t => t.done);
  return tasks.value;
});

const doneCount = computed(() =>
  tasks.value.filter(t => t.done).length
);
```

`visibleTasks` and `doneCount` are **derived state**.
They should stay consistent automatically.

---

# Vue Single File Component

```vue
<!-- TaskList.vue -->
<script setup lang="ts">
import type { Task } from "./types";

defineProps<{ tasks: Task[] }>();
const emit = defineEmits<{
  toggle: [id: number];
  open: [id: number];
}>();
</script>

<template>
  <ul>
    <li v-for="task in tasks" :key="task.id">
      <button @click="emit('toggle', task.id)">
        {{ task.done ? "Undo" : "Done" }}
      </button>
      <a href="#" @click.prevent="emit('open', task.id)">
        {{ task.title }}
      </a>
    </li>
  </ul>
</template>
```

---

# What Is a Component?

A component is a function-like unit of UI:

```text
props + internal state + template + event handlers
                  |
                  v
              rendered DOM
```

Components are not "HTML includes".
They are program objects with:

- inputs: `props`
- outputs: emitted events
- private state
- lifecycle
- dependencies

Good component design is mostly data-flow design.

---

# Component Tree

```text
App
 |
 +-- AppHeader
 |
 +-- RouterView
 |    |
 |    +-- TaskPage
 |         |
 |         +-- FilterTabs
 |         +-- TaskList
 |         +-- TaskStats
 |
 +-- AppFooter
```

The DOM tree is what the browser renders.
The component tree is how the program is organized.

They are related, but not identical.

---

# Props Down, Events Up

```text
TaskPage owns tasks
     |
     | props
     v
TaskList displays tasks
     |
     | emits "toggle"
     v
TaskPage mutates tasks
```

This keeps ownership clear:

- child does not secretly mutate parent state
- parent decides what the event means
- data flow is visible in component boundaries

This is a software engineering pattern, not just Vue syntax.

---

# Parent Component

```vue
<!-- TaskPage.vue -->
<script setup lang="ts">
import { ref, computed } from "vue";
import TaskList from "./TaskList.vue";

const tasks = ref([
  { id: 1, title: "Read API docs", done: false },
  { id: 2, title: "Implement form", done: true },
]);

function toggleTask(id: number) {
  const task = tasks.value.find(t => t.id === id);
  if (task) task.done = !task.done;
}
</script>

<template>
  <TaskList :tasks="tasks" @toggle="toggleTask" />
</template>
```

The child asks for something to happen.
The parent changes the state.

---

# How Buttons Are "Hijacked"

Vue does not magically own buttons.

The browser still owns the event system.
Vue compiles this:

```vue
<button @click="toggleTask(task.id)">Done</button>
```

into JavaScript roughly like this:

```javascript
button.addEventListener("click", event => {
  componentContext.toggleTask(componentContext.task.id);
});
```

Vue provides a declarative syntax for normal browser event listeners.

---

# Event Modifiers Are Also JavaScript

```vue
<form @submit.prevent="saveTask">
  ...
</form>
```

Roughly means:

```javascript
form.addEventListener("submit", event => {
  event.preventDefault();
  saveTask(event);
});
```

Important point:

- `.prevent` prevents the browser's default form submission
- the page does not reload
- your JavaScript handles the event
- state changes cause DOM updates

---

# What Actually Happens on Click?

```text
User clicks button
      |
Browser creates MouseEvent
      |
Event travels through DOM
      |
Vue-installed listener runs
      |
Your handler mutates reactive state
      |
Vue schedules component update
      |
DOM is patched
      |
Browser paints new pixels
```

Vue is layered on top of browser primitives.
It does not replace the browser.

---

# Reactivity: The Big Idea

Vue needs to know:

> Which rendered output depends on which state?

If `doneCount` depends on `tasks`, then changing `tasks` should invalidate `doneCount`.

If the task list template depends on `visibleTasks`, then changing `filter` should update the list.

This is a dependency graph:

```text
tasks -----> doneCount -----> TaskStats render
   |
   +-------> visibleTasks --> TaskList render

filter ----> visibleTasks
```

---

# Reactivity in Plain JavaScript

JavaScript has `Proxy`, which lets code intercept property access.

```javascript
const state = new Proxy({ count: 0 }, {
  get(target, key) {
    console.log("read", key);
    return target[key];
  },
  set(target, key, value) {
    console.log("write", key, value);
    target[key] = value;
    return true;
  }
});

state.count;      // read count
state.count = 1;  // write count 1
```

Vue uses this idea for reactive objects.

---

# Dependency Tracking, Simplified

```javascript
let activeEffect = null;
const deps = new Map();

function effect(fn) {
  activeEffect = fn;
  fn();
  activeEffect = null;
}

function track(key) {
  if (!activeEffect) return;
  let set = deps.get(key) ?? new Set();
  set.add(activeEffect);
  deps.set(key, set);
}

function trigger(key) {
  deps.get(key)?.forEach(fn => fn());
}
```

This is the core concept behind reactive UI systems.

---

# Proxy + Effects

```javascript
const state = new Proxy({ count: 0 }, {
  get(target, key) {
    track(key);
    return target[key];
  },
  set(target, key, value) {
    target[key] = value;
    trigger(key);
    return true;
  }
});

effect(() => {
  console.log("render:", state.count);
});

state.count++;
```

The first render records the dependency.
The later write re-runs the effect.

---

# Vue Rendering Is an Effect

Conceptually:

```javascript
effect(() => {
  const virtualTree = render(componentState);
  patch(realDom, virtualTree);
});
```

A component render reads reactive values.
Vue records those reads.

When one of those values changes, Vue knows which component render effects are dirty.

This is why you do not manually call `render()` after each click.

---

# Template Compilation

Vue templates are not interpreted as strings forever.
They are compiled to render functions.

```vue
<h1>{{ title }}</h1>
<button @click="count++">{{ count }}</button>
```

Roughly becomes:

```javascript
function render(ctx) {
  return h("div", [
    h("h1", ctx.title),
    h("button", { onClick: () => ctx.count++ }, ctx.count)
  ]);
}
```

The template is a nicer syntax for JavaScript that builds UI descriptions.

---

# Virtual DOM, Briefly

Vue does not rebuild the entire real DOM on every change.

It builds a lightweight description:

```javascript
{
  type: "button",
  props: { disabled: false },
  children: "Done"
}
```

Then it compares old and new descriptions and patches only necessary DOM operations.

```text
old virtual tree + new virtual tree -> minimal useful DOM mutations
```

This is an implementation strategy, not the purpose of Vue.

---

# Keys Matter

```vue
<li v-for="task in tasks" :key="task.id">
  {{ task.title }}
</li>
```

The key tells Vue object identity across renders.

Without stable keys, Vue may reuse DOM nodes incorrectly when:

- items are inserted in the middle
- lists are sorted
- form inputs live inside repeated rows
- animations depend on identity

For CS students: this is a matching problem between old and new sequences.

---

# Scheduler: Why Updates Are Batched

If one handler changes three fields:

```javascript
task.done = true;
filter.value = "done";
selectedTaskId.value = task.id;
```

Vue should not render three times.

It batches invalidated effects and flushes them later:

```text
state writes
   |
mark components dirty
   |
queue microtask
   |
run render effects once
```

This is why DOM updates are often visible after `await nextTick()`.

---

# `nextTick`

```typescript
import { nextTick } from "vue";

async function selectTask(id: number) {
  selectedTaskId.value = id;

  await nextTick();

  document.querySelector("#details")?.scrollIntoView();
}
```

State changes synchronously.
DOM patching is scheduled.

Use `nextTick` only when you need to touch the updated DOM directly.
Most component code should not need it.

---

# Local State vs Shared State

Local state belongs to one component:

```typescript
const isEditing = ref(false);
const draftTitle = ref("");
```

Shared state affects multiple distant components:

```text
TaskPage
  FilterTabs needs current filter
  TaskList needs visible tasks
  TaskStats needs counts
  Header needs number of open tasks
```

Options:

- lift state to the nearest common parent
- use `provide` / `inject`
- use a store such as Pinia
- encode some state in the URL

---

# URL Is Also State

For the task board, this is state:

```text
/tasks?filter=open
/tasks/42
```

URL state is useful because it is:

- shareable
- bookmarkable
- reload-safe
- visible to the user
- integrated with browser history

If the user copies the URL, the app should reconstruct the same meaningful view.

---

# How Vue Router "Hijacks" URLs

Traditional link:

```html
<a href="/tasks/42">Open</a>
```

Browser default:

```text
send HTTP request -> receive HTML -> reload page
```

Vue Router behavior:

```text
prevent default navigation
call history.pushState(...)
update current route object
render matched component
```

No full page reload is needed because JavaScript changes the view.

---

# Browser History API

Vue Router is built on browser APIs.

```javascript
history.pushState({ id: 42 }, "", "/tasks/42");

window.addEventListener("popstate", event => {
  console.log("user pressed back or forward");
});
```

`pushState` changes the URL bar without loading a new document.
`popstate` tells JavaScript when the user uses back/forward.

The router adds:

- route matching
- nested routes
- params
- navigation guards
- link components

---

# Router Configuration

```typescript
import { createRouter, createWebHistory } from "vue-router";

const router = createRouter({
  history: createWebHistory(),
  routes: [
    { path: "/", component: DashboardPage },
    { path: "/tasks", component: TaskPage },
    { path: "/tasks/:id", component: TaskDetailsPage },
    { path: "/settings", component: SettingsPage },
  ],
});
```

`/tasks/:id` is a pattern.
`/tasks/42` produces `route.params.id === "42"`.

The server must still return the app for these URLs in production.

---

# RouterLink

```vue
<RouterLink :to="`/tasks/${task.id}`">
  {{ task.title }}
</RouterLink>
```

This renders an anchor tag, but intercepts clicks.

It must still behave like a real link when appropriate:

- right click -> open in new tab
- copy link address
- keyboard navigation
- browser history

Good frameworks preserve web semantics.
Bad code often destroys them.

---

# Route Object as Reactive State

```typescript
import { useRoute } from "vue-router";

const route = useRoute();

const selectedTaskId = computed(() =>
  Number(route.params.id)
);
```

The current route is reactive.
When the URL changes:

```text
router updates route object
      |
computed values invalidate
      |
components using them re-render
```

This is URL state connected to the same reactivity system as normal data.

---

# Syncing Filter to Query Params

```typescript
const route = useRoute();
const router = useRouter();

const filter = computed({
  get: () => route.query.filter ?? "all",
  set: value => {
    router.push({
      query: { ...route.query, filter: value },
    });
  },
});
```

Changing the filter changes the URL.
Changing the URL changes the filter.

This gives a reload-safe UI without inventing another persistence mechanism.

---

# Data Flow in the Sample App

```text
click "Done"
    |
TaskList emits toggle(id)
    |
TaskPage updates tasks
    |
computed visibleTasks/doneCount invalidate
    |
TaskList and TaskStats re-render
    |
Vue patches DOM
```

No component should need to know every other component.

They interact through explicit state and events.

---

# Fetching Data

```typescript
const tasks = ref<Task[]>([]);
const loading = ref(false);
const error = ref<string | null>(null);

async function loadTasks() {
  loading.value = true;
  error.value = null;

  try {
    const res = await fetch("/api/tasks/");
    tasks.value = await res.json();
  } catch (err) {
    error.value = String(err);
  } finally {
    loading.value = false;
  }
}
```

Network state is state too.

---

# Lifecycle: When Code Runs

```typescript
import { onMounted } from "vue";

onMounted(() => {
  loadTasks();
});
```

Lifecycle hooks answer timing questions:

- component was created
- component entered the DOM
- component updated
- component is about to be removed

They are useful at the boundary between Vue and the outside world:

- network
- timers
- DOM APIs
- third-party libraries

---

# Watchers: Effects for Side Effects

```typescript
watch(
  () => route.query.filter,
  () => loadTasks(),
  { immediate: true }
);
```

Use `computed` for values.
Use `watch` for side effects.

Good uses:

- refetch when route changes
- save preferences to local storage
- start/stop subscriptions

Bad use:

- copying data from one variable to another because the model is unclear

---

# Forms: State at the Edge

```vue
<script setup lang="ts">
const title = ref("");

function submit() {
  tasks.value.push({
    id: Date.now(),
    title: title.value,
    done: false,
  });
  title.value = "";
}
</script>

<template>
  <form @submit.prevent="submit">
    <input v-model="title">
    <button>Add</button>
  </form>
</template>
```

`v-model` connects input value and state.

---

# `v-model` Is Not Magic

```vue
<input v-model="title">
```

Roughly means:

```vue
<input
  :value="title"
  @input="title = $event.target.value"
>
```

The DOM input has its own temporary state.
Vue synchronizes it with application state.

This is why form controls are a common source of subtle bugs.

---

# Why Frameworks Help

Practical advantages, without slogans:

- **Consistency:** the app has one standard way to express UI updates
- **Local reasoning:** component boundaries reduce the amount of code in your head
- **Declarative rendering:** templates describe desired output, not every mutation
- **Reactivity:** derived values stay synchronized
- **Routing:** URL, history, and page state use one abstraction
- **Tooling:** type checks, compiler warnings, devtools, hot reload
- **Composition:** reusable components and composables reduce repeated event code

These are engineering advantages, not magic performance claims.

---

# What Frameworks Cost

Costs are real:

- more runtime and build complexity
- framework-specific concepts to learn
- possible overengineering for small pages
- debugging includes generated code and scheduler timing
- dependency upgrades become part of maintenance
- performance problems can be hidden behind abstractions

Frameworks are worth it when UI state and interactions are complex enough.

They are not automatically worth it for every page.

---

# Advanced Topic: State Machines

Many UI bugs are invalid states.

Bad model:

```typescript
const loading = ref(false);
const error = ref<string | null>(null);
const data = ref<Task[] | null>(null);
```

This permits nonsense:

```text
loading = true, error = "failed", data = [...]
```

Better model:

```typescript
type LoadState =
  | { status: "idle" }
  | { status: "loading" }
  | { status: "error"; message: string }
  | { status: "ready"; data: Task[] };
```

---

# Advanced Topic: Composables

A composable is a reusable piece of stateful logic.

```typescript
export function useTasks() {
  const tasks = ref<Task[]>([]);
  const loading = ref(false);

  async function load() {
    loading.value = true;
    tasks.value = await fetch("/api/tasks/").then(r => r.json());
    loading.value = false;
  }

  return { tasks, loading, load };
}
```

This is not a component.
It has no template.
It packages behavior and state.

---

# Advanced Topic: Stores

When many routes need the same data, a shared store can be cleaner.

```typescript
export const useTaskStore = defineStore("tasks", () => {
  const tasks = ref<Task[]>([]);
  const openTasks = computed(() => tasks.value.filter(t => !t.done));

  function toggle(id: number) {
    const task = tasks.value.find(t => t.id === id);
    if (task) task.done = !task.done;
  }

  return { tasks, openTasks, toggle };
});
```

But stores are not a license to make everything global.
Global state should be deliberately owned.

---

# Advanced Topic: Server and Client Routing

If the browser requests:

```text
GET /tasks/42
```

The server must decide what to return.

For a single-page app, it usually returns the same `index.html`:

```text
server returns app shell
Vue Router sees /tasks/42
Vue renders TaskDetailsPage
```

If the server returns `404`, refresh breaks even though in-app navigation worked.

Client-side routing changes browser behavior, but it does not remove HTTP.

---

# Advanced Topic: Hydration

Some apps render HTML on the server first.

```text
server renders HTML
browser displays page quickly
JavaScript loads
Vue attaches event listeners to existing DOM
app becomes interactive
```

That attach step is called **hydration**.

Problems happen when server-rendered HTML and client-rendered state disagree.

Hydration is where frameworks meet distributed systems:

- server time
- client time
- network delay
- two executions of "same" rendering logic

---

# Performance Model

Most UI performance problems are not solved by choosing a framework.

Common causes:

- too much state invalidates too much UI
- expensive computed values run too often
- large lists render without virtualization
- event handlers block the main thread
- network waterfalls delay data
- unnecessary global state causes broad updates

Vue gives tools.
It does not remove algorithmic thinking.

---

# Debugging Mental Model

When a Vue app behaves incorrectly, ask:

1. What is the source of truth?
2. Which component owns it?
3. Is this value state or derived data?
4. Which event changes it?
5. Which render depends on it?
6. Is the URL also part of the state?
7. Is a side effect racing with another side effect?

This usually finds the bug faster than staring at the DOM.

---

# Common Mistakes

- Mutating props directly instead of emitting events
- Using watchers to patch over unclear state ownership
- Storing duplicate derived state
- Forgetting stable `:key` in lists
- Treating router state and component state as separate truths
- Breaking normal link behavior with custom click handlers
- Reading DOM immediately after a state write without waiting for update
- Making a global store before knowing ownership boundaries

---

# Summary

- State is the minimal memory needed to redraw the UI
- Components organize state, rendering, and events
- Vue event syntax compiles to ordinary browser listeners
- Vue reactivity is built from dependency tracking around JavaScript objects
- The router uses `preventDefault`, `pushState`, and `popstate`
- Frameworks help with consistency and scale, but introduce real complexity
- Advanced frontend work is mostly careful state modeling

---

# Questions?

Recommended reading:

- Vue Guide: Reactivity Fundamentals
- Vue Guide: Components Basics
- Vue Router: Dynamic Route Matching
- MDN: History API
- MDN: EventTarget and event bubbling

---
{% endraw %}
