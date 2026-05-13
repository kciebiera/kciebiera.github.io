import {
  computed,
  createApp,
  nextTick,
  reactive,
  ref,
  watch,
} from "vue";
import {
  createRouter,
  createWebHashHistory,
  useRoute,
  useRouter,
} from "vue-router";

const initialTasks = [
  {
    id: 1,
    title: "Read the Vue lecture",
    done: true,
    owner: "Ada",
    tags: ["state", "components"],
    notes: "Find where the lecture describes state as the minimum memory needed to redraw the UI.",
  },
  {
    id: 2,
    title: "Trace one button click",
    done: false,
    owner: "Grace",
    tags: ["events", "scheduler"],
    notes: "Click Done, then inspect which state and derived values changed.",
  },
  {
    id: 3,
    title: "Explain route state",
    done: false,
    owner: "Linus",
    tags: ["router", "history"],
    notes: "Use the links, browser Back button, and query string filter.",
  },
  {
    id: 4,
    title: "Remove duplicate state",
    done: false,
    owner: "Barbara",
    tags: ["computed", "debugging"],
    notes: "Open count and visible tasks are computed from tasks and route query state.",
  },
];

const state = reactive({
  tasks: initialTasks,
  events: [],
  lastDomPatch: "not measured yet",
});

function logEvent(message) {
  const time = new Date().toLocaleTimeString();
  state.events.unshift(`${time} - ${message}`);
  state.events = state.events.slice(0, 12);
}

function taskById(id) {
  return state.tasks.find(task => task.id === Number(id));
}

function toggleTask(id) {
  const task = taskById(id);
  if (!task) return;
  task.done = !task.done;
  logEvent(`state write: task ${id}.done = ${task.done}`);
}

function addTask(payload) {
  const nextId = Math.max(...state.tasks.map(task => task.id)) + 1;
  state.tasks.push({
    id: nextId,
    done: false,
    notes: "Created locally in the browser. Refresh to reset the demo.",
    ...payload,
  });
  logEvent(`state write: tasks.push(task ${nextId})`);
  return nextId;
}

const TaskList = {
  props: {
    tasks: { type: Array, required: true },
  },
  emits: ["toggle"],
  template: `
    <div class="grid">
      <article
        v-for="task in tasks"
        :key="task.id"
        class="task-row"
        :class="{ done: task.done }"
      >
        <button
          class="primary"
          type="button"
          @click="$emit('toggle', task.id)"
        >
          {{ task.done ? "Undo" : "Done" }}
        </button>

        <div class="task-title">
          <RouterLink :to="{ name: 'task-detail', params: { id: task.id } }">
            {{ task.title }}
          </RouterLink>
          <span class="muted">
            owner: {{ task.owner }} -
            <span :class="task.done ? 'done-label' : 'open-label'">
              {{ task.done ? "done" : "open" }}
            </span>
          </span>
          <div class="tag-list">
            <span v-for="tag in task.tags" :key="tag" class="tag">{{ tag }}</span>
          </div>
        </div>

        <RouterLink class="button" :to="{ name: 'task-detail', params: { id: task.id } }">
          Details
        </RouterLink>
      </article>
    </div>
  `,
};

const DashboardPage = {
  setup() {
    const openCount = computed(() => state.tasks.filter(task => !task.done).length);
    const doneCount = computed(() => state.tasks.filter(task => task.done).length);
    const owners = computed(() => new Set(state.tasks.map(task => task.owner)).size);

    return { openCount, doneCount, owners, state };
  },
  template: `
    <section>
      <div class="section-title">
        <div>
          <h1>Dashboard</h1>
          <p>Computed values rendered from the same task state.</p>
        </div>
        <RouterLink class="button primary" to="/tasks">Open task list</RouterLink>
      </div>

      <div class="cards">
        <article class="card">
          <h2>Open</h2>
          <div class="metric">{{ openCount }}</div>
          <p class="muted">computed from tasks where done is false</p>
        </article>
        <article class="card">
          <h2>Done</h2>
          <div class="metric">{{ doneCount }}</div>
          <p class="muted">computed from tasks where done is true</p>
        </article>
        <article class="card">
          <h2>Owners</h2>
          <div class="metric">{{ owners }}</div>
          <p class="muted">computed from unique owner names</p>
        </article>
      </div>
    </section>
  `,
};

const TaskPage = {
  components: { TaskList },
  setup() {
    const route = useRoute();
    const router = useRouter();
    const newTitle = ref("");
    const newOwner = ref("Ada");

    const routeFilter = computed({
      get() {
        return route.query.filter || "all";
      },
      set(value) {
        router.push({
          name: "tasks",
          query: value === "all" ? {} : { filter: value },
        });
      },
    });

    const visibleTasks = computed(() => {
      if (routeFilter.value === "open") {
        return state.tasks.filter(task => !task.done);
      }
      if (routeFilter.value === "done") {
        return state.tasks.filter(task => task.done);
      }
      return state.tasks;
    });

    const openCount = computed(() => state.tasks.filter(task => !task.done).length);
    const doneCount = computed(() => state.tasks.filter(task => task.done).length);

    function setFilter(value) {
      routeFilter.value = value;
      logEvent(`router push: ?filter=${value}`);
    }

    async function onToggle(id) {
      toggleTask(id);
      const beforePatch = document.querySelector(`[data-count="open"]`)?.textContent;
      await nextTick();
      const afterPatch = document.querySelector(`[data-count="open"]`)?.textContent;
      state.lastDomPatch = `nextTick observed open count: ${beforePatch} -> ${afterPatch}`;
      logEvent("Vue flushed DOM patch after state write");
    }

    function submitTask() {
      if (!newTitle.value.trim()) return;
      const id = addTask({
        title: newTitle.value.trim(),
        owner: newOwner.value,
        tags: ["local", "v-model"],
      });
      newTitle.value = "";
      router.push({ name: "task-detail", params: { id } });
    }

    return {
      doneCount,
      newOwner,
      newTitle,
      onToggle,
      openCount,
      routeFilter,
      setFilter,
      state,
      submitTask,
      visibleTasks,
    };
  },
  template: `
    <section>
      <div class="section-title">
        <div>
          <h1>Tasks</h1>
          <p>Route query state controls the filter. Task rows emit events upward.</p>
        </div>
        <div>
          <span class="tag">open: <strong data-count="open">{{ openCount }}</strong></span>
          <span class="tag">done: <strong>{{ doneCount }}</strong></span>
        </div>
      </div>

      <div class="toolbar">
        <div class="segmented" aria-label="Task filter">
          <button type="button" :class="{ active: routeFilter === 'all' }" @click="setFilter('all')">All</button>
          <button type="button" :class="{ active: routeFilter === 'open' }" @click="setFilter('open')">Open</button>
          <button type="button" :class="{ active: routeFilter === 'done' }" @click="setFilter('done')">Done</button>
        </div>
        <span class="muted">Visible tasks is computed, not stored.</span>
      </div>

      <TaskList :tasks="visibleTasks" @toggle="onToggle" />

      <form class="form panel" @submit.prevent="submitTask">
        <h2>Add local task</h2>
        <label>
          Title
          <input v-model="newTitle" placeholder="Try typing here">
        </label>
        <label>
          Owner
          <select v-model="newOwner">
            <option>Ada</option>
            <option>Grace</option>
            <option>Linus</option>
            <option>Barbara</option>
          </select>
        </label>
        <button class="primary" type="submit">Add and open details</button>
        <p class="muted">This demonstrates form state with v-model and submit.prevent.</p>
      </form>
    </section>
  `,
};

const TaskDetailsPage = {
  setup() {
    const route = useRoute();
    const router = useRouter();

    const task = computed(() => taskById(route.params.id));

    function goBackToList() {
      router.push({ name: "tasks" });
      logEvent("router push: /tasks");
    }

    return { goBackToList, task, toggleTask };
  },
  template: `
    <section class="detail-grid">
      <div class="section-title">
        <div>
          <h1>Task Details</h1>
          <p>The selected task id comes from route params.</p>
        </div>
        <button type="button" @click="goBackToList">Back to tasks</button>
      </div>

      <article v-if="task" class="panel">
        <h2>{{ task.title }}</h2>
        <p>
          Status:
          <strong :class="task.done ? 'done-label' : 'open-label'">
            {{ task.done ? "done" : "open" }}
          </strong>
        </p>
        <p><strong>Owner:</strong> {{ task.owner }}</p>
        <p>{{ task.notes }}</p>
        <div class="tag-list">
          <span v-for="tag in task.tags" :key="tag" class="tag">{{ tag }}</span>
        </div>
        <p>
          <button class="primary" type="button" @click="toggleTask(task.id)">
            Toggle from detail view
          </button>
        </p>
      </article>

      <article v-else class="panel">
        <h2>Task not found</h2>
        <p class="muted">The route exists, but there is no task with this id in state.</p>
      </article>
    </section>
  `,
};

const SettingsPage = {
  setup() {
    const denseMode = ref(false);
    const explainMode = ref(true);

    watch(denseMode, value => {
      logEvent(`watcher side effect: dense mode = ${value}`);
    });

    return { denseMode, explainMode };
  },
  template: `
    <section>
      <div class="section-title">
        <div>
          <h1>Settings</h1>
          <p>Local component state that does not need to be global.</p>
        </div>
      </div>

      <div class="panel">
        <label>
          <input type="checkbox" v-model="denseMode">
          Dense mode
        </label>
        <p class="muted">A watcher logs this value because it represents a side effect.</p>

        <label>
          <input type="checkbox" v-model="explainMode">
          Show teaching notes
        </label>

        <p v-if="explainMode" class="route-note">
          These settings are local state. Refreshing resets them. If a setting must survive
          refresh or be shared between routes, move it to URL state, localStorage, or a store.
        </p>
      </div>
    </section>
  `,
};

const routes = [
  { path: "/", name: "dashboard", component: DashboardPage },
  { path: "/tasks", name: "tasks", component: TaskPage },
  { path: "/tasks/:id", name: "task-detail", component: TaskDetailsPage },
  { path: "/settings", name: "settings", component: SettingsPage },
];

const router = createRouter({
  history: createWebHashHistory(),
  routes,
});

router.beforeEach((to, from) => {
  if (from.fullPath !== to.fullPath) {
    logEvent(`router navigation: ${from.fullPath || "(start)"} -> ${to.fullPath}`);
  }
});

const AppInspector = {
  setup() {
    const route = useRoute();

    const openTasks = computed(() => state.tasks.filter(task => !task.done));
    const doneTasks = computed(() => state.tasks.filter(task => task.done));
    const stateSnapshot = computed(() => JSON.stringify({
      tasks: state.tasks.map(task => ({
        id: task.id,
        title: task.title,
        done: task.done,
      })),
    }, null, 2));
    const derivedSnapshot = computed(() => JSON.stringify({
      openCount: openTasks.value.length,
      doneCount: doneTasks.value.length,
      currentPath: route.fullPath,
      filterFromUrl: route.query.filter || "all",
      selectedTaskIdFromUrl: route.params.id || null,
    }, null, 2));

    return { derivedSnapshot, route, state, stateSnapshot };
  },
  template: `
    <aside class="inspector">
      <section>
        <h2>Runtime Inspector</h2>
        <p class="muted">Use the app, then watch state, derived values, route state, and event flow.</p>
      </section>

      <section class="route-note">
        This static demo uses hash routing so it works on GitHub Pages refresh.
        Vue Router can use <code>history.pushState</code> with <code>createWebHistory()</code>
        when the server is configured to return the app shell for every route.
      </section>

      <section>
        <h3>Raw state</h3>
        <pre class="codebox">{{ stateSnapshot }}</pre>
      </section>

      <section>
        <h3>Derived + route state</h3>
        <pre class="codebox">{{ derivedSnapshot }}</pre>
      </section>

      <section>
        <h3>Last DOM scheduling observation</h3>
        <p class="muted">{{ state.lastDomPatch }}</p>
      </section>

      <section>
        <h3>Event log</h3>
        <ul class="event-log">
          <li v-for="event in state.events" :key="event">{{ event }}</li>
        </ul>
      </section>
    </aside>
  `,
};

const App = {
  components: { AppInspector },
  template: `
    <div class="shell">
      <header class="topbar">
        <div class="brand">
          <strong>Vue Task Board</strong>
          <span>state, components, reactivity, routing</span>
        </div>
        <nav class="nav" aria-label="Main navigation">
          <RouterLink to="/">Dashboard</RouterLink>
          <RouterLink to="/tasks">Tasks</RouterLink>
          <RouterLink to="/tasks/1">Task #1</RouterLink>
          <RouterLink to="/settings">Settings</RouterLink>
        </nav>
      </header>

      <div class="layout">
        <main class="main">
          <RouterView />
        </main>
        <AppInspector />
      </div>
    </div>
  `,
};

createApp(App).use(router).mount("#app");
logEvent("app mounted: Vue owns #app");
