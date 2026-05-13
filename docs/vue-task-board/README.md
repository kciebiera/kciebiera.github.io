# Vue Task Board Demo

Companion app for `docs/slides/lecture10.md`.

Open it from the course site:

```text
vue-task-board/
```

Or run locally from `docs/`:

```bash
python3 -m http.server 8765
```

Then open:

```text
http://127.0.0.1:8765/vue-task-board/
```

## What to Demonstrate

1. Open the dashboard and show that counts are computed from task state.
2. Go to `Tasks` and click `Done` on a task.
3. Watch the inspector update:
   - raw state changes
   - derived state changes
   - event log records the mutation
   - `nextTick` observation shows DOM update scheduling
4. Click the `Open` / `Done` filters and point out that the filter lives in the URL query.
5. Open a task detail route and show that the selected task id comes from route params.
6. Add a local task and show `v-model`, `submit.prevent`, state mutation, and router navigation.
7. Use the browser Back button to show that route changes are part of browser history.

## Implementation Notes

- The app is intentionally static: no Vite, npm install, or build step.
- Vue and Vue Router are loaded from CDN through an import map.
- The demo uses hash routing so refresh works on static hosting.
- The inspector mentions `createWebHistory()` because that is the production-style mode that uses the History API directly, but it needs server fallback routing.
