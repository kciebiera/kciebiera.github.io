# Lab 10: TypeScript Fetch — A Type-Safe HTTP Client

## Introduction

In Lab 9 you called the API with raw `fetch()`. Every endpoint was a one-off call. In a larger app, you would end up repeating the same error-handling, URL-building, and JSON-parsing logic everywhere. This lab refactors that into a typed HTTP client — a reusable module that knows your API's shape.

The Goal: Build a `BlogApiClient` class in TypeScript that wraps every endpoint from Lab 7. Use it to build a comment form that posts to the API and updates the page — no full reload.

### The Theory

`async/await` is syntactic sugar over Promises. It does not change when the request happens; it changes how the code *reads*.

```typescript
// Promise chain style
fetch("/api/posts/")
    .then(res => res.json())
    .then(data => console.log(data))
    .catch(err => console.error(err));

// async/await style — same operations, linear reading order
try {
    const res  = await fetch("/api/posts/");
    const data = await res.json();
    console.log(data);
} catch (err) {
    console.error(err);
}
```

Both styles mix freely. You can `await` any Promise.

## Setup

Continue in the same Django + TypeScript project from Lab 9. Create `pages/static/pages/ts/api.ts` — this is your client module. `main.ts` (and future files) will import from it.

Since you already have `esbuild` configured from Lab 9, imports between files (`import { api } from "./api"`) are resolved automatically during bundling. Update `npm run build` if needed to use `main.ts` as the entry point — esbuild will follow all imports and produce a single `.js` file.

## Phase 1: The Result Type and CSRF

Real API calls fail in two ways: network errors (no server) and API errors (wrong data, 401, 404). Returning `null` on error loses the reason. A `Result` type makes errors explicit:

```typescript
// pages/static/pages/ts/api.ts

type Result<T> =
    | { ok: true;  data: T }
    | { ok: false; error: string; status?: number };
```

### Handling CSRF tokens

Django protects against Cross-Site Request Forgery by requiring a token on all `POST`/`PUT`/`PATCH`/`DELETE` requests. The token lives in a cookie named `csrftoken`. You must read it and send it as a header:

```typescript
function getCsrfToken(): string {
    const match = document.cookie.match(/csrftoken=([^;]+)/);
    return match ? match[1] : "";
}
```

Without this, every write request from `fetch()` will receive `403 Forbidden` from Django.

### The generic fetch wrapper

```typescript
async function apiFetch<T>(
    method: string,
    path: string,
    body?: unknown
): Promise<Result<T>> {
    try {
        const options: RequestInit = {
            method,
            headers: {
                "Content-Type": "application/json",
                "X-CSRFToken":  getCsrfToken(),
            },
        };
        if (body !== undefined) {
            options.body = JSON.stringify(body);
        }

        const res = await fetch(path, options);

        if (!res.ok) {
            let message = `HTTP ${res.status}`;
            try {
                const err = await res.json();
                message = err.error ?? message;
            } catch { /* ignore */ }
            return { ok: false, error: message, status: res.status };
        }

        if (res.status === 204) {
            return { ok: true, data: {} as T };
        }

        const data: T = await res.json();
        return { ok: true, data };

    } catch (err) {
        return { ok: false, error: String(err) };
    }
}
```

`apiFetch` is generic over `T` — the caller tells it what shape to expect and TypeScript enforces it from that point on. The CSRF token is sent on every request — harmless on `GET`, required on mutations.

## Phase 2: The API Client and Utility Types

### Utility types for API payloads

TypeScript has built-in **utility types** that transform existing types. They are essential when working with APIs where create, update, and read shapes differ:

| Utility | What it does | API use case |
|---------|-------------|-------------|
| `Partial<T>` | Makes all fields optional | PATCH payloads (update some fields) |
| `Pick<T, K>` | Keeps only named fields | POST payloads (only send required fields) |
| `Omit<T, K>` | Removes named fields | Form data (no `id`, no `created`) |

```typescript
// Reuse the Post interface from Lab 9 (or import it); Comment is introduced here.
interface Post {
    id: number; title: string; slug: string;
    body: string; pub_date: string; category: string | null;
}
interface Comment {
    id: number; author: string; body: string; created: string;
}

// CreateCommentPayload is a Comment without server-generated fields:
type CreateCommentPayload = Pick<Comment, "author" | "body">;

// For PATCH, any subset of fields is valid:
type UpdatePostPayload = Partial<Pick<Post, "title" | "body" | "slug">>;
```

🧪 Try assigning `{ author: "Alice", body: "Hi", id: 99 }` to a `CreateCommentPayload` — TypeScript should error because `id` is not in the type.

### The client class

```typescript
class BlogApiClient {

    async getPosts(search?: string): Promise<Result<{ posts: Post[] }>> {
        const qs = search ? `?search=${encodeURIComponent(search)}` : "";
        return apiFetch("GET", `/api/posts/${qs}`);
    }

    async getPost(id: number): Promise<Result<Post>> {
        // TODO: call apiFetch for /api/posts/<id>/
        return apiFetch("GET", "");
    }

    async getComments(postId: number): Promise<Result<{ comments: Comment[] }>> {
        // TODO: call apiFetch for /api/posts/<postId>/comments/
        return apiFetch("GET", "");
    }

    async createComment(
        postId: number,
        payload: CreateCommentPayload
    ): Promise<Result<Comment>> {
        // TODO: call apiFetch with method POST and the payload as body
        return apiFetch("POST", "");
    }

    async updatePost(
        id: number,
        payload: UpdatePostPayload
    ): Promise<Result<Post>> {
        // TODO: call apiFetch with method PATCH and the payload as body
        return apiFetch("PATCH", "");
    }
}

const api = new BlogApiClient();
export { api, BlogApiClient, Post, Comment, Result, CreateCommentPayload, UpdatePostPayload };
```

## Phase 3: Using the Result Type

Update `pages/static/pages/ts/main.ts` to import from `api.ts` and use the typed client:

```typescript
import { api, Post } from "./api";

async function loadAndRender(query?: string): Promise<void> {
    const result = await api.getPosts(query);

    if (!result.ok) {
        statusBar.textContent = `Error: ${result.error}`;
        return;
    }

    const { posts } = result.data;
    statusBar.textContent = `${posts.length} post(s)`;
    postList.innerHTML = "";
    for (const post of posts) {
        postList.appendChild(renderPost(post));
    }
}
```

Because `result.ok` is a discriminated union, TypeScript knows:
- Inside the `if (!result.ok)` branch: `result.error` exists, `result.data` does not.
- After the block: `result.data` is available and typed as `{ posts: Post[] }`.

🧪 Try accessing `result.data` inside the `if (!result.ok)` block — TypeScript should give a compile error.

### Exercise: retry with exponential backoff

```typescript
// TODO: Write a function that retries failed API calls:
//
// async function withRetry<T>(
//     fn: () => Promise<Result<T>>,
//     retries: number,
//     delayMs: number
// ): Promise<Result<T>>
//
// - Call fn(). If it succeeds (result.ok === true), return immediately.
// - If it fails and retries > 0, wait delayMs, then try again with
//   retries - 1 and delayMs * 2 (exponential backoff).
// - If all retries are exhausted, return the last error.
//
// Display each attempt in the status bar: "Attempt 1/3…", "Retry 2/3 (waiting 400ms)…"
//
// Test: temporarily break your API URL to see retries in action.
// Restore the URL — the retry should eventually succeed.
```

## Phase 4: Inline Comment Form

Extend `renderPost` to show a comment form beneath each post.

```typescript
function renderCommentForm(postId: number, container: HTMLElement): void {
    const form = document.createElement("form");

    const authorInput = document.createElement("input");
    authorInput.type        = "text";
    authorInput.placeholder = "Your name";
    authorInput.required    = true;

    const bodyTextarea = document.createElement("textarea");
    bodyTextarea.placeholder = "Your comment";
    bodyTextarea.required    = true;
    bodyTextarea.rows        = 3;

    const submitBtn = document.createElement("button");
    submitBtn.type        = "submit";
    submitBtn.textContent = "Post Comment";

    const feedback = document.createElement("p");

    form.append(authorInput, bodyTextarea, submitBtn, feedback);

    form.addEventListener("submit", async (event: Event) => {
        event.preventDefault();
        submitBtn.disabled = true;
        feedback.textContent = "Posting…";

        const result = await api.createComment(postId, {
            author: authorInput.value.trim(),
            body:   bodyTextarea.value.trim(),
        });

        if (result.ok) {
            feedback.textContent = "Comment posted!";
            form.reset();
            // TODO: Append a new <div> to container displaying result.data.author
            //       and result.data.body — no page reload needed.
        } else {
            feedback.textContent = `Error: ${result.error}`;
        }

        submitBtn.disabled = false;
    });

    container.appendChild(form);
}
```

Call `renderCommentForm(post.id, article)` at the end of `renderPost`.

🧪 Submit a comment. It should appear immediately below the form without any page reload. Open DevTools → Network and confirm a single `POST` request was made.

### Exercise: client-side validation

```typescript
// TODO: Add client-side validation before submitting the comment form.
//
// Define the validation result type:
// type ValidationResult =
//     | { valid: true }
//     | { valid: false; errors: Record<string, string> };
//
// Write a function validateComment(author: string, body: string): ValidationResult
//   - author must be 2–50 characters
//   - body must be 10–500 characters
//   - Return all errors at once (not just the first one)
//
// In the submit handler, call validateComment BEFORE calling api.createComment.
// If validation fails, display each error message inline next to its field
// (create a <span class="error"> after the input/textarea).
// Do NOT send the request if validation fails.
//
// Style the error spans in CSS: .error { color: red; font-size: 0.85em; }
```

## Phase 5: Loading Comments on Expand

When a user clicks "Read more", also fetch and display existing comments:

```typescript
// In the toggle click handler:
toggle.addEventListener("click", async () => {
    const isExpanded = !fullBody.hidden;
    fullBody.hidden = isExpanded;
    toggle.textContent = isExpanded ? "Read more" : "Show less";

    if (!isExpanded) {
        // TODO: call api.getComments(post.id)
        // If result.ok, render each comment above the form.
        // Show a loading indicator while fetching.
    }
});
```

### Exercise: optimistic delete

```typescript
// TODO: Add a delete button to each rendered comment.
// When clicked:
//   1. Remove the comment from the DOM immediately (optimistic update)
//   2. Send a DELETE request to /api/posts/<postId>/comments/<commentId>/
//   3. If the request fails, re-insert the comment and show an error
//
// This requires:
//   - Adding a deleteComment(postId, commentId) method to BlogApiClient
//   - Keeping a reference to the comment element so you can re-insert it
//   - Showing a brief "Deleting…" state on the button before removing
//
// Hint: use element.remove() to remove, and container.insertBefore()
// to re-insert at the original position.
```

### Exercise: real-time polling

```typescript
// TODO: When a post is expanded, automatically refresh its comments every 10 seconds.
//
// 1. Start a setInterval when a post is expanded (fullBody.hidden === false).
//    Store the timer handle with proper typing: let pollTimer: ReturnType<typeof setInterval>;
//
// 2. On each tick, call api.getComments(post.id) and re-render the comment list.
//    Be careful not to duplicate comments — replace the entire list.
//
// 3. When the post is collapsed, clearInterval(pollTimer).
//
// 4. Display "Last updated: Xs ago" below the comments. Update the counter
//    every second using a separate setInterval (clear it on collapse too).
//
// 5. If the tab is hidden (document.hidden === true), pause polling to save
//    bandwidth. Resume when the tab becomes visible again.
//    Hint: listen for the "visibilitychange" event on document.
```

## Submission

Final checks:

1. `npm run check` — zero errors, zero `any` types.
2. The `Result` type is used for all API calls; errors are displayed to the user.
3. Submitting the comment form validates client-side before sending.
4. Expanding a post loads and displays its existing comments.
5. The CSRF token is sent on every `POST` request (verify in DevTools → Network → Headers).
6. `withRetry` shows retry attempts in the UI and recovers after transient failures.
7. Deleting a comment is optimistic — the UI updates immediately.

**Exploration:** Type a paginated API response. Many real APIs don't return all results at once — they paginate. Define a generic paginated wrapper:

```typescript
interface PaginatedResponse<T> {
    count:    number;
    next:     string | null;
    previous: string | null;
    results:  T[];
}

// Usage: getPosts() could return Result<PaginatedResponse<Post>>
// The client would follow `next` URLs to load more pages.
```

Try modifying `getPosts()` to accept an optional `page` parameter and return `PaginatedResponse<Post>`. This is the shape DRF uses by default when pagination is enabled.
