# Lab 10: TypeScript Fetch — A Type-Safe HTTP Client

## Introduction

In Lab 9 you called the API with raw `fetch()`. Each endpoint was handled separately. In this lab you will move URL building, JSON parsing, error handling, and CSRF handling into a typed API client.

**Goal:** Build a `BlogApiClient` class in TypeScript, use it from the blog browser, and update comments without a full page reload.

### Async Basics

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

Both styles can be used in the same project. You can `await` any Promise.

## Setup

Continue in the same Django + TypeScript project from Lab 9. Create `pages/static/pages/ts/api.ts` — this is your client module. `main.ts` (and future files) will import from it.

Since you already have `esbuild` configured from Lab 9, imports between files (`import { api } from "./api"`) are resolved automatically during bundling. Update `npm run build` if needed to use `main.ts` as the entry point — esbuild will follow all imports and produce a single `.js` file.

Expected structure:

```text
pages/
    static/
        pages/
            ts/
                api.ts
                main.ts
            js/
                main.js
```

## Phase 1: The Result Type and CSRF

Real API calls fail in two ways: network errors (no server) and API errors (wrong data, 401, 404). Returning `null` on error loses the reason. A `Result` type makes errors explicit:

```typescript
// pages/static/pages/ts/api.ts

type Result<T> =
    | { ok: true;  data: T }
    | { ok: false; error: string; status?: number };
```

### Handling CSRF Tokens

Django protects against Cross-Site Request Forgery by requiring a token on all `POST`/`PUT`/`PATCH`/`DELETE` requests. The token lives in a cookie named `csrftoken`. You must read it and send it as a header:

```typescript
function getCsrfToken(): string {
    const match = document.cookie.match(/csrftoken=([^;]+)/);
    return match ? match[1] : "";
}
```

Without this, every write request from `fetch()` will receive `403 Forbidden` from Django.

### Generic Fetch Wrapper

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

### Utility Types for API Payloads

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

**Check:** Try assigning `{ author: "Alice", body: "Hi", id: 99 }` to a `CreateCommentPayload`. TypeScript should error because `id` is not in the type.

### API Client Class

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

**Check:** Try accessing `result.data` inside the `if (!result.ok)` block. TypeScript should give a compile error.

### Exercise: Retry with Exponential Backoff

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

**Check:** Submit a comment. It should appear immediately below the form without any page reload. Open DevTools -> Network and confirm a single `POST` request was made.

### Exercise: Client-Side Validation

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

### Exercise: Optimistic Delete

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

Backend requirement for this exercise: add a comment delete endpoint to the Django API if you do not already have one.

```python
# blog/api.py
class CommentDetailView(View):
    def delete(self, request, post_id, comment_id):
        if not request.user.is_authenticated:
            return JsonResponse({"error": "Authentication required."}, status=401)
        comment = Comment.objects.filter(
            pk=comment_id,
            post_id=post_id,
        ).first()
        if comment is None:
            return JsonResponse({"error": "Not found."}, status=404)
        comment.delete()
        return JsonResponse({}, status=204)
```

Add the route:

```python
path(
    "posts/<int:post_id>/comments/<int:comment_id>/",
    api.CommentDetailView.as_view(),
    name="api-comment-detail",
)
```

### Exercise: Real-Time Polling

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

Submit the complete TypeScript browser work from Labs 8-10.

Required files:

1. `package.json`
2. `tsconfig.json`
3. Lab 8 standalone files: `index.html`, `src/`, and `dist/main.js`
4. `pages/static/pages/ts/main.ts`
5. `pages/static/pages/ts/api.ts`
6. `pages/static/pages/js/main.js`
7. `pages/templates/pages/blog_browser.html`

Lab 8 requirements:

1. `npm run check` passes with zero TypeScript errors.
2. No `any` types are used.
3. Primitive type exercises render results in the page.
4. `fetchWithTimeout` succeeds with a normal timeout and fails with a very short timeout.
5. Interfaces for posts, categories, and comments are used.
6. Union types, type guards, and generics are implemented.
7. Source maps allow debugging `.ts` files in DevTools.

Lab 9 requirements:

1. `/browse/` loads the TypeScript bundle from Django static files.
2. Posts load from `/api/posts/`.
3. Search uses the API `?search=` query parameter and is debounced.
4. Sort works for date, title, and category.
5. Matching search text is highlighted with `<mark>`.
6. Expand/collapse works with mouse and keyboard.
7. Arrow-key navigation has a visible focus indicator.
8. Search and sort preferences persist with `localStorage`.

Lab 10 requirements:

1. `BlogApiClient` wraps all API calls used by the page.
2. API calls return `Result<T>` instead of throwing errors into UI code.
3. Errors are displayed to the user.
4. Comment form data is validated client-side before sending.
5. Creating a comment sends `POST` through the API client and updates the page without reload.
6. Expanding a post loads existing comments.
7. CSRF token is sent on `POST`, `PATCH`, and `DELETE` requests.
8. `withRetry` shows retry attempts and recovers after transient failures.
9. Optimistic delete removes a comment immediately and restores it if the request fails.
10. The Django API includes the comment delete endpoint used by optimistic delete.

Before submitting, run:

```bash
npm run check
npm run build
uv run python manage.py runserver
```

Then manually test `/browse/`, search, sort, expand/collapse, comment creation, comment loading, and failed API/error states.

Common errors:

1. `403 Forbidden` on writes usually means the CSRF token header is missing.
2. `result.data` not available usually means TypeScript has not narrowed `Result<T>` with `result.ok`.
3. Duplicate comments after expand usually means the comments container is appended to repeatedly instead of replaced.
4. Optimistic delete not restoring usually means the original DOM position was not stored.
5. `main.js` not changing usually means `npm run build` or `npm run watch` was not run.

Optional: Type a paginated API response. Many APIs do not return all results at once. Define a generic paginated wrapper:

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
