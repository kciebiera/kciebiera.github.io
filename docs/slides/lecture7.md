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
{% raw %}

# Lecture 7
## REST APIs — Ideas, Principles, and Django

**WWW 25/26**
REST · Resources · HTTP semantics · JSON · CORS · API design

---

# What This Lecture Is About

This lecture is about **ideas**, not Django wiring.

- What problem do APIs solve?
- Where does REST come from and what does it actually mean?
- What is a "resource" and why does the concept matter?
- Why do HTTP methods and status codes carry semantic weight?
- How do you design an API that other developers can understand without reading your source code?

> The lab will cover the Django implementation details (views, serializers, DRF).

---

# The Problem — Why APIs?

A traditional Django view returns **HTML** — a full page the browser renders.

But modern apps need **data**, not pages:
- A React/Vue/Svelte frontend renders its own UI from raw data
- A mobile app needs structured responses, not markup
- A third-party service needs machine-readable output
- A CLI tool or script wants to automate tasks

**Solution:** build endpoints that return **data** (usually JSON) instead of HTML.

```
Browser                 Django (traditional)
  ──── GET /posts/ ──►  renders posts.html ──► full HTML page

JavaScript / mobile     Django (API)
  ── GET /api/posts/ ►  returns JSON       ──► [{id:1, title:...}, ...]
```

---

# Decoupling Frontend from Backend

<div class="columns">
<div>

**Tightly coupled (traditional):**
- Template logic lives in Python
- Changing the UI requires touching views
- Only one client possible (browser)
- Server re-renders on every interaction

</div>
<div>

**Decoupled (API-first):**
- Django owns data & business logic
- Any client can consume the API
- Mobile, web, CLI, other services
- Partial page updates with `fetch()`

</div>
</div>

```
                  ┌─────────────────┐
  React app ─────►│                 │
  Mobile app ────►│  Django REST API│──► Database
  CLI script ────►│                 │
  3rd-party ─────►└─────────────────┘
```

Django becomes the **single source of truth** for data; the presentation layer is somebody else's problem.

---

# Where REST Comes From

In 2000, **Roy Fielding** published his PhD dissertation:
*"Architectural Styles and the Design of Network-based Software Architectures"*

Fielding was one of the principal authors of the **HTTP specification** (RFC 2616). He didn't invent a new protocol — he described the **architectural principles already present in the web** and gave them a name:

> **REST** — Representational State Transfer

Key insight: the web already worked phenomenally well at scale. REST captured *why* — and showed how to apply the same principles to APIs.

REST is **not** a protocol, a library, or a standard. It is a set of **architectural constraints** that, when followed, produce systems with desirable properties: scalability, simplicity, modifiability, visibility, reliability.

---

# The Six REST Constraints

| Constraint | What it means | Why it matters |
|------------|---------------|----------------|
| **Client–server** | UI and data storage are separated | They evolve independently |
| **Stateless** | Each request carries all info needed | Server doesn't track sessions between requests |
| **Cacheable** | Responses say whether they can be reused | Reduces load, improves latency |
| **Uniform interface** | All resources follow the same conventions | Any developer can guess how a new endpoint works |
| **Layered system** | Client can't tell if it talks to origin or proxy | Load balancers, CDNs, gateways work transparently |
| **Code on demand** *(optional)* | Server can send executable code (e.g., JS) | Extends client functionality |

For day-to-day API design, the ones that matter most are **stateless** and **uniform interface**.

---

# Statelessness — Why It Matters

<div class="columns">
<div>

**Stateful (session-based):**
1. Client logs in → server stores session
2. Each subsequent request references that session
3. Server must remember state between requests
4. Load balancing is tricky (sticky sessions?)
5. Server crash = lost sessions

</div>
<div>

**Stateless (REST):**
1. Each request includes everything needed (token, data, context)
2. Server processes request, forgets about it
3. Any server can handle any request
4. Horizontal scaling is trivial
5. Server crash = no state lost

</div>
</div>

**Statelessness** doesn't mean you can't have login — it means the **server doesn't store per-client state between requests**. Authentication info (token, API key) travels with every request.

> "Each request from client to server must contain all of the information necessary to understand the request." — Fielding, §5.1.3

---

# What Is a Resource?

A **resource** is any concept that can be named and addressed. It's the fundamental abstraction in REST.

<div class="columns">
<div>

**Resources are nouns:**
- A blog post (`/posts/42`)
- A collection of posts (`/posts/`)
- A user profile (`/users/ada`)
- A shopping cart (`/cart/`)
- Today's weather in Warsaw (`/weather/warsaw/today`)

</div>
<div>

**Not resources (bad URLs):**
- `/getPost?id=42` — verb, not noun
- `/deleteUser/5` — action in the URL
- `/doSearch` — procedure call
- `/api/v2/processPayment` — RPC

</div>
</div>

A resource has an **identifier** (URL) and one or more **representations** (JSON, HTML, XML…).

When you `GET /posts/42`, you receive a **representation** of the resource — not the resource itself. The resource is the abstract concept; the JSON is just one way to look at it.

---

# Representations and Content Negotiation

The same resource can have **multiple representations**:

```
GET /api/posts/42
Accept: application/json     →  {"id": 42, "title": "Hello"}

GET /api/posts/42
Accept: text/html            →  <h1>Hello</h1><p>...</p>

GET /api/posts/42
Accept: application/xml      →  <post><id>42</id>...</post>
```

The **`Accept` header** tells the server what format the client prefers. The server picks the best match and sets `Content-Type` in the response.

This is called **content negotiation** — a core idea in REST that most APIs simplify by only supporting JSON.

> The URL identifies **what** (the resource); the representation describes **how** (the format).

---

# HTTP Methods Are the Verbs

Resources are nouns in the URL. HTTP **methods** are the verbs.

| Method | Meaning | Safe? | Idempotent? |
|--------|---------|-------|-------------|
| `GET` | Read a resource | ✅ | ✅ |
| `POST` | Create a new resource | ❌ | ❌ |
| `PUT` | Replace a resource entirely | ❌ | ✅ |
| `PATCH` | Partially update a resource | ❌ | ⚠️* |
| `DELETE` | Remove a resource | ❌ | ✅ |

```
GET    /api/posts/       → list all posts
POST   /api/posts/       → create a new post
GET    /api/posts/42/    → read post #42
PUT    /api/posts/42/    → replace post #42 entirely
PATCH  /api/posts/42/    → update some fields of post #42
DELETE /api/posts/42/    → delete post #42
```

*\*PATCH **can** be idempotent (e.g., `{"title": "new"}`) but the spec does not guarantee it (e.g., `{"views": "+1"}`). Unlike PUT, the result may depend on current state.*

**Golden rule:** URLs are nouns — never `/getPost/`, `/deleteUser/`, `/createComment/`.

---

# Safety and Idempotency

Two properties that HTTP methods promise — and that clients, caches, and intermediaries rely on:

<div class="columns">
<div>

## Safe

A method is **safe** if it does **not change** server state.

`GET` and `HEAD` are safe. Calling them 1000 times changes nothing.

Crawlers, prefetch, caches — all assume `GET` is safe. If your `GET /delete-post/5` actually deletes something, you've broken the contract.

</div>
<div>

## Idempotent

A method is **idempotent** if calling it **N times has the same effect as calling it once**.

`PUT` and `DELETE` are idempotent. If a network error occurs mid-request, the client can **safely retry**.

`POST` is *not* idempotent — sending it twice may create two resources!

`PATCH` is a grey area — it *can* be idempotent (setting a field to a value) but doesn't have to be (incrementing a counter).

</div>
</div>

> These aren't just theory — they affect how browsers, load balancers, and retry logic actually behave.

---

# HTTP Status Codes — Semantic Responses

Status codes tell the client **what happened** without parsing the body.

| Code | Name | When to use |
|------|------|-------------|
| `200` | OK | Successful GET, PATCH |
| `201` | Created | Successful POST that created a resource |
| `204` | No Content | Successful DELETE (no body) |
| `301` | Moved Permanently | Resource URL has changed permanently |
| `304` | Not Modified | Cached version is still valid |
| `400` | Bad Request | Malformed JSON or missing required field |
| `401` | Unauthorized | Not authenticated |
| `403` | Forbidden | Authenticated but not allowed |
| `404` | Not Found | Resource doesn't exist |
| `405` | Method Not Allowed | e.g., DELETE on a read-only resource |
| `409` | Conflict | e.g., editing a resource that was concurrently changed |
| `429` | Too Many Requests | Rate limit exceeded |
| `500` | Internal Server Error | Bug on the server — should never reach clients |

---

# Error Responses — Design Matters

Status codes say *what happened*; the body should explain *why* and *how to fix it*.

**A well-structured error response:**
```json
{
  "error": {
    "code": "validation_error",
    "message": "Invalid input.",
    "details": [
      {"field": "title", "message": "This field is required."},
      {"field": "body", "message": "Must be at least 10 characters."}
    ]
  }
}
```

**Anti-patterns:**
- A plain string: `"Something went wrong"` — no structure, no field info
- HTML error page returned from an API — client can't parse it
- Raw stack trace: `"NoneType has no attribute 'title'"` — leaks internals

> See **RFC 7807** (Problem Details for HTTP APIs) for a standardised error format used by many public APIs.

---

# HATEOAS — Hypermedia as Navigation

**HATEOAS** (Hypermedia As The Engine Of Application State) is the most misunderstood REST constraint.

The idea: responses should contain **links** that tell the client what it can do next.

```json
{
  "id": 42,
  "title": "Hello",
  "author": "Ada",
  "_links": {
    "self": "/api/posts/42/",
    "author": "/api/users/ada/",
    "comments": "/api/posts/42/comments/",
    "edit": "/api/posts/42/",
    "delete": "/api/posts/42/"
  }
}
```

Like hyperlinks in HTML — you don't hardcode URLs, you follow links from the response.

**In practice**, most APIs ignore HATEOAS and clients hardcode URL patterns. But the GitHub API, for example, includes `_links` and `url` fields everywhere — making it navigable and forward-compatible.

---

# Richardson Maturity Model

A framework (by Leonard Richardson) for classifying how "RESTful" an API really is:

| Level | Name | What it means | Example |
|-------|------|---------------|---------|
| **0** | The Swamp of POX | One endpoint, POST everything | `POST /api` with action in body |
| **1** | Resources | Individual URLs for resources, but only POST | `POST /posts/42` |
| **2** | HTTP Verbs | Proper methods + status codes | `GET /posts/42`, `DELETE /posts/42` |
| **3** | Hypermedia | HATEOAS — responses contain navigation links | `_links: {comments: "/posts/42/comments/"}` |

Most production APIs live at **Level 2** — proper resources, methods, and status codes. Level 3 (full HATEOAS) is rare outside APIs like GitHub's.

**Useful mental model:** if you're designing an API, aim for at least Level 2. That's where the practical benefits (cacheability, predictability, tooling) come from.

---

# REST vs Alternatives

REST is not the only way to build APIs. Each style makes different trade-offs:

| | REST | GraphQL | gRPC | SOAP |
|---|------|---------|------|------|
| **Format** | JSON (usually) | JSON | Protobuf (binary) | XML |
| **Transport** | HTTP | HTTP | HTTP/2 | HTTP |
| **Schema** | OpenAPI (optional) | Built-in type system | `.proto` files | WSDL |
| **Strengths** | Simple, cacheable, universal | Client picks exact fields | Fast, typed, streaming | Enterprise, ACID transactions |
| **Weaknesses** | Over-fetching, many round trips | No caching, complex queries | Not browser-friendly | Verbose, complex |
| **Era** | 2000s–now | 2015–now | 2015–now | 1990s–2010s |

**When to use REST:** most web APIs, public APIs, when simplicity and cacheability matter.
**When to consider GraphQL:** complex UIs that need flexible queries, mobile apps where bandwidth matters.
**When to consider gRPC:** microservice-to-microservice communication, high-performance needs.

---

# An API Is a Contract

Once you publish an API, other people write code against it. That code **breaks** if you change the API.

An API is a **contract** — a promise about:
- What URLs exist and what they accept
- What HTTP methods each endpoint supports
- What the request body looks like
- What the response body looks like
- What status codes are returned and when

**Breaking changes** (removing a field, renaming a URL, changing a status code) are expensive — every client must update.

This is why API design matters: **a good design is easy to use correctly and hard to use incorrectly**.

> "APIs are like user interfaces for developers. Design them with the same care."

---

# API Versioning

How do you evolve an API without breaking existing clients?

<div class="columns">
<div>

**URL versioning (most common):**
```
/api/v1/posts/
/api/v2/posts/
```
Simple, visible, easy to route.

**Header versioning:**
```http
GET /api/posts/
Accept: application/vnd.myapp.v2+json
```
Cleaner URLs, but harder to test.

</div>
<div>

**Query parameter:**
```
/api/posts/?version=2
```
Easy to add, but feels hacky.

**No versioning (evolve carefully):**
- Only add fields, never remove
- Deprecated fields return `null`
- New behavior behind feature flags

Used by GitHub, Stripe. Requires discipline.

</div>
</div>

**The best strategy:** design carefully from the start, add fields freely, avoid breaking changes, version only when you must.

---

# JSON in APIs — Conventions That Matter

You know JSON. Here's what matters for **API design**:

- **Dates** have no native type — use **ISO 8601**: `"2025-10-14T09:31:05+00:00"`
- **Null vs. absent** — decide on a convention and stick to it (`"deleted_at": null` vs. omitting the field)
- **Naming** — pick `snake_case` or `camelCase` and be consistent across all endpoints
- **Envelope pattern** — wrap collections in metadata:
  ```json
  {"count": 87, "next": "/api/posts/?page=2", "results": [...]}
  ```
  vs. bare arrays `[{...}, {...}]` — envelopes are easier to extend without breaking clients
- **No trailing commas, no comments** — JSON is strict; a single trailing comma breaks parsing

---

# From Concepts to Django — A Preview

The ideas from this lecture map directly to Django code. The **lab** will cover the implementation in detail.

| Concept | Django implementation |
|---------|---------------------|
| JSON responses | `JsonResponse` (manual) or DRF `Serializer` |
| HTTP methods → CRUD | `View` subclass with `get()`, `post()`, `patch()`, `delete()` |
| Status codes | `return JsonResponse(data, status=201)` |
| Validation | Manual checks or DRF `.is_valid()` |
| Browsable docs | DRF Browsable API + `drf-spectacular` → Swagger UI |
| Authentication | Session cookies or `Authorization: Token ...` header |

**Django REST Framework** (DRF) is the de-facto standard — it provides serializers, viewsets, routers, pagination, and permissions out of the box. Full CRUD in ~10 lines of code.

> You'll build it by hand first, then see what DRF saves you.

---

# Nested Resources and URL Design

Model relationships are expressed as **URL nesting**:

```
/api/posts/                      # all posts
/api/posts/42/                   # post #42
/api/posts/42/comments/          # comments on post #42
/api/posts/42/comments/7/        # comment #7 on post #42
```

**Design guidelines:**
- Keep nesting to **two levels** maximum — deeper nesting becomes unmanageable
- Use **query parameters** for filtering, sorting, and searching:
  ```
  /api/posts/?author=ada              # filter by field
  /api/posts/?ordering=-created_at    # sort (- = descending)
  /api/posts/?search=django           # full-text search
  /api/posts/?fields=id,title         # sparse fields (return less data)
  /api/comments/?post=42&author=ada   # alternative to deep nesting
  ```
- Always **sanitise and cap** numeric parameters — never let a client ask for 100,000 rows

---

# Pagination — Don't Return Everything

Large datasets must be paginated — never dump all records in one response.

**Common patterns:**

<div class="columns">
<div>

**Offset-based (simple):**
```json
{
  "count": 87,
  "page": 2,
  "page_size": 20,
  "next": "/api/posts/?page=3",
  "previous": "/api/posts/?page=1",
  "results": [...]
}
```
Easy to understand.
Skipping pages is costly on large tables.

</div>
<div>

**Cursor-based (scalable):**
```json
{
  "next_cursor": "eyJpZCI6IDQyfQ",
  "results": [...]
}
```
Opaque cursor = no page-skipping.
Consistent performance at any depth.
Used by Twitter, Slack, Stripe APIs.

</div>
</div>

**Always include metadata** (total count, next link) — clients need to know when to stop fetching.

---

# Authentication in APIs

You covered sessions and cookies in Lecture 6. For APIs, the key shift is:

| | Traditional (HTML views) | API |
|---|---|---|
| **Identity** | `sessionid` cookie (automatic) | Token/key in `Authorization` header (explicit) |
| **CSRF** | Needed (browser sends cookies automatically) | Not needed with tokens (no cookies) |
| **Cross-domain** | Tricky (CORS + credentials) | Easy (token travels in header) |

```http
GET /api/posts/ HTTP/1.1
Authorization: Token 9944b09199c62bcf
```

**The REST connection:** statelessness means the server doesn't store per-client state. Every request carries its own proof of identity — whether that's a cookie, a token, a JWT, or an API key.

For this course, session auth is fine. Real-world APIs typically use **JWT** or **OAuth2**.

---

# Rate Limiting — Protecting Your API

Without rate limits, a single client can:
- Overload your server (accidentally or maliciously)
- Scrape all your data
- Trigger huge cloud bills

**Rate limiting** restricts how many requests a client can make in a time window.

```http
HTTP/1.1 200 OK
X-RateLimit-Limit: 100
X-RateLimit-Remaining: 42
X-RateLimit-Reset: 1698000000

HTTP/1.1 429 Too Many Requests
Retry-After: 30
{"error": "rate limit exceeded"}
```

**Common strategies:**
- Per user/token: 100 requests/minute
- Per IP: 30 requests/minute (unauthenticated)
- Per endpoint: write endpoints have stricter limits than reads

DRF has built-in throttling classes. In production, this is often handled by a reverse proxy (Nginx, Cloudflare).

---

# CORS — The API Angle

You learned about the **Same-Origin Policy** in Lecture 6. Here's the part that matters for APIs:

When your React app on `localhost:3000` calls your Django API on `localhost:8000`, the browser **blocks it** unless the server sends CORS headers.

**What's new for APIs:** requests with `Content-Type: application/json` or `Authorization` headers trigger a **preflight** — the browser sends `OPTIONS` first:

```
Browser                          API server
  ──── OPTIONS /api/posts/ ────►
  ◄─── 200 + CORS headers ──────  (Access-Control-Allow-Origin, etc.)
  ──── POST /api/posts/ ───────►
  ◄─── 201 JSON ────────────────
```

In Django, `django-cors-headers` middleware handles this. The key setting: `CORS_ALLOWED_ORIGINS`.

> CORS is **browser-only** — `curl`, mobile apps, and server-to-server calls are not affected.

---

# Consuming APIs — Clients and Tools

An API endpoint is useless until something calls it. Common clients:

| Client | How it calls the API | Use case |
|--------|---------------------|----------|
| Browser JS (`fetch`) | `await fetch("/api/posts/")` | SPA, dynamic page updates |
| CLI (`curl`, `httpie`) | `http GET localhost:8000/api/posts/` | Testing, scripting |
| Mobile app | HTTP library (Retrofit, Alamofire) | iOS/Android clients |
| Another server | HTTP client (requests, httpx) | Microservices, webhooks |
| Generated client | From OpenAPI spec | Type-safe, auto-updated |

**Key `fetch()` gotcha:** it does **not** throw on 4xx/5xx — a 404 is a successful HTTP response! Always check `response.ok`.

> The lab will cover `fetch()` in detail. For testing during development, try **httpie**: `uv tool install httpie`

---

# Real-World REST APIs — Learning from the Wild

Study how major APIs are designed — they reflect years of iteration:

**GitHub API** (`api.github.com`):
- Consistent resource URLs: `/repos/{owner}/{repo}/issues`
- HATEOAS links in every response
- Pagination via `Link` header
- Rate limiting: 5000 req/hr authenticated

**Stripe API** (`api.stripe.com`):
- Versioning via date-based header: `Stripe-Version: 2023-10-16`
- Idempotency keys for safe retries on POST
- Expandable fields: `?expand[]=customer`

**Common patterns across all good APIs:**
- Consistent naming (plural nouns, `snake_case` or `camelCase` — pick one)
- Predictable error format: `{"error": {"code": "...", "message": "..."}}`
- Pagination metadata in every list response
- Comprehensive documentation with examples

---

# API Design Best Practices

<div class="columns">
<div>

**✅ Do:**
- Use plural nouns: `/posts/`, `/users/`
- Return proper status codes
- Be consistent in naming and structure
- Include error messages in the body
- Paginate all list endpoints
- Version if you expect breaking changes
- Document with OpenAPI / Swagger

</div>
<div>

**❌ Don't:**
- Put verbs in URLs: `/getPost/`, `/deleteUser/`
- Return 200 for errors
- Expose internal field names or IDs
- Return HTML from an API endpoint
- Change response shapes between endpoints
- Ignore idempotency in mutation endpoints
- Let clients request unbounded datasets

</div>
</div>

> Design your API as if someone who has never seen your codebase will use it. Because they will.

---

# Common Design Mistakes

<div class="columns">
<div>

**URL anti-patterns:**
```
# Bad — verbs in URLs
GET  /getPosts/
POST /createPost/
GET  /deletePost/?id=5

# Good — nouns + methods
GET    /api/posts/
POST   /api/posts/
DELETE /api/posts/5/
```

**Status code mistakes:**
```python
# Bad — 200 with error body
return JsonResponse(
    {"error": "not found"},
    status=200  # ← wrong!
)
```

</div>
<div>

**Inconsistent responses:**
```python
# Bad — different shapes
{"post": {...}}  # detail
[{...}]          # list

# Good — consistent envelope
{"results": [...], "count": 2}
{"id": 1, "title": "Hello"}
```

**Leaking internals:**
```python
# Bad — raw exception
{"error": "RelatedObjectDoesNotExist:
  Post has no author."}

# Good — generic message
{"error": "internal server error"}
```

</div>
</div>

---

# API Documentation — OpenAPI

**OpenAPI 3.0** (formerly Swagger) is a standardised machine-readable format for describing REST APIs.

**Why it matters:**
- Auto-generate API clients in any language (Python, TypeScript, Swift…)
- Interactive documentation (Swagger UI) — try endpoints live in the browser
- Contract tests — verify your implementation matches the spec
- A single source of truth that stays in sync with your code

```yaml
paths:
  /api/posts/:
    get:
      summary: List all posts
      responses:
        '200':
          content:
            application/json:
              schema:
                type: array
                items:
                  $ref: '#/components/schemas/Post'
```

> In the lab, you'll use `drf-spectacular` to auto-generate this from your code.

---

# Summary

REST APIs let Django serve **any client** — browser, mobile, CLI, third-party service.

**Ideas to remember:**
- **REST** is an architectural style from Roy Fielding (2000), not a protocol
- **Resources** are nouns (URLs); **HTTP methods** are verbs
- **Statelessness** — every request is self-contained; enables scaling
- **Safety** (GET doesn't change state) and **idempotency** (retrying is safe) are contracts
- **Status codes** carry meaning — use them correctly; **error bodies** explain what went wrong
- **Content negotiation** — same resource, multiple representations
- **HATEOAS** and the **Richardson Maturity Model** — levels of RESTfulness
- **API = contract** — design carefully, breaking changes are expensive
- **Versioning, pagination, rate limiting** — production essentials
- **CORS** — the server opts in to cross-origin browser requests
- **OpenAPI** — machine-readable API documentation

---

# Lab 7 Preview

**What you'll build:**
- A JSON REST API for the blog app from Labs 4–6
- Endpoints: `GET /api/posts/`, `POST /api/posts/`, `GET /api/posts/<pk>/`, `PATCH /api/posts/<pk>/`, `DELETE /api/posts/<pk>/`
- A `GET /api/posts/?search=` filter endpoint
- A JavaScript frontend that fetches and renders posts using `fetch()` and `createElement`
- A DRF-powered version of the same API using `ModelViewSet` and `DefaultRouter`
- OpenAPI schema generation with `drf-spectacular` + Swagger UI at `/api/swagger/`

**Check your understanding:**
1. What does "statelessness" mean for API design? How does the client prove identity?
2. Why is `GET` safe and `POST` not idempotent? What are the practical consequences?
3. What is a "resource" and how does it differ from its representation?
4. Why does `fetch()` not throw on a 404 response?
5. What problem does API versioning solve?

---

# Questions?

**Next lecture:** Django — Forms, ModelForms & Validation

Recommended reading:
- Fielding's dissertation, Chapter 5: fielding.net/pubs/dissertation/rest_arch_style.htm
- Martin Fowler: Richardson Maturity Model — martinfowler.com/articles/richardsonMaturityModel.html
- MDN: HTTP response status codes — developer.mozilla.org/en-US/docs/Web/HTTP/Status
- MDN: Using the Fetch API — developer.mozilla.org/en-US/docs/Web/API/Fetch_API/Using_Fetch
- GitHub REST API docs — docs.github.com/en/rest (excellent real-world example)
- RFC 7807: Problem Details for HTTP APIs — tools.ietf.org/html/rfc7807

Useful tools:
- **httpie** (`uv tool install httpie`) — friendlier `curl` alternative: `http GET localhost:8000/api/posts/`
- **Bruno** or **Insomnia** — GUI REST clients (open-source alternatives to Postman)
- **DRF Browsable API** — install DRF and visit `/api/` in the browser for a clickable interface

---
{% endraw %}
