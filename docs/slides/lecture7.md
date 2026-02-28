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
## Django — REST APIs & JSON

**WWW 25/26**
REST · JSON · JsonResponse · fetch() · CORS

---

# Why REST APIs?

A traditional Django view returns **HTML** — a full page the browser renders.

But modern apps need **data**, not HTML:
- A React/Vue/Svelte frontend wants raw data to render itself
- A mobile app needs structured responses, not markup
- A third-party service needs machine-readable output

**Solution:** build a second kind of view — an **API endpoint** that returns **JSON**.

```
Browser                 Django (traditional)
  ──── GET /posts/ ──►  renders posts.html ──► full HTML page

JavaScript app          Django (API)
  ── GET /api/posts/ ►  returns JSON list  ──► [{id:1, title:...}, ...]
```

The frontend and backend become **decoupled**: they communicate through a stable contract (the API), and either side can change independently.

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

# REST Principles

**REST** (Representational State Transfer) is an architectural style, not a protocol.

The six constraints:

| Constraint | Meaning |
|------------|---------|
| **Stateless** | Each request carries all information needed; server holds no session per request |
| **Client–server** | UI and data storage are separated |
| **Uniform interface** | Consistent conventions for all resources |
| **Cacheable** | Responses declare whether they can be cached |
| **Layered system** | Client doesn't know if it's talking to the real server |
| **Code on demand** (optional) | Server can send executable code |

The most important for day-to-day API design: **stateless** + **uniform interface**.

---

# REST Resource Conventions

Resources are **nouns** in the URL. HTTP **methods** are the verbs.

| Method | URL | Action |
|--------|-----|--------|
| `GET` | `/api/posts/` | List all posts |
| `POST` | `/api/posts/` | Create a new post |
| `GET` | `/api/posts/42/` | Retrieve post #42 |
| `PUT` | `/api/posts/42/` | Replace post #42 entirely |
| `PATCH` | `/api/posts/42/` | Partially update post #42 |
| `DELETE` | `/api/posts/42/` | Delete post #42 |

**Golden rules:**
- URLs are nouns (`/posts/`, `/users/`, `/comments/`) — never verbs (`/getPost/`, `/deleteUser/`)
- `GET` must be **safe** (no side effects)
- `PUT`/`DELETE`/`PATCH` must be **idempotent** (same result if repeated)

---

# HTTP Status Codes for APIs

Returning the right status code is as important as the response body.

| Code | Name | When to use |
|------|------|-------------|
| `200` | OK | Successful GET or PATCH |
| `201` | Created | Successful POST that created a resource |
| `204` | No Content | Successful DELETE (no body) |
| `400` | Bad Request | Malformed JSON or missing required field |
| `401` | Unauthorized | Not authenticated |
| `403` | Forbidden | Authenticated but not allowed |
| `404` | Not Found | Resource doesn't exist |
| `422` | Unprocessable Entity | Data is valid JSON but fails validation |
| `500` | Internal Server Error | Unhandled exception — should never reach clients |

```python
# Good
return JsonResponse({"error": "title is required"}, status=400)

# Bad — never return 200 with an error body
return JsonResponse({"error": "not found"}, status=200)  # ← wrong!
```

---

# JSON — The Language of APIs

**JSON** (JavaScript Object Notation) — a text format for structured data.

```json
{
  "id": 1,
  "title": "Hello Django",
  "published": true,
  "views": 42,
  "tags": ["python", "web"],
  "author": {
    "name": "Ada",
    "email": "ada@example.com"
  },
  "deleted_at": null
}
```

Rules:
- Keys must be **double-quoted strings**
- Values: string, number, boolean (`true`/`false`), array, object, `null`
- No trailing commas
- No comments

JSON is **language-agnostic** — Python, JavaScript, Go, Rust all speak it.

---

# Python `json` Module

```python
import json

# Python dict → JSON string
data = {"title": "Hello", "views": 42, "published": True}
text = json.dumps(data)
# '{"title": "Hello", "views": 42, "published": true}'

# Pretty-print
text = json.dumps(data, indent=2)

# JSON string → Python dict
raw = '{"title": "Hello", "views": 42}'
data = json.loads(raw)
print(data["title"])  # Hello

# Python types map to JSON types:
# dict  → object
# list  → array
# str   → string
# int/float → number
# True/False → true/false
# None  → null
```

`json.dumps` raises `TypeError` if the object contains non-serialisable types (e.g., `datetime`, Django model instances).

---

# Django's `JsonResponse`

`JsonResponse` is a subclass of `HttpResponse` that:
1. Calls `json.dumps()` on the data you pass
2. Sets `Content-Type: application/json` automatically

```python
from django.http import JsonResponse

def post_list(request):
    data = [
        {"id": 1, "title": "Hello"},
        {"id": 2, "title": "World"},
    ]
    return JsonResponse(data, safe=False)
```

**`safe=False`** is required when the top-level value is a list (not a dict).  
This prevents a historical JSON security issue in old browsers.

```python
# Dict — no safe=False needed
return JsonResponse({"count": 2, "results": [...]})

# List — must pass safe=False
return JsonResponse([...], safe=False)
```

The response automatically has status `200` unless you pass `status=`.

---

# Serialising Model Instances

Django model instances are **not JSON-serialisable** by default:

```python
post = Post.objects.get(pk=1)
json.dumps(post)  # TypeError: Object of type Post is not JSON serializable
```

The simplest fix: write a helper that converts a model to a plain dict.

```python
def post_to_dict(post):
    return {
        "id": post.pk,
        "title": post.title,
        "body": post.body,
        "author": post.author.username,
        "created_at": post.created_at.isoformat(),
        "updated_at": post.updated_at.isoformat(),
    }
```

Why explicit? You control exactly which fields are exposed.  
Automatic serialisation (e.g., Django's `serializers.serialize`) often leaks internal fields or requires extra parsing on the client.

```python
return JsonResponse(post_to_dict(post))
return JsonResponse([post_to_dict(p) for p in qs], safe=False)
```

---

# Class-Based Views for APIs

`django.views.View` is a great base for API views — it dispatches to `get()`, `post()`, `patch()`, `delete()` methods automatically.

```python
from django.views import View
from django.http import JsonResponse

class PostListView(View):
    def get(self, request):
        posts = Post.objects.all()
        return JsonResponse([post_to_dict(p) for p in posts], safe=False)

    def post(self, request):
        # create a new post
        ...
```

Wire it up in `urls.py`:

```python
from django.urls import path
from .views import PostListView, PostDetailView

urlpatterns = [
    path("api/posts/", PostListView.as_view()),
    path("api/posts/<int:pk>/", PostDetailView.as_view()),
]
```

The `dispatch()` method routes by `request.method.lower()` — if you don't define `patch()`, Django returns `405 Method Not Allowed` automatically.

---

# `@csrf_exempt` and `method_decorator`

By default, Django requires a **CSRF token** on all `POST`, `PUT`, `PATCH`, and `DELETE` requests from forms. API clients (mobile apps, `curl`, other servers) don't send this token.

For API views consumed by non-browser clients, disable CSRF checking:

```python
from django.views import View
from django.views.decorators.csrf import csrf_exempt
from django.utils.decorators import method_decorator

@method_decorator(csrf_exempt, name="dispatch")
class PostListView(View):
    def get(self, request):
        ...

    def post(self, request):
        ...
```

`method_decorator` wraps the **class-based view** with a function decorator.  
`name="dispatch"` applies it to the entry point, so all methods are covered.

**Note:** In production, use token-based auth (e.g., DRF's `TokenAuthentication`) instead of disabling CSRF.

---

# Reading a JSON Request Body

The incoming JSON lives in `request.body` as **bytes**. You must decode it.

```python
import json
from django.http import JsonResponse

def post(self, request):
    try:
        data = json.loads(request.body)
    except json.JSONDecodeError:
        return JsonResponse({"error": "invalid JSON"}, status=400)

    title = data.get("title", "").strip()
    if not title:
        return JsonResponse({"error": "title is required"}, status=400)

    body = data.get("body", "")
    # ... create the object
```

**Common mistakes:**
- Reading `request.POST` instead of `request.body` — `POST` only works for `application/x-www-form-urlencoded`
- Not catching `JSONDecodeError` — any badly-formed request crashes with 500
- Forgetting `.strip()` — a title of `"   "` passes `if title` but is not valid

---

# The Full GET List Endpoint

```python
from django.views import View
from django.http import JsonResponse
from django.utils.decorators import method_decorator
from django.views.decorators.csrf import csrf_exempt
from .models import Post

def post_to_dict(post):
    return {
        "id": post.pk,
        "title": post.title,
        "body": post.body,
        "author": post.author.username,
        "created_at": post.created_at.isoformat(),
    }

@method_decorator(csrf_exempt, name="dispatch")
class PostListView(View):
    def get(self, request):
        qs = Post.objects.select_related("author").order_by("-created_at")
        return JsonResponse([post_to_dict(p) for p in qs], safe=False)
```

`select_related("author")` avoids **N+1 queries** — without it, each `post.author.username` triggers a separate SQL query.

`order_by("-created_at")` — newest first (minus = descending).

---

# The POST Create Endpoint

```python
import json
from django.http import JsonResponse

@method_decorator(csrf_exempt, name="dispatch")
class PostListView(View):
    def get(self, request):
        ...  # as above

    def post(self, request):
        try:
            data = json.loads(request.body)
        except json.JSONDecodeError:
            return JsonResponse({"error": "invalid JSON"}, status=400)

        title = data.get("title", "").strip()
        if not title:
            return JsonResponse({"error": "title is required"}, status=400)

        post = Post.objects.create(
            title=title,
            body=data.get("body", ""),
            author=request.user,
        )
        return JsonResponse(post_to_dict(post), status=201)
```

Return **201 Created** (not 200) and the newly created resource in the body.  
The client now has the `id` and `created_at` assigned by the server.

---

# The GET Detail Endpoint

```python
from django.http import JsonResponse
from django.views import View
from .models import Post

@method_decorator(csrf_exempt, name="dispatch")
class PostDetailView(View):
    def _get_post(self, pk):
        try:
            return Post.objects.select_related("author").get(pk=pk)
        except Post.DoesNotExist:
            return None

    def get(self, request, pk):
        post = self._get_post(pk)
        if post is None:
            return JsonResponse({"error": "not found"}, status=404)
        return JsonResponse(post_to_dict(post))
```

`pk` comes from the URL pattern `<int:pk>` and is passed as a keyword argument to every method.

The private helper `_get_post()` is shared by `get`, `patch`, and `delete` — avoids repeating the try/except.

**Never** raise `Http404` in an API view — it returns HTML, confusing API clients.

---

# The PATCH Update Endpoint

```python
    def patch(self, request, pk):
        post = self._get_post(pk)
        if post is None:
            return JsonResponse({"error": "not found"}, status=404)

        try:
            data = json.loads(request.body)
        except json.JSONDecodeError:
            return JsonResponse({"error": "invalid JSON"}, status=400)

        # Only update fields that were sent
        if "title" in data:
            title = data["title"].strip()
            if not title:
                return JsonResponse({"error": "title cannot be blank"}, status=400)
            post.title = title

        if "body" in data:
            post.body = data["body"]

        post.save()
        return JsonResponse(post_to_dict(post))
```

`PATCH` = **partial update** — only fields present in the request are changed.  
`PUT` = full replacement — absent fields would be cleared.  
Prefer `PATCH` for typical edit operations.

---

# The DELETE Endpoint

```python
    def delete(self, request, pk):
        post = self._get_post(pk)
        if post is None:
            return JsonResponse({"error": "not found"}, status=404)

        post.delete()
        return JsonResponse({}, status=204)
```

**204 No Content** is the correct status for a successful delete.  
The response body is empty (or `{}`).

Some APIs return `200` with a `{"deleted": true}` body — both are acceptable, but 204 is the REST convention.

```bash
# curl test
curl -X DELETE http://localhost:8000/api/posts/42/
# HTTP/1.1 204 No Content
```

After deletion, a `GET /api/posts/42/` should return `404`.  
Make sure your helper method queries the database fresh each time.

---

# Nested Resources

Model relationships are expressed as **URL nesting**:

```
/api/posts/                      # all posts
/api/posts/42/                   # post #42
/api/posts/42/comments/          # comments on post #42
/api/posts/42/comments/7/        # comment #7 on post #42
```

```python
# urls.py
urlpatterns = [
    path("api/posts/", PostListView.as_view()),
    path("api/posts/<int:post_pk>/", PostDetailView.as_view()),
    path("api/posts/<int:post_pk>/comments/", CommentListView.as_view()),
    path("api/posts/<int:post_pk>/comments/<int:pk>/", CommentDetailView.as_view()),
]

# views.py
class CommentListView(View):
    def get(self, request, post_pk):
        post = get_object_or_404(Post, pk=post_pk)
        comments = post.comments.select_related("author").order_by("created_at")
        return JsonResponse([comment_to_dict(c) for c in comments], safe=False)
```

Keep nesting to **two levels** maximum — deeper nesting becomes unmanageable.

---

# Query Parameters for Filtering

`request.GET` contains query parameters (despite the name, it works for any GET request).

```python
# GET /api/posts/?search=django&author=ada&limit=10

class PostListView(View):
    def get(self, request):
        qs = Post.objects.select_related("author").order_by("-created_at")

        search = request.GET.get("search", "").strip()
        if search:
            qs = qs.filter(title__icontains=search)

        author = request.GET.get("author", "").strip()
        if author:
            qs = qs.filter(author__username=author)

        try:
            limit = int(request.GET.get("limit", 20))
            limit = min(limit, 100)   # cap at 100
        except ValueError:
            limit = 20

        qs = qs[:limit]
        return JsonResponse([post_to_dict(p) for p in qs], safe=False)
```

Always **sanitise and cap** numeric parameters — never let a client ask for 100 000 rows.

---

# Authentication in APIs

<div class="columns">
<div>

**Session auth (cookies)**
- Django's default login sets a `sessionid` cookie
- Works for browser-based `fetch()` calls on the same domain
- CSRF still needed for state-changing requests
- Simple: reuse Django's existing login view

```python
if not request.user.is_authenticated:
    return JsonResponse(
        {"error": "login required"},
        status=401
    )
```

</div>
<div>

**Token auth (intro)**
- Client sends `Authorization: Token abc123` header
- No cookies, no CSRF
- Works across domains and from mobile apps
- Django REST Framework provides `TokenAuthentication`
- Tokens stored in database, revocable

```http
GET /api/posts/ HTTP/1.1
Host: api.example.com
Authorization: Token 9944b09199c62bcf
```

</div>
</div>

For this course, session auth is fine. Real projects often use JWT (JSON Web Tokens) or OAuth2.

---

# `curl` for API Testing

`curl` lets you test every HTTP method from the terminal.

```bash
# GET list
curl http://localhost:8000/api/posts/

# GET with pretty-print (pipe to python -m json.tool)
curl -s http://localhost:8000/api/posts/ | python -m json.tool

# POST — create a post
curl -X POST http://localhost:8000/api/posts/ \
  -H "Content-Type: application/json" \
  -d '{"title": "Hello", "body": "World"}'

# PATCH — partial update
curl -X PATCH http://localhost:8000/api/posts/1/ \
  -H "Content-Type: application/json" \
  -d '{"title": "Updated title"}'

# DELETE
curl -X DELETE http://localhost:8000/api/posts/1/ -v

# Check status codes
curl -o /dev/null -s -w "%{http_code}\n" http://localhost:8000/api/posts/999/
# 404
```

Always send `-H "Content-Type: application/json"` when posting JSON.

---

# The `fetch()` API — Basics

`fetch()` is the modern browser API for HTTP requests. It returns a `Promise`.

```javascript
// Simple GET
fetch("/api/posts/")
  .then(response => response.json())
  .then(posts => console.log(posts))
  .catch(err => console.error(err));
```

With `async/await` (cleaner):

```javascript
async function loadPosts() {
  const response = await fetch("/api/posts/");
  if (!response.ok) {
    throw new Error(`HTTP error: ${response.status}`);
  }
  const posts = await response.json();
  return posts;
}
```

Key points:
- `fetch()` only rejects on **network errors** — a 404 does not throw!
- Always check `response.ok` (true for 200–299) or `response.status` explicitly
- `.json()` is also async — you must `await` it

---

# `fetch()` — POST, PATCH, DELETE

```javascript
// POST — create a new post
async function createPost(title, body) {
  const response = await fetch("/api/posts/", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ title, body }),
  });
  if (!response.ok) {
    const err = await response.json();
    throw new Error(err.error);
  }
  return await response.json();  // returns the created post (201)
}

// PATCH — partial update
async function updatePost(id, changes) {
  const response = await fetch(`/api/posts/${id}/`, {
    method: "PATCH",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(changes),
  });
  return await response.json();
}

// DELETE
async function deletePost(id) {
  await fetch(`/api/posts/${id}/`, { method: "DELETE" });
}
```

---

# Consuming the API — Rendering Posts Dynamically

```javascript
async function renderPosts() {
  const container = document.getElementById("posts");
  container.innerHTML = "<p>Loading…</p>";

  try {
    const posts = await loadPosts();
    container.innerHTML = "";

    posts.forEach(post => {
      const article = document.createElement("article");

      const h2 = document.createElement("h2");
      h2.textContent = post.title;

      const meta = document.createElement("p");
      meta.textContent = `by ${post.author} on ${post.created_at}`;

      const body = document.createElement("p");
      body.textContent = post.body;

      article.append(h2, meta, body);
      container.append(article);
    });
  } catch (err) {
    container.innerHTML = `<p class="error">Failed to load: ${err.message}</p>`;
  }
}

document.addEventListener("DOMContentLoaded", renderPosts);
```

Use `textContent` (not `innerHTML`) to avoid **XSS** — never insert user data as HTML.

---

# CORS — Cross-Origin Resource Sharing

A browser **blocks** a page on `http://frontend.com` from calling `http://api.example.com/` by default.

This is the **Same-Origin Policy** — origins must match (scheme + host + port).

CORS is the mechanism by which a server **opts in** to cross-origin requests:

```http
HTTP/1.1 200 OK
Access-Control-Allow-Origin: https://frontend.com
Access-Control-Allow-Methods: GET, POST, PATCH, DELETE
Access-Control-Allow-Headers: Content-Type, Authorization
```

**Preflight:** before a `POST`/`PATCH`/`DELETE`, the browser sends an `OPTIONS` request. The server must respond with the correct headers or the real request is blocked.

```
Browser                          API server
  ──── OPTIONS /api/posts/ ────►
  ◄─── 200 + CORS headers ──────
  ──── POST /api/posts/ ───────►
  ◄─── 201 JSON ────────────────
```

---

# CORS in Django

Install `django-cors-headers`:

```bash
uv add django-cors-headers
```

```python
# settings.py
INSTALLED_APPS = [
    ...
    "corsheaders",
    ...
]

MIDDLEWARE = [
    "corsheaders.middleware.CorsMiddleware",   # must be first
    "django.middleware.common.CommonMiddleware",
    ...
]

# Allow a specific frontend origin
CORS_ALLOWED_ORIGINS = [
    "http://localhost:3000",
    "https://myfrontend.com",
]

# Or allow all origins (development only — never in production!)
CORS_ALLOW_ALL_ORIGINS = True
```

The middleware automatically adds the correct `Access-Control-Allow-*` headers and handles `OPTIONS` preflight requests.

---

# Django REST Framework (DRF)

`django-rest-framework` is the de-facto standard for Django APIs.  
It builds on top of the manual approach you just learned.

**What it adds:**

| Feature | Manual views | DRF |
|---------|-------------|-----|
| Serialisation | `post_to_dict()` helper | `ModelSerializer` class |
| Validation | `if not title: ...` | Serializer `.is_valid()` |
| Browsable API | No | HTML form in browser |
| Authentication | Manual check | Pluggable backends |
| Permissions | Manual check | `IsAuthenticated`, etc. |
| Pagination | Manual slice | `PageNumberPagination` |
| ViewSets | Manual class | `ModelViewSet` (5 lines) |

```python
# DRF — a full CRUD API in ~10 lines
class PostSerializer(serializers.ModelSerializer):
    class Meta:
        model = Post
        fields = ["id", "title", "body", "author", "created_at"]

class PostViewSet(viewsets.ModelViewSet):
    queryset = Post.objects.all()
    serializer_class = PostSerializer
```

---

# DRF — Minimal Setup

```bash
uv add djangorestframework
```

```python
# settings.py
INSTALLED_APPS = [
    ...
    "rest_framework",
]

REST_FRAMEWORK = {
    "DEFAULT_AUTHENTICATION_CLASSES": [
        "rest_framework.authentication.SessionAuthentication",
    ],
    "DEFAULT_PERMISSION_CLASSES": [
        "rest_framework.permissions.IsAuthenticatedOrReadOnly",
    ],
}
```

```python
# urls.py
from rest_framework.routers import DefaultRouter
from .views import PostViewSet

router = DefaultRouter()
router.register("posts", PostViewSet)

urlpatterns = [
    path("api/", include(router.urls)),
]
```

Visit `/api/` in the browser — DRF shows a clickable, self-documenting interface.

---

# DRF — Serializer in Depth

A **serializer** replaces your hand-written `post_to_dict()`. It handles both serialisation (Python → JSON) and deserialisation + validation (JSON → Python).

```python
# blog/serializers.py
from rest_framework import serializers
from .models import Post, Comment

class CommentSerializer(serializers.ModelSerializer):
    class Meta:
        model  = Comment
        fields = ["id", "author", "body", "created"]

class PostSerializer(serializers.ModelSerializer):
    comments = CommentSerializer(many=True, read_only=True)

    class Meta:
        model  = Post
        fields = ["id", "title", "slug", "body",
                  "pub_date", "category", "comments"]
```

**In a view:**

```python
serializer = PostSerializer(post)
return JsonResponse(serializer.data)           # serialise

serializer = PostSerializer(data=request.data)
if serializer.is_valid():
    serializer.save()                          # validate + create
else:
    return JsonResponse(serializer.errors, status=400)
```

---

# DRF — ModelViewSet: 5 Lines = Full CRUD

```python
# blog/drf_views.py
from rest_framework import viewsets, permissions
from .models import Post
from .serializers import PostSerializer

class PostViewSet(viewsets.ModelViewSet):
    queryset           = Post.objects.all()
    serializer_class   = PostSerializer
    permission_classes = [permissions.IsAuthenticatedOrReadOnly]
```

`ModelViewSet` auto-generates **6 actions**:

| Action | Method | URL |
|--------|--------|-----|
| `list` | GET | `/api/posts/` |
| `create` | POST | `/api/posts/` |
| `retrieve` | GET | `/api/posts/1/` |
| `update` | PUT | `/api/posts/1/` |
| `partial_update` | PATCH | `/api/posts/1/` |
| `destroy` | DELETE | `/api/posts/1/` |

---

# DRF — Router Wires It All Up

```python
# blog/drf_urls.py
from rest_framework.routers import DefaultRouter
from .drf_views import PostViewSet

router = DefaultRouter()
router.register("posts", PostViewSet)

urlpatterns = router.urls
```

```python
# mysite/urls.py
path("api/", include("blog.drf_urls")),
```

**What `DefaultRouter` generates:**

```
GET  /api/          → browsable API root (HTML in browser, JSON via curl)
GET  /api/posts/    → list
POST /api/posts/    → create
GET  /api/posts/1/  → retrieve
PUT  /api/posts/1/  → update
PATCH /api/posts/1/ → partial update
DELETE /api/posts/1/→ destroy
```

---

# OpenAPI — What It Is

**OpenAPI 3.0** (formerly Swagger) is a standardised format for describing REST APIs as a YAML or JSON document.

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
    post:
      summary: Create a post
      requestBody:
        required: true
        content:
          application/json:
            schema:
              $ref: '#/components/schemas/Post'
```

**Why it matters:**
- Auto-generate API clients in any language
- Generate interactive documentation (Swagger UI)
- Run contract tests against the spec

---

# OpenAPI with `drf-spectacular`

```bash
uv add drf-spectacular
```

```python
# settings.py
INSTALLED_APPS = [..., "drf_spectacular"]

REST_FRAMEWORK = {
    "DEFAULT_SCHEMA_CLASS": "drf_spectacular.openapi.AutoSchema",
}
```

Add the schema endpoints to `urls.py`:

```python
from drf_spectacular.views import (
    SpectacularAPIView,
    SpectacularSwaggerView,
    SpectacularRedocView,
)

urlpatterns += [
    path("api/schema/",  SpectacularAPIView.as_view(),  name="schema"),
    path("api/swagger/", SpectacularSwaggerView.as_view(url_name="schema"),
                                                         name="swagger-ui"),
    path("api/redoc/",   SpectacularRedocView.as_view(url_name="schema"),
                                                         name="redoc"),
]
```

Generate a static file:

```bash
uv run python manage.py spectacular --file schema.yaml
```

---

# Swagger UI

`http://127.0.0.1:8000/api/swagger/` opens an **interactive browser UI**:

```
┌────────────────────────────────────────────┐
│  My Blog API  v1.0                         │
├────────────────────────────────────────────┤
│  ▶ GET  /api/posts/        List posts      │
│  ▶ POST /api/posts/        Create post     │
│  ▶ GET  /api/posts/{id}/   Retrieve post   │
│  ▶ PATCH /api/posts/{id}/  Update post     │
│  ▶ DELETE /api/posts/{id}/ Delete post     │
└────────────────────────────────────────────┘
```

**What you can do in Swagger UI:**
- Click any endpoint → expands with parameters, request body schema, response examples
- Click **Try it out** → fill in fields and execute the real request
- See the actual `curl` command it sends
- Inspect the response body, status code, and headers live

---

# Annotating DRF Views for Better Docs

`drf-spectacular` reads Python type hints and docstrings. You can improve the generated schema:

```python
from drf_spectacular.utils import extend_schema, OpenApiParameter

class PostViewSet(viewsets.ModelViewSet):
    queryset         = Post.objects.all()
    serializer_class = PostSerializer

    @extend_schema(
        parameters=[
            OpenApiParameter("search", str,
                             description="Filter posts by title"),
        ],
        summary="List all blog posts",
    )
    def list(self, request, *args, **kwargs):
        query = request.query_params.get("search", "")
        if query:
            self.queryset = self.queryset.filter(title__icontains=query)
        return super().list(request, *args, **kwargs)
```

The `@extend_schema` decorator is optional — `drf-spectacular` auto-generates reasonable docs even without it.

---

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
# Bad
return JsonResponse(
    {"error": "not found"},
    status=200   # ← wrong!
)

# Bad
return JsonResponse(
    {"data": post},
    status=404   # ← confusing!
)
```

</div>
<div>

**Body mistakes:**
```python
# Bad — returning HTML on error
# (Django's default 500 page)
raise ValueError("oops")

# Good — always JSON
try:
    ...
except Exception as e:
    return JsonResponse(
        {"error": "server error"},
        status=500
    )

# Bad — inconsistent shapes
{"post": {...}}  # sometimes
[{...}]          # other times

# Good — consistent envelope
{"results": [...], "count": 2}
```

</div>
</div>

---

# Putting It All Together — File Layout

A typical app with a REST API:

```
myapp/
├── models.py          # Post, Comment, etc.
├── serializers.py     # post_to_dict() helpers (or DRF serializers)
├── views.py           # HTML views (templates)
├── api_views.py       # JSON API views
├── urls.py
└── templates/
    └── myapp/
        └── index.html

# urls.py
urlpatterns = [
    # HTML views
    path("posts/", views.PostListView.as_view(), name="post-list"),
    # API views
    path("api/posts/", api_views.PostListView.as_view()),
    path("api/posts/<int:pk>/", api_views.PostDetailView.as_view()),
]
```

Keeping `views.py` and `api_views.py` separate makes it easier to evolve the two independently.

---

# Complete `api_views.py` — Reference

```python
import json
from django.http import JsonResponse
from django.views import View
from django.utils.decorators import method_decorator
from django.views.decorators.csrf import csrf_exempt
from .models import Post

def post_to_dict(post):
    return {"id": post.pk, "title": post.title, "body": post.body,
            "author": post.author.username, "created_at": post.created_at.isoformat()}

@method_decorator(csrf_exempt, name="dispatch")
class PostListView(View):
    def get(self, request):
        qs = Post.objects.select_related("author").order_by("-created_at")
        return JsonResponse([post_to_dict(p) for p in qs], safe=False)

    def post(self, request):
        try:
            data = json.loads(request.body)
        except json.JSONDecodeError:
            return JsonResponse({"error": "invalid JSON"}, status=400)
        title = data.get("title", "").strip()
        if not title:
            return JsonResponse({"error": "title required"}, status=400)
        post = Post.objects.create(title=title, body=data.get("body", ""), author=request.user)
        return JsonResponse(post_to_dict(post), status=201)
```

---

# Complete `api_views.py` — Detail View

```python
@method_decorator(csrf_exempt, name="dispatch")
class PostDetailView(View):
    def _get_post(self, pk):
        try:
            return Post.objects.select_related("author").get(pk=pk)
        except Post.DoesNotExist:
            return None

    def get(self, request, pk):
        post = self._get_post(pk)
        if post is None:
            return JsonResponse({"error": "not found"}, status=404)
        return JsonResponse(post_to_dict(post))

    def patch(self, request, pk):
        post = self._get_post(pk)
        if post is None:
            return JsonResponse({"error": "not found"}, status=404)
        try:
            data = json.loads(request.body)
        except json.JSONDecodeError:
            return JsonResponse({"error": "invalid JSON"}, status=400)
        if "title" in data:
            post.title = data["title"].strip() or post.title
        if "body" in data:
            post.body = data["body"]
        post.save()
        return JsonResponse(post_to_dict(post))

    def delete(self, request, pk):
        post = self._get_post(pk)
        if post is None:
            return JsonResponse({"error": "not found"}, status=404)
        post.delete()
        return JsonResponse({}, status=204)
```

---

# Testing the Full Flow with `curl`

Start the dev server, then:

```bash
# 1. List all posts (empty at first)
curl -s http://localhost:8000/api/posts/ | python -m json.tool

# 2. Create a post
curl -s -X POST http://localhost:8000/api/posts/ \
  -H "Content-Type: application/json" \
  -d '{"title": "My First Post", "body": "Hello from curl!"}' \
  | python -m json.tool
# → {"id": 1, "title": "My First Post", ...}

# 3. Read it back
curl -s http://localhost:8000/api/posts/1/ | python -m json.tool

# 4. Update the title
curl -s -X PATCH http://localhost:8000/api/posts/1/ \
  -H "Content-Type: application/json" \
  -d '{"title": "Updated!"}' | python -m json.tool

# 5. Delete it
curl -s -X DELETE http://localhost:8000/api/posts/1/ -w "%{http_code}\n"
# → 204

# 6. Confirm it's gone
curl -s http://localhost:8000/api/posts/1/ | python -m json.tool
# → {"error": "not found"}
```

---

# `isoformat()` and Datetime Serialisation

`datetime` objects are not JSON-serialisable — always convert them to strings.

```python
from datetime import datetime, timezone

dt = datetime.now(timezone.utc)
dt.isoformat()
# '2025-10-14T09:31:05.123456+00:00'
```

ISO 8601 is the correct format for API date/time fields.  
JavaScript can parse it natively: `new Date("2025-10-14T09:31:05+00:00")`.

```python
# In post_to_dict:
"created_at": post.created_at.isoformat(),

# On the JS side:
const date = new Date(post.created_at);
console.log(date.toLocaleDateString("pl-PL"));
// e.g. "14.10.2025"
```

If your field might be `None` (e.g., `deleted_at`):

```python
"deleted_at": post.deleted_at.isoformat() if post.deleted_at else None,
```

---

# Pagination

Large datasets must be paginated — never return all records.

```python
class PostListView(View):
    PAGE_SIZE = 20

    def get(self, request):
        qs = Post.objects.select_related("author").order_by("-created_at")

        try:
            page = max(1, int(request.GET.get("page", 1)))
        except ValueError:
            page = 1

        start = (page - 1) * self.PAGE_SIZE
        end = start + self.PAGE_SIZE
        total = qs.count()

        return JsonResponse({
            "count": total,
            "page": page,
            "page_size": self.PAGE_SIZE,
            "next": page + 1 if end < total else None,
            "results": [post_to_dict(p) for p in qs[start:end]],
        })
```

```
GET /api/posts/?page=2
→ {"count": 87, "page": 2, "page_size": 20, "next": 3, "results": [...]}
```

---

# Ordering and Sorting via Query Params

```python
ALLOWED_ORDER_FIELDS = {"created_at", "title", "views"}

def get(self, request):
    qs = Post.objects.select_related("author")

    order_by = request.GET.get("order_by", "-created_at")
    # Strip leading '-' to check the field name
    field = order_by.lstrip("-")
    if field not in ALLOWED_ORDER_FIELDS:
        return JsonResponse({"error": "invalid order_by"}, status=400)

    qs = qs.order_by(order_by)
    return JsonResponse([post_to_dict(p) for p in qs], safe=False)
```

```bash
# Newest first (default)
curl "/api/posts/?order_by=-created_at"

# Alphabetical by title
curl "/api/posts/?order_by=title"

# Most viewed
curl "/api/posts/?order_by=-views"

# Rejected
curl "/api/posts/?order_by=password"
# → {"error": "invalid order_by"}
```

Always **whitelist** allowed fields — never pass user input directly to `order_by()`.

---

# Error Handling Best Practices

Wrap your view logic to catch unexpected errors gracefully:

```python
import logging
logger = logging.getLogger(__name__)

@method_decorator(csrf_exempt, name="dispatch")
class PostListView(View):
    def get(self, request):
        try:
            qs = Post.objects.select_related("author").order_by("-created_at")
            return JsonResponse([post_to_dict(p) for p in qs], safe=False)
        except Exception as exc:
            logger.exception("Unexpected error in PostListView.get")
            return JsonResponse({"error": "internal server error"}, status=500)
```

**Rules:**
- Never expose raw exception messages to clients — they leak internals
- Always log the real exception with `logger.exception()` (includes traceback)
- Return a generic `{"error": "internal server error"}` to the client
- In development, `DEBUG=True` will still show Django's error page in the browser

Use `DEBUG=False` + Sentry (or similar) in production to capture exceptions.

---

# Summary

REST APIs let Django serve **any client** — browser, mobile, CLI, third-party service.

Key ideas from this lecture:

- **REST** — stateless, resource-based URLs (nouns), HTTP methods as verbs
- **HTTP status codes** matter: 200/201/204 for success, 400/404/422 for client errors, 500 for server errors
- **`JsonResponse`** serialises dicts/lists; use `safe=False` for lists
- **`post_to_dict()`** — explicit field selection, call `.isoformat()` on datetimes
- **`@csrf_exempt` + `method_decorator`** — required for non-browser API clients
- **`json.loads(request.body)`** — always wrap in try/except
- **`select_related()`** — avoid N+1 queries on foreign keys
- **CORS** — the server opts in to cross-origin browser requests via headers
- **`fetch()`** — always check `response.ok`; use `textContent` to prevent XSS
- **Django REST Framework** — `ModelSerializer` + `ModelViewSet` + `DefaultRouter` = full CRUD in ~15 lines
- **OpenAPI / drf-spectacular** — auto-generates machine-readable API spec from your ViewSets
- **Swagger UI** — interactive browser for exploring and testing the API live

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
1. Why do we need `safe=False` when returning a list from `JsonResponse`?
2. What is the difference between `PUT` and `PATCH`?
3. What does `select_related("author")` do and why does it matter?
4. Why does `fetch()` not throw on a 404 response?
5. What does `drf-spectacular` read to generate the OpenAPI schema?

---

# Questions?

**Next lecture:** Django — Forms, ModelForms & Validation

Recommended reading before Lab 7:
- Django docs: `JsonResponse` — docs.djangoproject.com/en/5.2/ref/request-response/#jsonresponse-objects
- MDN: Using the Fetch API — developer.mozilla.org/en-US/docs/Web/API/Fetch_API/Using_Fetch
- MDN: HTTP response status codes — developer.mozilla.org/en-US/docs/Web/HTTP/Status
- `django-cors-headers` — github.com/adamchainz/django-cors-headers

Useful tools:
- **httpie** (`uv tool install httpie`) — friendlier `curl` alternative: `http GET localhost:8000/api/posts/`
- **Bruno** or **Insomnia** — GUI REST clients (open-source alternatives to Postman)
- **DRF Browsable API** — install DRF and visit `/api/` in the browser for a clickable interface

---
{% endraw %}
