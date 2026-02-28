---
render_with_liquid: false
---

{% raw %}

# Lab 7: Django REST API — Serving JSON

## Introduction

So far your Django app returns HTML pages — full documents for human browsers. But modern frontends (and mobile apps) want raw data that they render themselves. A **REST API** serves data as JSON over the same HTTP protocol you learned in Lab 1.

The Goal: Build a JSON API for the blog. Implement endpoints following REST conventions, handle authentication in the API, and consume the API from a plain browser `fetch()` call.

### The Theory

A **REST API** maps HTTP methods to operations on resources:

| Method   | Path              | Meaning                    |
|----------|-------------------|----------------------------|
| `GET`    | `/api/posts/`     | List all posts             |
| `GET`    | `/api/posts/1/`   | Get one post by ID         |
| `POST`   | `/api/posts/`     | Create a new post          |
| `PUT`    | `/api/posts/1/`   | Replace a post             |
| `PATCH`  | `/api/posts/1/`   | Partially update a post    |
| `DELETE` | `/api/posts/1/`   | Delete a post              |

The server responds with **JSON** and appropriate HTTP status codes: `200 OK`, `201 Created`, `400 Bad Request`, `401 Unauthorized`, `404 Not Found`.

## Setup

No new packages required — Django's `JsonResponse` and `View` are sufficient. You will *not* use Django REST Framework in this lab; building it by hand first makes the abstractions meaningful.

Create `blog/api.py` for all API views.

## Phase 1: The First Endpoint

```python
# blog/api.py
import json
from django.http import JsonResponse
from django.views import View
from django.views.decorators.csrf import csrf_exempt
from django.utils.decorators import method_decorator
from .models import Post, Comment

def post_to_dict(post):
    return {
        "id":       post.id,
        "title":    post.title,
        "slug":     post.slug,
        "body":     post.body,
        "pub_date": post.pub_date.isoformat(),
        "category": post.category.name if post.category else None,
    }

@method_decorator(csrf_exempt, name="dispatch")
class PostListView(View):

    def get(self, request):
        posts = Post.objects.all()
        return JsonResponse({"posts": [post_to_dict(p) for p in posts]})

    def post(self, request):
        if not request.user.is_authenticated:
            return JsonResponse({"error": "Authentication required."}, status=401)

        try:
            data = json.loads(request.body)
        except json.JSONDecodeError:
            return JsonResponse({"error": "Invalid JSON."}, status=400)

        # TODO: Validate that data contains "title", "slug", and "body".
        # Return status=400 with {"error": "..."} if any are missing.

        # TODO: Create and save a Post. Return status=201 with post_to_dict(post).
        pass
```

Create `blog/api_urls.py`:

```python
from django.urls import path
from . import api

urlpatterns = [
    path("posts/",      api.PostListView.as_view(), name="api-post-list"),
    # TODO: add path("posts/<int:pk>/", api.PostDetailView.as_view(), ...)
]
```

Include in `mysite/urls.py`:

```python
path("api/", include("blog.api_urls")),
```

🧪 In your terminal:

```bash
curl http://127.0.0.1:8000/api/posts/
```

You should receive a JSON array of your posts.

## Phase 2: Detail, Update, Delete

```python
@method_decorator(csrf_exempt, name="dispatch")
class PostDetailView(View):

    def get_post(self, pk):
        try:
            return Post.objects.get(pk=pk)
        except Post.DoesNotExist:
            return None

    def get(self, request, pk):
        post = self.get_post(pk)
        if post is None:
            return JsonResponse({"error": "Not found."}, status=404)
        return JsonResponse(post_to_dict(post))

    def patch(self, request, pk):
        if not request.user.is_authenticated:
            return JsonResponse({"error": "Authentication required."}, status=401)
        post = self.get_post(pk)
        if post is None:
            return JsonResponse({"error": "Not found."}, status=404)

        try:
            data = json.loads(request.body)
        except json.JSONDecodeError:
            return JsonResponse({"error": "Invalid JSON."}, status=400)

        # TODO: Update only the fields present in data
        # Allowed fields: title, body, slug
        # Save and return updated post_to_dict(post)
        pass

    def delete(self, request, pk):
        # TODO: Return 401 if not authenticated.
        # TODO: Delete the post and return JsonResponse({}, status=204).
        pass
```

🧪 Test with curl:

```bash
# Get a single post
curl http://127.0.0.1:8000/api/posts/1/

# Patch (requires session — use the browser console instead)
curl -X PATCH http://127.0.0.1:8000/api/posts/1/ \
     -H "Content-Type: application/json" \
     -d '{"title": "Updated Title"}'
```

## Phase 3: Comments Endpoint

Add a nested comments list endpoint:

```python
@method_decorator(csrf_exempt, name="dispatch")
class PostCommentView(View):
    """GET/POST comments for a given post."""

    def get(self, request, pk):
        post = Post.objects.filter(pk=pk).first()
        if post is None:
            return JsonResponse({"error": "Not found."}, status=404)
        comments = post.comments.filter(active=True)
        data = [
            {
                "id":      c.id,
                "author":  c.author,
                "body":    c.body,
                "created": c.created.isoformat(),
            }
            for c in comments
        ]
        return JsonResponse({"comments": data})

    def post(self, request, pk):
        # TODO: Parse JSON body, validate author+body fields,
        # create Comment(post=post, ..., active=True), return status=201.
        pass
```

Add `path("posts/<int:pk>/comments/", api.PostCommentView.as_view())` to your API URLs.

## Phase 4: Consuming the API from JavaScript

This brings together what you know from Labs 2–3. Add a new Django view that returns a plain HTML page with an embedded `<script>` block:

```python
# pages/views.py
def api_demo(request):
    return render(request, "pages/api_demo.html")
```

`pages/templates/pages/api_demo.html`:

```html
{% extends "pages/base.html" %}
{% block title %}API Demo{% endblock %}

{% block content %}
<h1>Live Posts</h1>
<div id="post-list">Loading...</div>

<script>
async function loadPosts() {
    const response = await fetch("/api/posts/");
    const data = await response.json();

    const container = document.getElementById("post-list");
    container.innerHTML = "";

    for (const post of data.posts) {
        const article = document.createElement("article");
        // TODO: Set article.innerHTML to show post.title and
        //       the first 100 characters of post.body
        container.appendChild(article);
    }
}

loadPosts();
</script>
{% endblock %}
```

🧪 Visit the page — posts should load without a full page reload. Open DevTools → Network → Fetch/XHR to see the API call.

## Submission

Final checks:

1. `GET /api/posts/` returns JSON list.
2. `GET /api/posts/<id>/` returns one post or `404`.
3. `POST /api/posts/<id>/comments/` creates a comment and returns `201`.
4. The API demo page loads posts dynamically via `fetch()`.
5. `GET /drf/posts/` works via DRF and returns the same data.
6. `http://127.0.0.1:8000/api/swagger/` opens Swagger UI and shows all endpoints.

**Exploration:** Add a `?search=` query parameter to `PostListView.get()`:

```python
query = request.GET.get("search", "")
if query:
    posts = posts.filter(title__icontains=query)
```

Test: `curl "http://127.0.0.1:8000/api/posts/?search=django"`

## Phase 5: Django REST Framework, OpenAPI & Swagger

So far you have built the API by hand. **Django REST Framework (DRF)** is the de-facto standard library that handles serialisation, validation, authentication, and routing for you — in a fraction of the code.

### Setup

```bash
uv add djangorestframework drf-spectacular
```

Add both to `INSTALLED_APPS` in `settings.py`:

```python
INSTALLED_APPS = [
    ...
    "rest_framework",
    "drf_spectacular",
]
```

Configure DRF and point it at `drf-spectacular` for schema generation:

```python
REST_FRAMEWORK = {
    "DEFAULT_SCHEMA_CLASS": "drf_spectacular.openapi.AutoSchema",
    "DEFAULT_AUTHENTICATION_CLASSES": [
        "rest_framework.authentication.SessionAuthentication",
    ],
    "DEFAULT_PERMISSION_CLASSES": [
        "rest_framework.permissions.IsAuthenticatedOrReadOnly",
    ],
}
```

### Serializer

A **serializer** replaces your hand-written `post_to_dict()` helper. It also handles validation on write. Create `blog/serializers.py`:

```python
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
        fields = ["id", "title", "slug", "body", "pub_date", "category", "comments"]
```

### ViewSet

A **ViewSet** replaces your four separate class-based views. Create `blog/drf_views.py`:

```python
from rest_framework import viewsets, permissions
from .models import Post
from .serializers import PostSerializer

class PostViewSet(viewsets.ModelViewSet):
    queryset           = Post.objects.all()
    serializer_class   = PostSerializer
    permission_classes = [permissions.IsAuthenticatedOrReadOnly]
```

`ModelViewSet` automatically provides `list`, `create`, `retrieve`, `update`, `partial_update`, and `destroy` — all five REST operations.

### Router

A **Router** auto-generates URLs for the ViewSet. Create `blog/drf_urls.py`:

```python
from rest_framework.routers import DefaultRouter
from .drf_views import PostViewSet

router = DefaultRouter()
router.register("posts", PostViewSet)

urlpatterns = router.urls
```

Include it in `mysite/urls.py`:

```python
path("drf/", include("blog.drf_urls")),
```

🧪 Visit `http://127.0.0.1:8000/drf/` in the browser — DRF renders a **browsable API**: a clickable HTML interface where you can make GET/POST requests without curl.

### OpenAPI Schema

`drf-spectacular` introspects your ViewSets and generates a standard **OpenAPI 3.0** YAML schema automatically. Add the schema and UI endpoints to `mysite/urls.py`:

```python
from drf_spectacular.views import SpectacularAPIView, SpectacularSwaggerView

urlpatterns += [
    path("api/schema/",  SpectacularAPIView.as_view(),      name="schema"),
    path("api/swagger/", SpectacularSwaggerView.as_view(url_name="schema"),
                                                             name="swagger-ui"),
]
```

🧪 Visit:
- `http://127.0.0.1:8000/api/schema/` — downloads the raw `openapi.yaml` file.
- `http://127.0.0.1:8000/api/swagger/` — opens **Swagger UI**: an interactive browser where you can read docs, inspect request/response shapes, and execute real API calls.

You can also generate the schema as a file on disk:

```bash
uv run python manage.py spectacular --file schema.yaml
```

Open `schema.yaml` — it describes every endpoint, method, parameter, and response model in a machine-readable format that other tools (code generators, API clients, testing suites) can consume.

### Comparing hand-rolled vs DRF

| | Hand-rolled (Phases 1–3) | DRF (Phase 5) |
|---|---|---|
| Serialisation | `post_to_dict()` by hand | `ModelSerializer` |
| Validation | `if "title" not in data` | `.is_valid()` + field types |
| 5 CRUD endpoints | ~80 lines | `ModelViewSet` (~5 lines) |
| Browsable UI | No | Built-in |
| OpenAPI docs | No | `drf-spectacular` auto-generates |

> **Why learn both?** Writing the raw version first gives you intuition for what DRF does under the hood. In practice, you would use DRF from day one.




{% endraw %}
