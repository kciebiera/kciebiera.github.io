---
render_with_liquid: false
---

{% raw %}

# Lab 7: Django REST API — Serving JSON

## Introduction

So far your Django app returns HTML pages. A **REST API** returns data as JSON so browsers, frontend apps, mobile apps, and scripts can render or process it themselves.

**Goal:** Build a JSON API for the blog, handle API authentication rules, consume the API with `fetch()`, and compare a hand-written API with Django REST Framework.

### REST Basics

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

Continue in the same Django project from Labs 4-6. This lab uses the `Post` and `Comment` models from Lab 5 and the authentication setup from Lab 6.

Create `blog/api.py` for all API views.

Expected structure by the end of this lab:

```text
blog/
    api.py
    api_urls.py
    drf_views.py
    drf_urls.py
    serializers.py
pages/
    templates/
        pages/
            api_demo.html
```

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

**Check:** In your terminal:

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

**Check:** Test public reads with curl:

```bash
# Get a single post
curl http://127.0.0.1:8000/api/posts/1/

```

To test authenticated `PATCH` and `DELETE`, log in through the browser first, then use DevTools -> Console:

```javascript
await fetch("/api/posts/1/", {
  method: "PATCH",
  headers: {"Content-Type": "application/json"},
  body: JSON.stringify({title: "Updated Title"})
}).then(r => r.json())
```

For `DELETE`, use:

```javascript
await fetch("/api/posts/1/", {method: "DELETE"}).then(r => r.status)
```

Only run `DELETE` on a test post you can afford to remove.

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

**Check:** Visit the page. Posts should load without a full page reload. Open DevTools -> Network -> Fetch/XHR to see the API call.

## Phase 5: Search Query Parameter

Add a `?search=` query parameter to `PostListView.get()`:

```python
query = request.GET.get("search", "")
if query:
    posts = posts.filter(title__icontains=query)
```

Test: `curl "http://127.0.0.1:8000/api/posts/?search=django"`

**Check:** Searching for a word in a post title should return a smaller list. No `search` parameter should still return all posts.

## Phase 6: Django REST Framework, OpenAPI, and Swagger

So far you have built the API by hand. **Django REST Framework (DRF)** handles serialization, validation, authentication, and routing with less repeated code.

### Install DRF

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

`ModelViewSet` automatically provides `list`, `create`, `retrieve`, `update`, `partial_update`, and `destroy`.

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

**Check:** Visit `http://127.0.0.1:8000/drf/` in the browser. DRF renders a browsable API where you can inspect and test endpoints.

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

**Check:** Visit:
- `http://127.0.0.1:8000/api/schema/` to download the raw OpenAPI schema.
- `http://127.0.0.1:8000/api/swagger/` to open Swagger UI.

You can also generate the schema as a file on disk:

```bash
uv run python manage.py spectacular --file schema.yaml
```

Open `schema.yaml`. It describes every endpoint, method, parameter, and response model in a machine-readable format.

### Compare Hand-Written API and DRF

| | Hand-written API (Phases 1-5) | DRF (Phase 6) |
|---|---|---|
| Serialization | `post_to_dict()` by hand | `ModelSerializer` |
| Validation | `if "title" not in data` | `.is_valid()` + field types |
| 5 CRUD endpoints | ~80 lines | `ModelViewSet` (~5 lines) |
| Browsable UI | No | Built-in |
| OpenAPI docs | No | `drf-spectacular` auto-generates |

The hand-written version shows the HTTP and JSON mechanics. DRF removes repeated serialization, validation, routing, and documentation code.

## Submission

Submit the complete Django project from Labs 4-7.

Your project must include:

1. A Django project with `manage.py` and the `mysite/` configuration package.
2. A `pages` app.
3. A `blog` app.
4. An `accounts` app.
5. Templates for pages, blog posts, account views, and API demo page.
6. Static CSS served through Django's static files system.
7. Database migrations for blog models.

Lab 4 requirements:

1. Routes `/`, `/about/`, `/greet/<name>/`, `/projects/`, and `/blog/` work.
2. Pages use `pages/base.html` through template inheritance.
3. The projects page supports `?q=` search.
4. Static CSS loads with status 200.
5. The `blog` app has its own `blog/urls.py` with `app_name = "blog"`.
6. Blog links use the `blog:` namespace.
7. `uv run python manage.py test pages` passes.

Lab 5 requirements:

1. `Category`, `Post`, and `Comment` models exist and have migrations.
2. The admin can create categories, posts, and comments.
3. `/blog/` lists posts and category filter links.
4. `/blog/post/<slug>/` shows a post and its active comments.
5. The comment form uses `POST`, `{% csrf_token %}`, validation, save, and redirect.
6. Invalid comments re-render the form with inline errors.
7. The post list query uses `select_related("category")`.
8. The `Post` model has `view_count`, and the detail view increments it with `F()`.

Lab 6 requirements:

1. Users can register, log in, log out, and view `/accounts/profile/`.
2. The nav changes based on `user.is_authenticated`.
3. `/accounts/profile/` requires login.
4. Staff users can create posts at `/blog/new/`.
5. Regular users cannot create posts.
6. GitHub OAuth login works through django-allauth, or the project clearly documents missing local OAuth credentials.
7. Secrets such as GitHub client secret are not committed.

Lab 7 requirements:

1. `GET /api/posts/` returns a JSON list.
2. `GET /api/posts/<id>/` returns one post or `404`.
3. `PATCH /api/posts/<id>/` updates allowed fields for authenticated users.
4. `DELETE /api/posts/<id>/` deletes a post for authenticated users.
5. `GET /api/posts/?search=django` filters posts by title.
6. `GET` and `POST /api/posts/<id>/comments/` work.
7. The API demo page loads posts with `fetch()`.
8. `GET /drf/posts/` works through DRF.
9. `http://127.0.0.1:8000/api/schema/` returns an OpenAPI schema.
10. `http://127.0.0.1:8000/api/swagger/` opens Swagger UI.

Before submitting, run:

```bash
uv run python manage.py check
uv run python manage.py test
uv run python manage.py runserver
```

Then manually test the main HTML pages, account flow, blog comment flow, API endpoints, DRF endpoint, and Swagger page.

Common errors:

1. `401 Unauthorized` on write endpoints usually means you are not logged in.
2. `JSONDecodeError` or `400 Invalid JSON` usually means the request body is not valid JSON.
3. `TypeError: Object of type ... is not JSON serializable` means `post_to_dict()` is returning a model object or datetime instead of plain JSON-compatible values.
4. `404` on `/api/posts/` usually means `blog.api_urls` was not included under `path("api/", ...)`.
5. Swagger not loading usually means `drf_spectacular` is missing from `INSTALLED_APPS` or the schema URLs were not added.




{% endraw %}
