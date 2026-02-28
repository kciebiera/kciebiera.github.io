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

**Exploration:** Add a `?search=` query parameter to `PostListView.get()`:

```python
query = request.GET.get("search", "")
if query:
    posts = posts.filter(title__icontains=query)
```

Test: `curl "http://127.0.0.1:8000/api/posts/?search=django"`


{% endraw %}
