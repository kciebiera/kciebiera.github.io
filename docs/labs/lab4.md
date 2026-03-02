---
render_with_liquid: false
---

{% raw %}

# Lab 4: Django — Views, URLs & Templates

## Introduction

In Labs 1–3 you built a server, structured content, and added style — all by hand. You have experienced exactly the pain that web frameworks exist to solve: routing, templating, static file serving, and request parsing were all written from scratch.

Django is a "batteries-included" Python web framework. In this lab you will recreate what you built in Labs 1–3 in a fraction of the code, and then learn how Django separates layout from content using **template inheritance** — so a change to the `<nav>` updates every page at once.

The Goal: A running Django project with multiple URL routes, views rendered from templates, and a shared base layout.

### The Theory

Django follows the **MVT** pattern (Model–View–Template):

- **Model**: the data layer (database, next lab).
- **View**: a Python function that receives an `HttpRequest` and returns an `HttpResponse`.
- **Template**: an HTML file with placeholders that a view fills in.

Template inheritance adds one more idea:

- `{% block name %}...{% endblock %}` — a replaceable region in a base template.
- `{% extends "base.html" %}` — a child template that inherits the parent's structure and overrides specific blocks.

## Setup

```bash
uv add django
django-admin startproject mysite .
uv run python manage.py runserver
```

Open `http://127.0.0.1:8000` — you should see the Django rocket launch page.

Key files created:

```
mysite/
    settings.py   ← project configuration
    urls.py       ← root URL table
manage.py         ← project CLI
```

Create your first app:

```bash
uv run python manage.py startapp pages
```

Register it in `mysite/settings.py`:

```python
INSTALLED_APPS = [
    ...
    'pages',   # TODO: add this line
]
```

## Phase 1: Your First View

Open `pages/views.py`. Django views are just functions that take a request and return a response.

```python
from django.http import HttpResponse

def home(request):
    # TODO: Return an HttpResponse with "<h1>Hello from Django!</h1>"
    pass
```

Wire it to a URL. Create `pages/urls.py`:

```python
from django.urls import path
from . import views

urlpatterns = [
    # TODO: Add a path for "" (empty string = root) that calls views.home
    # path("", views.home, name="home"),
]
```

Include `pages.urls` in `mysite/urls.py`:

```python
from django.urls import path, include

urlpatterns = [
    path("admin/", admin.site.urls),
    # TODO: Add: path("", include("pages.urls"))
]
```

🧪 Visit `http://127.0.0.1:8000/` — you should see your heading.

## Phase 2: Template Context & URL Parameters

Hardcoding HTML in a view is no better than your socket server. Templates separate logic from presentation.

Create the directory `pages/templates/pages/`. Inside it, create `home.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>{{ page_title }}</title>
</head>
<body>
    <h1>{{ heading }}</h1>
    <p>The server time is: {{ server_time }}</p>
</body>
</html>
```

`{{ variable }}` is a Django template placeholder. Update your view:

```python
from django.shortcuts import render
import datetime

def home(request):
    context = {
        # TODO: Add "page_title", "heading", and "server_time" keys
        # server_time should be datetime.datetime.now()
    }
    return render(request, "pages/home.html", context)
```

Now add two more views that demonstrate URL parameters and template loops:

```python
def about(request):
    # TODO: render pages/about.html with a context containing a list of skills
    # skills = ["Python", "HTTP", "HTML", "CSS"]
    pass

def greet(request, name):
    # TODO: render pages/greet.html passing the name
    pass
```

Add the routes in `pages/urls.py`:

```python
urlpatterns = [
    path("", views.home, name="home"),
    # TODO: path for "about/"
    # TODO: path for "greet/<str:name>/" that captures the name
]
```

Create `pages/templates/pages/about.html` using a template loop:

```html
<ul>
    {% for skill in skills %}
        <li>{{ skill }}</li>
    {% endfor %}
</ul>
```

Create `pages/templates/pages/greet.html`:

```html
<h1>Hello, {{ name }}!</h1>
<!-- TODO: Add a link back to home using {% url 'home' %} -->
```

🧪 Visit `/greet/Alice/` and `/greet/Bob/` — each should show a personalised greeting. Reload the home page — the time should update on every refresh.

## Phase 3: Template Inheritance

Every page currently duplicates `<html>`, `<head>`, and any navigation. Template inheritance solves this with a single base layout.

Create `pages/templates/pages/base.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    {% load static %}
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{% block title %}My Site{% endblock %} — MyApp</title>
    <link rel="stylesheet" href="{% static 'pages/style.css' %}">
    {% block extra_head %}{% endblock %}
</head>
<body>
    <header>
        <nav>
            <a href="{% url 'home' %}">Home</a>
            <a href="{% url 'about' %}">About</a>
            <a href="{% url 'projects' %}">Projects</a>
        </nav>
    </header>

    <main>
        {% block content %}{% endblock %}
    </main>

    <footer>
        <p>{% block footer_text %}© 2025 My Site{% endblock %}</p>
    </footer>
</body>
</html>
```

Refactor `home.html`, `about.html`, and `greet.html` to extend the base:

```html
{% extends "pages/base.html" %}

{% block title %}Home{% endblock %}

{% block content %}
    <h1>{{ heading }}</h1>
    <p>Server time: {{ server_time }}</p>
{% endblock %}
```

🧪 Reload each page. The nav and footer should now appear without being in the child templates. Open DevTools → Elements and confirm the full DOM is present.

## Phase 4: Template Tags, Filters & Static Files

Add a `projects` view. Use the expanded dataset below and implement search. In `pages/views.py`:

```python
ALL_PROJECTS = [
    {"name": "Socket Server",    "lang": "Python",     "year": 2025, "done": True},
    {"name": "HTML Profile",     "lang": "HTML",       "year": 2025, "done": True},
    {"name": "CSS Layout",       "lang": "CSS",        "year": 2025, "done": True},
    {"name": "Django App",       "lang": "Python",     "year": 2025, "done": False},
    {"name": "REST API",         "lang": "Python",     "year": 2024, "done": True},
    {"name": "React Dashboard",  "lang": "JavaScript", "year": 2024, "done": True},
    {"name": "SQL Queries Lab",  "lang": "SQL",        "year": 2024, "done": True},
    {"name": "CLI Tool",         "lang": "Python",     "year": 2023, "done": True},
]

def projects(request):
    q = request.GET.get("q", "")
    # TODO: if q is non-empty, filter ALL_PROJECTS so only entries whose name
    #       or lang contains q (case-insensitive) are kept; otherwise show all
    project_list = ALL_PROJECTS  # replace this line
    context = {
        # TODO: pass project_list, done_count, and q
    }
    return render(request, "pages/projects.html", context)
```

**Your task:** Create `pages/templates/pages/projects.html` yourself. Requirements:

- Extend `pages/base.html`.
- Show a heading with the number of completed projects.
- Use `{% for %}` with `{% empty %}` to render the project list.
- Use `{% if %}` to display ✅ or 🔄 per project.
- Apply at least two template filters (e.g. `|lower`, `|upper`, `|length`, `{{ project.year|add:1 }}`).
- Add a search `<form method="GET">` with a text input named `q`. Pre-fill it with the current `q` value from context so the search term persists after submission.
- If `q` is non-empty, show a line like *Showing results for "python"*.

📖 Refer to [Template tags and filters](https://docs.djangoproject.com/en/stable/ref/templates/builtins/) in the Django docs.

Now wire up static files. In `mysite/settings.py`, confirm:

```python
STATIC_URL = "/static/"
```

Create `pages/static/pages/style.css` and paste the CSS from your Lab 3 stylesheet. The `{% static %}` tag in `base.html` generates the correct URL automatically.

🧪 Verify in DevTools → Network that `style.css` loads with status 200.

## Phase 5: A Second App & Multi-App Routing

A Django project can host many apps, each owning its own models, views, URLs, and templates. Right now your project has one app (`pages`). In this phase you will add a second app — `blog` — and see how the root URL router delegates to each.

### 5.1 Create the `blog` App

```bash
uv run python manage.py startapp blog
```

Register it in `mysite/settings.py`:

```python
INSTALLED_APPS = [
    ...
    'pages',
    'blog',   # ← add this
]
```

### 5.2–5.4 Implement the Blog App

Your task is to build the blog app **without copy-pasting** — figure out the details from the Django documentation.

**Requirements:**

1. Create `blog/urls.py` with `app_name = "blog"` and two URL patterns:
   - `""` → a list view named `"post-list"`
   - `"<int:pk>/"` → a detail view named `"post-detail"`

2. Wire the blog app into `mysite/urls.py` under the `blog/` prefix. Everything under `/blog/` should be owned by `blog/urls.py`.

3. In `blog/views.py`, use this in-memory dataset:
   ```python
   POSTS = [
       {"pk": 1, "title": "Hello Django",    "body": "My first post."},
       {"pk": 2, "title": "URL Routing",     "body": "How include() works."},
       {"pk": 3, "title": "Template Tricks", "body": "Extends and blocks."},
   ]
   ```
   Write `post_list(request)` and `post_detail(request, pk)`. The detail view must return a `404` response when the `pk` is not found.

4. Create the template directory structure:
   ```
   blog/templates/blog/post_list.html
   blog/templates/blog/post_detail.html
   ```
   Both templates must extend `pages/base.html`. The list template must link each post to its detail page using `{% url 'blog:post-detail' pk=post.pk %}`. Use the `blog:` namespace prefix in every `{% url %}` tag — without it Django raises `NoReverseMatch` when two apps share a pattern name.

📖 Refer to:
- [URL dispatcher](https://docs.djangoproject.com/en/stable/topics/http/urls/)
- [URL namespaces](https://docs.djangoproject.com/en/stable/topics/http/urls/#url-namespaces)

### 5.5 Add a Blog Link to the Nav

In `pages/templates/pages/base.html`, add a link to the blog:

```html
<nav>
    <a href="{% url 'home' %}">Home</a>
    <a href="{% url 'about' %}">About</a>
    <a href="{% url 'blog:post-list' %}">Blog</a>
</nav>
```

🧪 Visit `http://127.0.0.1:8000/blog/` — you should see the post list. Click a title to go to the detail page. The `<nav>` should work on both the `pages` and `blog` templates since they both extend the same `base.html`.

### 5.6 Confirm Namespace Isolation

Open `uv run python manage.py shell` and verify that both namespaces resolve correctly without colliding:

```python
from django.urls import reverse

reverse("home")                           # → "/"
reverse("blog:post-list")                 # → "/blog/"
reverse("blog:post-detail", kwargs={"pk": 2})  # → "/blog/2/"
```

Try `reverse("post-list")` (no namespace) — it will raise `NoReverseMatch` because the name is only registered under the `blog` namespace.

## Phase 6: POST Requests & Forms

Views can handle both GET and POST. Before models are introduced, `request.POST` and in-memory data are enough to practise the full HTTP request cycle.

### 6.1 Add a Note Form to the Greet Page

Modify the `greet` view in `pages/views.py`:

```python
def greet(request, name):
    message = ""
    if request.method == "POST":
        # TODO: read request.POST.get("note") and build a message string
        # e.g. 'Thanks, Alice! Your note: "great lab"'
        pass
    return render(request, "pages/greet.html", {"name": name, "message": message})
```

In `pages/templates/pages/greet.html`, add the form inside `{% block content %}`:

```html
{% if message %}<p>{{ message }}</p>{% endif %}

<form method="POST">
    {% csrf_token %}
    <label for="note">Leave a note:</label>
    <input type="text" id="note" name="note" required>
    <button type="submit">Send</button>
</form>
```

`{% csrf_token %}` inserts a hidden token Django verifies on every POST. Without it, Django raises a `403 Forbidden` — this is protection against **Cross-Site Request Forgery** attacks, where a malicious site could otherwise submit a form on behalf of an authenticated user.

🧪 Visit `/greet/Alice/`, submit the form, and confirm the message appears. Then temporarily remove `{% csrf_token %}` and observe the 403.

### 6.2 Challenge: In-Memory Guestbook

**Your task (no code provided):** Add a route `/guestbook/` that:

1. Displays all past entries stored in a module-level list (e.g. `ENTRIES = []`).
2. Shows a `<form method="POST">` with `name` and `message` fields.
3. On POST: validates both fields are non-empty, appends `{"name": …, "message": …}` to `ENTRIES`, then **redirects** back to `/guestbook/` using `HttpResponseRedirect` (this is the **Post/Redirect/Get** pattern — look it up).

> **Hint:** `from django.http import HttpResponseRedirect`; `from django.urls import reverse`

## Phase 7: Automated Testing

Django ships with a test client that issues HTTP requests without a real server. Writing tests alongside code is a core engineering discipline.

Open `pages/tests.py`:

```python
from django.test import TestCase

class PagesTests(TestCase):

    def test_home_status(self):
        response = self.client.get("/")
        # TODO: assert the status code is 200

    def test_about_uses_correct_template(self):
        response = self.client.get("/about/")
        # TODO: assert "pages/about.html" was used (hint: assertTemplateUsed)

    def test_about_has_four_skills(self):
        response = self.client.get("/about/")
        # TODO: assert len(response.context["skills"]) == 4

    def test_greet_contains_name(self):
        response = self.client.get("/greet/Alice/")
        # TODO: assert b"Alice" in response.content

    def test_projects_search_filters(self):
        response = self.client.get("/projects/?q=python")
        # TODO: assert every project in response.context["project_list"]
        #       has lang == "Python" or name containing "python" (case-insensitive)
```

Run your tests:

```bash
uv run python manage.py test pages
```

All tests should pass. If one fails, fix the view or template — never weaken a test to make it green.

🧪 Intentionally remove a skill from the `about` view's list. Re-run the tests and observe the failure message. Then restore it and confirm all tests pass again.

📖 Refer to: [Writing and running tests](https://docs.djangoproject.com/en/stable/topics/testing/overview/)

## Submission

Final checks:

1. Routes `/`, `/about/`, `/greet/<name>/`, and `/projects/` all work.
2. Every page extends `base.html` — zero duplicated `<html>/<head>/<body>` tags in child templates.
3. The `projects` page uses `{% for %}`, `{% if %}`, `{% empty %}`, and at least two filters.
4. Visiting `/projects/?q=python` returns only Python-related projects; visiting `/projects/` with no `q` returns all projects.
5. CSS from Lab 3 is served as a static file.
6. `/blog/` lists all posts and `/blog/<pk>/` shows a single post.
7. `{% url %}` tags use the `blog:` namespace prefix — no hard-coded URLs anywhere.
8. Both apps coexist in `INSTALLED_APPS` and `mysite/urls.py`.
9. The `greet` page handles POST and shows the submitted note; `{% csrf_token %}` is present.
10. `uv run python manage.py test pages` passes all five tests.

**Exploration:** Use `{% include %}` to extract the project table row into a partial file `pages/templates/pages/_project_row.html` and include it from the loop with `{% include "pages/_project_row.html" %}`. The page should render identically.

**Session State (stretch):** Django sessions let you persist small amounts of per-user data server-side across requests without a database. In any view, read and write `request.session["key"]`:

```python
count = request.session.get("visit_count", 0) + 1
request.session["visit_count"] = count
```

**Your task:** Add a visit counter to the home page so the heading reads *"Welcome! You have visited this page N time(s) this session."* Navigating away and back should keep incrementing the count; opening an Incognito window should start at 1.

📖 Refer to: [How to use sessions](https://docs.djangoproject.com/en/stable/topics/http/sessions/)


{% endraw %}
