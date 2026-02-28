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

Add a `projects` view. In `pages/views.py`:

```python
def projects(request):
    project_list = [
        {"name": "Socket Server",  "lang": "Python", "year": 2025, "done": True},
        {"name": "HTML Profile",   "lang": "HTML",   "year": 2025, "done": True},
        {"name": "CSS Layout",     "lang": "CSS",    "year": 2025, "done": True},
        {"name": "Django App",     "lang": "Python", "year": 2025, "done": False},
    ]
    context = {
        # TODO: pass project_list and a count of done projects
    }
    return render(request, "pages/projects.html", context)
```

Create `pages/templates/pages/projects.html` extending the base:

```html
{% extends "pages/base.html" %}
{% block title %}Projects{% endblock %}

{% block content %}
<h1>Projects ({{ done_count }} complete)</h1>

<table>
    <thead>
        <tr><th>Name</th><th>Language</th><th>Year</th><th>Status</th></tr>
    </thead>
    <tbody>
        {% for project in project_list %}
        <tr>
            <td>{{ project.name }}</td>
            <td>{{ project.lang|lower }}</td>
            <td>{{ project.year }}</td>
            <td>{% if project.done %}✅ Done{% else %}🔄 In progress{% endif %}</td>
        </tr>
        {% empty %}
        <tr><td colspan="4">No projects yet.</td></tr>
        {% endfor %}
    </tbody>
</table>
{% endblock %}
```

The `|lower` is a **filter** — it transforms the value before displaying it. Try a few more in your template: `|upper`, `|length`, `{{ project.year|add:1 }}`.

Now wire up static files. In `mysite/settings.py`, confirm:

```python
STATIC_URL = "/static/"
```

Create `pages/static/pages/style.css` and paste the CSS from your Lab 3 stylesheet. The `{% static %}` tag in `base.html` generates the correct URL automatically.

🧪 Verify in DevTools → Network that `style.css` loads with status 200.

## Submission

Final checks:

1. Routes `/`, `/about/`, `/greet/<name>/`, and `/projects/` all work.
2. Every page extends `base.html` — zero duplicated `<html>/<head>/<body>` tags in child templates.
3. The `projects` page uses `{% for %}`, `{% if %}`, `{% empty %}`, and at least two filters.
4. CSS from Lab 3 is served as a static file.

**Exploration:** Use `{% include %}` to extract the project table row into a partial file `pages/templates/pages/_project_row.html` and include it from the loop with `{% include "pages/_project_row.html" %}`. The page should render identically.


{% endraw %}
