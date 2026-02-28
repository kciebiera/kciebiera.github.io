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

# Lecture 4
## Django — Views, URLs & Templates

**WWW 25/26**
MVT · URL Routing · Template Inheritance · Static Files

---

# What is a Web Framework?

In Lab 1 you wrote a raw HTTP server using Python's `socket` module.

```python
# Lab 1 — the pain
server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(("", 8080))
server.listen(1)
conn, _ = server.accept()
raw = conn.recv(4096).decode()
# manually parse "GET /path HTTP/1.1\r\nHost: ..."
# manually build "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n\r\n..."
```

Every project would have to solve the same problems from scratch:
- Parsing request headers, query strings, cookies
- Routing URLs to the right handler
- Escaping output to prevent XSS
- Managing sessions, CSRF, file uploads …

A **web framework** is a reusable solution to all of these, so you can focus on *your* application logic.

---

# What a Framework Gives You

| You solved in Lab 1 | Django does it for you |
|---|---|
| Parse raw HTTP bytes | `request.method`, `request.GET` |
| Build response string | `HttpResponse`, `render()` |
| Map URL strings to code | `urls.py` + `path()` |
| Read POST body | `request.POST` |
| Escape HTML output | automatic in templates |
| Session cookies | `request.session` |
| CSRF protection | `{% csrf_token %}` middleware |

**Less boilerplate → fewer bugs → faster development.**

---

# Django's Place in the Ecosystem

Django is a **batteries-included** web framework for Python.

<div class="columns">
<div>

**Django**
- URL routing ✓
- ORM (database) ✓
- Templating engine ✓
- Forms & validation ✓
- Authentication ✓
- Admin interface ✓
- One opinionated way

</div>
<div>

**Flask / FastAPI**
- URL routing ✓
- ORM: choose your own
- Templates: choose your own
- Forms: choose your own
- Auth: choose your own
- Admin: choose your own
- Many flexible ways

</div>
</div>

**Rule of thumb:** Django for full-featured web apps; Flask/FastAPI for small APIs or when you want full control of the stack.

---

# The MVT Pattern

Django uses **Model–View–Template** — a close cousin of the classic MVC pattern.

```
Browser ──► urls.py ──► View (views.py)
                            │
                     ┌──────┴──────┐
                     ▼             ▼
                  Model         Template
               (database)       (.html)
                     │             │
                     └──────┬──────┘
                            ▼
                        Response
                      (HTML page)
```

| MVT | MVC equivalent |
|---|---|
| Model | Model |
| View | Controller |
| Template | View |

Django's "View" is the *controller* — it decides what data to fetch and which template to render.

---

# Installing Django with uv

```bash
# create a new project directory and virtual environment
mkdir mysite && cd mysite
uv init
uv add django

# scaffold a new Django project
uv run django-admin startproject mysite .
```

The `.` at the end places files in the current directory instead of a nested folder.

**Resulting structure:**
```
mysite/
    manage.py          ← project CLI
    mysite/
        __init__.py
        settings.py    ← all configuration
        urls.py        ← root URL table
        wsgi.py        ← deployment entry point
        asgi.py        ← async deployment entry point
```

---

# Project Structure — What Each File Does

| File | Purpose |
|---|---|
| `manage.py` | Developer CLI — runserver, migrations, shell … |
| `settings.py` | All configuration: DB, apps, templates, static |
| `urls.py` | Root URL dispatcher |
| `wsgi.py` / `asgi.py` | Entry points for production servers (gunicorn, uvicorn) |

**Create your first app inside the project:**
```bash
uv run python manage.py startapp blog
```

```
blog/
    __init__.py
    admin.py        ← register models in admin
    apps.py         ← app config
    models.py       ← database models
    views.py        ← request handlers
    urls.py         ← (you create this)
    tests.py
    migrations/
```

---

# `manage.py` — the Project CLI

`manage.py` is the single entry point for every developer task:

```bash
# start the development server (auto-reloads on file changes)
uv run python manage.py runserver

# run on a specific port
uv run python manage.py runserver 8080

# create a new app
uv run python manage.py startapp blog

# open an interactive Python shell with Django configured
uv run python manage.py shell

# create and apply database migrations
uv run python manage.py makemigrations
uv run python manage.py migrate

# create an admin superuser account
uv run python manage.py createsuperuser

# collect static files for production
uv run python manage.py collectstatic
```

You will use `runserver` and `shell` dozens of times per day.

---

# Django's Request / Response Cycle

A complete journey for `GET /blog/42/`:

```
1. Browser sends:   GET /blog/42/ HTTP/1.1

2. Django creates:  HttpRequest object
                    (method="GET", path="/blog/42/")

3. urls.py matches: path("blog/<int:pk>/", views.post_detail)

4. View is called:  post_detail(request, pk=42)
                    → queries DB, builds context dict

5. Template renders: blog/post_detail.html
                    {{ post.title }}, {{ post.body }} …

6. Django returns:  HttpResponse(html_string, status=200)

7. Browser renders the HTML page.
```

Every single HTTP request follows this cycle — understanding it is the foundation of all Django development.

---

# Your First View — `HttpResponse`

`views.py` in your app:

```python
from django.http import HttpResponse

def hello(request):
    return HttpResponse("<h1>Hello, world!</h1>")
```

Wire it up in `mysite/urls.py`:

```python
from django.urls import path
from blog import views

urlpatterns = [
    path("hello/", views.hello),
]
```

Visit `http://127.0.0.1:8000/hello/` — Django calls `hello(request)` and sends back the HTML string.

**`HttpResponse` is the lowest-level response.** In practice you almost always use `render()` instead — but knowing `HttpResponse` helps you understand what every other shortcut ultimately produces.

---

# Your First View — `render()`

`render()` is the standard shortcut that combines a template with a context dict and returns an `HttpResponse`.

```python
from django.shortcuts import render

def hello(request):
    context = {
        "name": "Django",
        "version": 5,
    }
    return render(request, "blog/hello.html", context)
```

**Template** `blog/templates/blog/hello.html`:

```html
<!DOCTYPE html>
<html>
<head><title>Hello</title></head>
<body>
  <h1>Hello from {{ name }} {{ version }}!</h1>
</body>
</html>
```

Result: `Hello from Django 5!`

---

# URL Patterns — `urlpatterns`

Every URL in your site must appear in a `urlpatterns` list somewhere.

**Root `mysite/urls.py`:**
```python
from django.contrib import admin
from django.urls import path, include

urlpatterns = [
    path("admin/", admin.site.urls),
    path("blog/", include("blog.urls")),
    path("", include("pages.urls")),
]
```

**App-level `blog/urls.py`:**
```python
from django.urls import path
from . import views

app_name = "blog"   # namespace — important for {% url %} tags

urlpatterns = [
    path("", views.post_list, name="post-list"),
    path("<int:pk>/", views.post_detail, name="post-detail"),
]
```

`include()` delegates everything after the prefix to the app's own URL file. **Always use `app_name` for namespacing.**

---

# Multiple Apps — One Project

A Django **project** is the container; **apps** are the reusable components inside it.

```
mysite/              ← project (one per deployment)
├── mysite/
│   ├── settings.py
│   └── urls.py      ← root router: delegates to each app
├── pages/           ← app: static pages (home, about)
├── blog/            ← app: posts, comments, categories
├── shop/            ← app: products, orders, cart
└── accounts/        ← app: custom user profiles
```

**Why split into apps?**

| Reason | Explanation |
|---|---|
| **Separation of concerns** | Each app owns its models, views, templates, and URLs |
| **Reusability** | A well-scoped app can be packaged and used in another project |
| **Team collaboration** | Different developers own different apps without stepping on each other |
| **`INSTALLED_APPS`** | Django only discovers models, templates, and static files for registered apps |

**Rule of thumb:** if it could sensibly exist in a different project, it deserves its own app.

---

# Multi-App URL Routing — The Cascade

Requests flow through three layers:

```
Browser: GET /blog/42/
    │
    ▼
mysite/urls.py          path("blog/", include("blog.urls"))
    │  strips "blog/" prefix, forwards "42/"
    ▼
blog/urls.py            path("<int:pk>/", views.post_detail, name="post-detail")
    │  captures pk=42
    ▼
blog/views.py           def post_detail(request, pk): …
```

**Root `mysite/urls.py` — routes per app:**
```python
urlpatterns = [
    path("admin/",    admin.site.urls),
    path("",          include("pages.urls")),    # home, about
    path("blog/",     include("blog.urls")),     # blog section
    path("shop/",     include("shop.urls")),     # shop section
    path("accounts/", include("accounts.urls")), # auth
]
```

**Each app's `urls.py` declares its own `app_name`:**
```python
# blog/urls.py
app_name = "blog"   # namespace prefix

urlpatterns = [
    path("",            views.post_list,   name="post-list"),
    path("<int:pk>/",   views.post_detail, name="post-detail"),
    path("new/",        views.post_create, name="post-create"),
]
```

In templates, always qualify with the namespace:  
`{% url 'blog:post-detail' pk=post.pk %}` — never just `{% url 'post-detail' … %}`.

---

# URL Parameters — Capturing Values

Django captures typed values directly in the URL pattern:

```python
# blog/urls.py
urlpatterns = [
    path("<int:pk>/", views.post_detail, name="post-detail"),
    path("<slug:slug>/", views.post_by_slug, name="post-by-slug"),
    path("user/<str:username>/", views.user_profile, name="user-profile"),
]
```

| Converter | Matches | Example |
|---|---|---|
| `str` | Any non-empty string without `/` | `hello` |
| `int` | Zero or positive integer | `42` |
| `slug` | Letters, numbers, hyphens, underscores | `my-post` |
| `uuid` | UUID format | `075194d3-6885-417e-a8a8-6c931e272f00` |
| `path` | Any string including `/` | `images/2024/photo.jpg` |

Captured values are passed as **keyword arguments** to the view function:

```python
def post_detail(request, pk):
    # pk is already an int — Django validated & converted it
    post = get_object_or_404(Post, pk=pk)
    return render(request, "blog/detail.html", {"post": post})
```

---

# The `HttpRequest` Object

Django constructs an `HttpRequest` for every incoming request and passes it to your view as the first argument.

```python
def my_view(request):
    request.method      # "GET", "POST", "PUT", "DELETE", …
    request.path        # "/blog/42/"
    request.GET         # QueryDict of URL query parameters
    request.POST        # QueryDict of form POST data
    request.FILES       # uploaded files
    request.user        # logged-in user (or AnonymousUser)
    request.session     # dict-like session storage
    request.COOKIES     # dict of cookies
    request.META        # raw HTTP headers (REMOTE_ADDR, HTTP_HOST, …)
```

**Common pattern — handle GET and POST in the same view:**

```python
def contact(request):
    if request.method == "POST":
        name = request.POST.get("name", "")
        return HttpResponse(f"Thanks, {name}!")
    return render(request, "contact.html")
```

---

# Template Syntax — Variables and Tags

Django templates use two kinds of special markup:

**Variables** — `{{ variable }}` outputs a value:
```html
<h1>{{ post.title }}</h1>
<p>By {{ post.author.username }} on {{ post.created_at }}</p>
```

**Tags** — `{% tag %}` control logic or load features:
```html
{% if user.is_authenticated %}
  <p>Welcome, {{ user.username }}!</p>
{% endif %}

{% for post in posts %}
  <h2>{{ post.title }}</h2>
{% endfor %}
```

**Comments** — never rendered, not sent to the browser:
```html
{# This is a single-line comment #}

{% comment %}
  This is a
  multi-line comment block.
{% endcomment %}
```

---

# Template Filters

Filters transform a variable's value using `|`:

```html
{{ post.title|upper }}              → MY GREAT POST
{{ post.title|lower }}              → my great post
{{ post.title|title }}              → My Great Post
{{ post.body|truncatewords:20 }}    → First twenty words…
{{ post.body|truncatechars:100 }}   → First 100 characters…
{{ post.created_at|date:"d M Y" }}  → 15 Apr 2025
{{ post.created_at|timesince }}     → 3 days, 2 hours ago
{{ tags|join:", " }}                → python, django, web
{{ post.body|wordcount }}           → 143
{{ items|length }}                  → 5
{{ value|default:"n/a" }}           → n/a  (if value is falsy)
{{ post.body|linebreaks }}          → wraps \n in <p> tags
{{ user_input|escape }}             → escapes < > & " '
{{ html_content|safe }}             → mark as already safe (careful!)
```

Filters can be **chained**: `{{ post.title|lower|truncatechars:30 }}`

---

# Template Tags — `{% if %}` and `{% for %}`

**`{% if %}`** — supports `and`, `or`, `not`, `==`, `!=`, `<`, `>`, `in`, `not in`:

```html
{% if posts %}
  <p>Found {{ posts|length }} post(s).</p>
{% elif drafts %}
  <p>No published posts, but you have drafts.</p>
{% else %}
  <p>No posts yet.</p>
{% endif %}
```

**`{% for %}`** — iterates any iterable; `{% empty %}` handles empty sequences:

```html
<ul>
{% for post in posts %}
  <li>
    {{ forloop.counter }}. {{ post.title }}
    {% if forloop.first %}⭐{% endif %}
  </li>
{% empty %}
  <li>No posts found.</li>
{% endfor %}
</ul>
```

`forloop` variables: `counter`, `counter0`, `revcounter`, `first`, `last`, `parentloop`.

---

# Template Tags — `{% url %}`

Hard-coding URLs in templates is fragile. Use `{% url %}` to generate URLs by name:

```html
<!-- Without namespace -->
<a href="{% url 'post-list' %}">All Posts</a>
<a href="{% url 'post-detail' pk=post.pk %}">Read more</a>

<!-- With app namespace (preferred) -->
<a href="{% url 'blog:post-list' %}">All Posts</a>
<a href="{% url 'blog:post-detail' pk=post.pk %}">Read more</a>
<a href="{% url 'blog:post-by-slug' slug=post.slug %}">{{ post.title }}</a>
```

Django looks up the named URL pattern and builds the correct URL — if you ever change the URL pattern, all `{% url %}` tags automatically update.

**In Python code**, use `reverse()` for the same thing:
```python
from django.urls import reverse

url = reverse("blog:post-detail", kwargs={"pk": 42})
# → "/blog/42/"
```

---

# Template Inheritance — The Problem

Without inheritance, every template repeats the same boilerplate:

```html
<!-- Every single template has this... -->
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>My Site</title>
  <link rel="stylesheet" href="/static/style.css">
</head>
<body>
  <nav>…same nav everywhere…</nav>
  <!-- actual page content -->
  <footer>…same footer everywhere…</footer>
</body>
</html>
```

If you change the nav, you must edit *every* template.

**Solution: template inheritance.** Define a `base.html` with placeholders (`{% block %}`), then child templates fill in only what's different.

---

# Template Inheritance — `{% extends %}` and `{% block %}`

**`templates/base.html`** — the parent layout:
```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <title>{% block title %}My Site{% endblock %}</title>
  <link rel="stylesheet" href="/static/style.css">
</head>
<body>
  <nav><a href="/">Home</a> | <a href="/blog/">Blog</a></nav>
  <main>
    {% block content %}{% endblock %}
  </main>
  <footer>© 2025 My Site</footer>
</body>
</html>
```

**`templates/blog/post_list.html`** — a child template:
```html
{% extends "base.html" %}

{% block title %}Blog Posts — My Site{% endblock %}

{% block content %}
  <h1>Latest Posts</h1>
  {% for post in posts %}
    <h2>{{ post.title }}</h2>
  {% endfor %}
{% endblock %}
```

---

# The Base Template Pattern

A well-designed `base.html` exposes multiple named blocks:

```html
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>{% block title %}My Site{% endblock %}</title>
  {% block extra_head %}{% endblock %}
</head>
<body class="{% block body_class %}{% endblock %}">
  <header>
    <nav>{% block nav %}
      <a href="/">Home</a>
      <a href="/blog/">Blog</a>
    {% endblock %}</nav>
  </header>

  <main>{% block content %}{% endblock %}</main>

  <footer>
    {% block footer %}<p>© 2025</p>{% endblock %}
  </footer>

  {% block extra_js %}{% endblock %}
</body>
</html>
```

Child templates override only the blocks they need. **Blocks can have default content.**

---

# `{% include %}` — Reusable Fragments

`{% include %}` inserts another template at that point — useful for repeated UI fragments:

```html
<!-- base.html -->
<nav>
  {% include "partials/navbar.html" %}
</nav>
```

```html
<!-- blog/post_list.html -->
{% for post in posts %}
  {% include "blog/partials/post_card.html" with post=post %}
{% endfor %}
```

The included template shares the current context automatically. Use `with` to pass extra variables or `only` to restrict the context:

```html
{% include "partials/alert.html" with message="Saved!" level="success" only %}
```

**Typical use cases:**
- Navigation bar shared between layouts
- Post/product card used in lists
- Pagination controls
- Flash message banners

---

# Static Files — CSS, JS, Images

Static files (CSS, JavaScript, images) are served separately from templates.

**`settings.py`:**
```python
STATIC_URL = "/static/"
# optional: extra directories to search
STATICFILES_DIRS = [BASE_DIR / "static"]
```

**Directory convention:**
```
blog/
    static/
        blog/
            style.css
            logo.png
```

**In a template** — always use `{% load static %}` first:
```html
{% load static %}
<!DOCTYPE html>
<html>
<head>
  <link rel="stylesheet" href="{% static 'blog/style.css' %}">
</head>
<body>
  <img src="{% static 'blog/logo.png' %}" alt="Logo">
  <script src="{% static 'blog/app.js' %}"></script>
</body>
</html>
```

`{% static %}` generates the correct URL — in development it is `/static/blog/style.css`.

---

# Static Files — Development vs Production

**Development (`DEBUG=True`):**

Django's `runserver` serves static files automatically via `django.contrib.staticfiles`. No extra configuration needed.

**Production (`DEBUG=False`):**

```bash
# Collect all static files into STATIC_ROOT
uv run python manage.py collectstatic
```

```python
# settings.py
STATIC_ROOT = BASE_DIR / "staticfiles"   # destination directory
```

A dedicated web server (nginx, Caddy) or a service like WhiteNoise / AWS S3 then serves the files directly — **Django should never serve static files in production.**

```python
# Quick production static with WhiteNoise (add to settings.py)
MIDDLEWARE = [
    "whitenoise.middleware.WhiteNoiseMiddleware",
    ...
]
STATICFILES_STORAGE = "whitenoise.storage.CompressedManifestStaticFilesStorage"
```

---

# Context — Passing Data from View to Template

The **context** is a plain Python `dict` that maps template variable names to Python values.

```python
def post_list(request):
    posts = Post.objects.filter(published=True).order_by("-created_at")
    context = {
        "posts": posts,
        "page_title": "Latest Posts",
        "count": posts.count(),
    }
    return render(request, "blog/post_list.html", context)
```

In the template:
```html
<h1>{{ page_title }}</h1>
<p>{{ count }} post(s) found.</p>
{% for post in posts %}
  <article>
    <h2>{{ post.title }}</h2>
    <p>{{ post.body|truncatewords:30 }}</p>
  </article>
{% endfor %}
```

You can pass **any Python object** — strings, numbers, lists, querysets, model instances, dicts. Django's template engine accesses attributes with `.` notation.

---

# `render()` — What It Does Under the Hood

```python
return render(request, "blog/post_list.html", context)
```

This single call is equivalent to:

```python
from django.template.loader import get_template
from django.http import HttpResponse

# 1. Locate the template file in TEMPLATES dirs / app template dirs
template = get_template("blog/post_list.html")

# 2. Render it: substitute {{ variables }} and execute {% tags %}
html_string = template.render(context, request)

# 3. Wrap in an HTTP 200 response with Content-Type: text/html
return HttpResponse(html_string, content_type="text/html; charset=utf-8")
```

`render()` also accepts an optional `status` argument:
```python
return render(request, "404.html", status=404)
return render(request, "blog/form.html", context, status=422)
```

---

# Django Settings — Key Settings

`settings.py` is ordinary Python — you can import, compute, and read environment variables.

```python
from pathlib import Path
import os

BASE_DIR = Path(__file__).resolve().parent.parent

SECRET_KEY = os.environ.get("SECRET_KEY", "dev-only-insecure-key")
DEBUG = os.environ.get("DEBUG", "True") == "True"
ALLOWED_HOSTS = ["localhost", "127.0.0.1"]

INSTALLED_APPS = [
    "django.contrib.admin",
    "django.contrib.auth",
    "django.contrib.contenttypes",
    "django.contrib.sessions",
    "django.contrib.messages",
    "django.contrib.staticfiles",
    "blog",       # ← your apps go here
]

TEMPLATES = [{
    "BACKEND": "django.template.backends.django.DjangoTemplates",
    "DIRS": [BASE_DIR / "templates"],   # project-wide templates
    "APP_DIRS": True,                   # also look in app/templates/
    "OPTIONS": {"context_processors": [...]},
}]

STATIC_URL = "/static/"
```

---

# `INSTALLED_APPS` and Template Discovery

For Django to find templates inside your app, two things must be true:

**1. Your app must be in `INSTALLED_APPS`:**
```python
INSTALLED_APPS = [
    ...
    "blog",   # ← without this, app templates are invisible
]
```

**2. Your template must be in `app/templates/app/template.html`:**
```
blog/
    templates/
        blog/            ← repeat the app name as a subdirectory!
            post_list.html
            post_detail.html
            partials/
                post_card.html
```

The extra `blog/` subdirectory prevents naming collisions between apps. If two apps both had `templates/index.html`, Django would use whichever app comes first in `INSTALLED_APPS`. Namespacing with `blog/index.html` avoids this entirely.

---

# Common URL Mistakes — 404 in Debug Mode

When a URL doesn't match any pattern, Django shows a helpful debug page listing every URL it tried:

```
Page not found (404)
Request Method: GET
Request URL:    http://127.0.0.1:8000/bolg/
Using the URLconf defined in mysite.urls, Django tried these URL patterns,
in this order:
    admin/
    blog/
    [name='post-list']
No patterns matched the URL path: 'bolg/'
```

**Common causes:**
- Typo in the URL (note: `bolg/` vs `blog/`)
- Forgot to `include("blog.urls")` in the root `urls.py`
- Forgot to add a trailing slash (`/blog` instead of `/blog/`)
- `APPEND_SLASH = True` (default) will redirect `GET /blog` → `GET /blog/` automatically if the pattern has a slash

**Check the debug page first** — it tells you exactly what Django tried.

---

# Common URL Mistakes — `NoReverseMatch`

`NoReverseMatch` means `{% url 'name' %}` or `reverse('name')` couldn't find the named pattern.

```
NoReverseMatch at /blog/
Reverse for 'post-detail' with keyword arguments '{'pk': 1}' not found.
```

**Checklist:**
```python
# 1. Is app_name set in blog/urls.py?
app_name = "blog"

# 2. Are you using the namespace in the template?
{% url 'blog:post-detail' pk=post.pk %}   # ✓ with namespace
{% url 'post-detail' pk=post.pk %}        # ✗ missing namespace

# 3. Does the pattern name match exactly?
path("<int:pk>/", views.post_detail, name="post-detail")
# 'post-detail' ≠ 'post_detail' — hyphens and underscores differ!

# 4. Is the app in INSTALLED_APPS and included in root urls.py?
path("blog/", include("blog.urls")),
```

---

# Template Debugging

When a template variable is missing, Django **silently outputs an empty string** by default — this is intentional (avoids crashing public sites).

During development, make missing variables loud:

**`settings.py`:**
```python
TEMPLATES = [{
    ...
    "OPTIONS": {
        "context_processors": [...],
        "string_if_invalid": "MISSING: %s",   # ← add this
    },
}]
```

Now `{{ typo_variable }}` renders as `MISSING: typo_variable` — easy to spot.

**Django's 500 debug page** shows:
- Full Python traceback with local variables
- Template file path and line number where the error occurred
- All context variables passed to the template
- Settings snapshot

Only visible when `DEBUG = True` — in production it shows a generic 500 page.

---

# Template Debugging — Tips

**Inspect the context from inside the template:**
```html
{% comment %}Debug block — remove before commit{% endcomment %}
<pre>{{ request.user }}</pre>
<pre>{{ posts|length }} posts</pre>
```

**Use `{% debug %}` tag to dump all context variables:**
```html
{% if debug %}
  {% debug %}
{% endif %}
```

**Django shell — test view logic interactively:**
```python
uv run python manage.py shell

>>> from blog.views import post_list
>>> from django.test import RequestFactory
>>> rf = RequestFactory()
>>> req = rf.get("/blog/")
>>> response = post_list(req)
>>> response.status_code
200
>>> b"Latest Posts" in response.content
True
```

---

# Quick Reference — Template Cheatsheet

```html
{% load static %}           {# load the static tag library #}

{{ variable }}              {# output variable #}
{{ obj.attr }}              {# attribute / dict key / list index #}
{{ value|filter }}          {# apply filter #}
{{ value|filter:arg }}      {# filter with argument #}

{% if x %} … {% elif y %} … {% else %} … {% endif %}
{% for item in list %} … {% empty %} … {% endfor %}
{% block name %} … {% endblock %}
{% extends "base.html" %}
{% include "partial.html" %}
{% include "p.html" with key=val only %}
{% url 'namespace:name' arg1 kwarg=val %}
{% static 'path/to/file.css' %}
{% csrf_token %}            {# required in every POST form #}

{# single-line comment #}
{% comment %} … {% endcomment %}
```

---

# A Complete Example — Blog Post List

**`blog/views.py`:**
```python
from django.shortcuts import render, get_object_or_404
from .models import Post

def post_list(request):
    posts = Post.objects.filter(published=True).order_by("-created_at")
    return render(request, "blog/post_list.html", {"posts": posts})

def post_detail(request, pk):
    post = get_object_or_404(Post, pk=pk, published=True)
    return render(request, "blog/post_detail.html", {"post": post})
```

**`blog/urls.py`:**
```python
from django.urls import path
from . import views

app_name = "blog"
urlpatterns = [
    path("", views.post_list, name="post-list"),
    path("<int:pk>/", views.post_detail, name="post-detail"),
]
```

**`mysite/urls.py`:**
```python
from django.urls import path, include
urlpatterns = [
    path("blog/", include("blog.urls")),
]
```

---

# A Complete Example — Templates

**`templates/blog/post_list.html`:**
```html
{% extends "base.html" %}
{% block title %}Blog — My Site{% endblock %}

{% block content %}
<h1>Latest Posts</h1>
{% for post in posts %}
  <article>
    <h2><a href="{% url 'blog:post-detail' pk=post.pk %}">{{ post.title }}</a></h2>
    <p>{{ post.created_at|date:"d M Y" }} · {{ post.body|truncatewords:25 }}</p>
  </article>
{% empty %}
  <p>No posts yet.</p>
{% endfor %}
{% endblock %}
```

**`templates/blog/post_detail.html`:**
```html
{% extends "base.html" %}
{% block title %}{{ post.title }} — My Site{% endblock %}

{% block content %}
<article>
  <h1>{{ post.title }}</h1>
  <p><small>{{ post.created_at|date:"d M Y" }}</small></p>
  {{ post.body|linebreaks }}
</article>
<a href="{% url 'blog:post-list' %}">← Back to list</a>
{% endblock %}
```

---

# Django Admin — Free CRUD Interface

Once you have models, you get a full admin interface for free:

```python
# blog/admin.py
from django.contrib import admin
from .models import Post

@admin.register(Post)
class PostAdmin(admin.ModelAdmin):
    list_display = ("title", "author", "published", "created_at")
    list_filter = ("published", "created_at")
    search_fields = ("title", "body")
    prepopulated_fields = {"slug": ("title",)}
```

```bash
uv run python manage.py createsuperuser
uv run python manage.py runserver
# visit http://127.0.0.1:8000/admin/
```

The admin is in `INSTALLED_APPS` and `urls.py` by default — it's ready the moment you run migrations. Useful for managing content during development and in production.

---

# `get_object_or_404` — Safe Object Fetching

Fetching an object that might not exist is so common that Django provides a shortcut:

```python
# Without the shortcut — verbose
from django.http import Http404
from .models import Post

def post_detail(request, pk):
    try:
        post = Post.objects.get(pk=pk)
    except Post.DoesNotExist:
        raise Http404("Post not found")
    return render(request, "blog/post_detail.html", {"post": post})
```

```python
# With the shortcut — clean
from django.shortcuts import render, get_object_or_404
from .models import Post

def post_detail(request, pk):
    post = get_object_or_404(Post, pk=pk, published=True)
    return render(request, "blog/post_detail.html", {"post": post})
```

`get_object_or_404` raises `Http404` (which Django renders as a 404 response) rather than letting a `DoesNotExist` exception bubble up as a 500 error.

---

# Redirects

After a successful form submission you should **redirect** rather than re-render — this prevents duplicate submissions on browser refresh (Post/Redirect/Get pattern).

```python
from django.shortcuts import redirect

def create_post(request):
    if request.method == "POST":
        # ... save the post ...
        # redirect to the new post's detail page
        return redirect("blog:post-detail", pk=new_post.pk)
    return render(request, "blog/create.html")
```

`redirect()` accepts:
```python
redirect("/blog/")                          # hard-coded URL
redirect("blog:post-list")                  # named URL (no args)
redirect("blog:post-detail", pk=42)         # named URL with args
redirect(post)                              # model with get_absolute_url()
```

Returns `HttpResponseRedirect` (302) by default; pass `permanent=True` for 301.

---

# Summary

<div class="columns">
<div>

**Core concepts**
- Framework = reusable HTTP plumbing
- MVT: Model · View · Template
- `manage.py` — developer CLI
- Request/response cycle

**Views**
- `HttpResponse` — raw response
- `render()` — template + context
- `get_object_or_404` — safe fetch
- `redirect()` — PRG pattern

</div>
<div>

**URLs**
- `urlpatterns` + `path()`
- `include()` for app URL files
- Multiple apps: `pages/`, `blog/`, `shop/` …
- URL namespaces: `app_name` + `blog:post-detail`
- Type converters: `int`, `str`, `slug`
- `{% url %}` / `reverse()`

**Templates**
- `{{ var }}` and `{% tag %}`
- Filters: `|date`, `|truncatewords` …
- Inheritance: `extends` / `block`
- `{% include %}` for partials
- `{% load static %}` / `{% static %}`

</div>
</div>

---

# Lab 4 Preview

**You will build a small blog application from scratch.**

1. Scaffold a new Django project with `uv` and `django-admin startproject`
2. Create a `blog` app; register it in `INSTALLED_APPS`
3. Define `Post` model (title, body, slug, created\_at, published)
4. Create and apply migrations; register `Post` in the admin
5. Write three views: `post_list`, `post_detail`, `about`
6. Wire up URLs with namespacing and `include()`; add a second `pages` app alongside `blog`
7. Build a `base.html` with navigation and footer blocks
8. Create child templates for each view using `{% extends %}`
9. Add a static CSS file; link it with `{% load static %}`
10. Use `{% url %}` tags everywhere — no hard-coded paths

**Bonus:** add a search view that filters posts by title using `request.GET.get("q")` and passes results to a template.

---

# Questions?

<br>

## Key Takeaways

- Django's MVT pattern separates concerns cleanly — views are controllers
- Every URL must be named so templates can use `{% url %}`
- Always put app templates in `app/templates/app/` to avoid collisions
- Template inheritance (`extends` / `block`) keeps layouts DRY
- Use `render()`, `get_object_or_404()`, and `redirect()` — they exist for good reasons

<br>

**Next lecture:** Django Models & the ORM — defining database schemas in Python, QuerySets, filtering, ordering, and relations.
