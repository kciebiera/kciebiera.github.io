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

# Lecture 5
## Django — Models, ORM & Forms

**WWW 25/26**
ORM · Migrations · Admin · ModelForms · Validation · CSRF

---

# Why Databases?

## The Persistence Problem

Every Python variable you create lives in **RAM**.  
When the process stops — server restart, crash, redeploy — it is **gone**.

```python
# views.py — DO NOT do this
posts = []   # ← wiped on every restart

def create_post(request):
    posts.append({"title": request.POST["title"]})
```

You need a place to store data that **survives** process restarts, scales beyond a single machine, and allows multiple workers to read and write simultaneously.

**Answer: a relational database** (PostgreSQL in production, SQLite for development).

Django handles the connection, query building, and schema management for you — but you need to understand what it does under the hood.

---

# Relational Databases Recap

A relational database stores data in **tables** (also called relations).

| id | title            | published | category_id |
|----|-----------------|-----------|-------------|
| 1  | Hello Django    | true      | 2           |
| 2  | ORM Deep Dive   | false     | 2           |
| 3  | CSS Tricks      | true      | 1           |

Key concepts:
- **Row** — one record (one blog post, one user…)
- **Column** — one attribute (title, published date…)
- **Primary key** — unique identifier per row (usually `id`)
- **Foreign key** — a column that references a primary key in **another** table (`category_id → categories.id`)

Rows in different tables are **joined** on matching key values.  
This avoids duplicating the category name in every post row.

---

# What is an ORM?

## Object-Relational Mapper

An ORM lets you work with database rows as **Python objects** instead of writing raw SQL.

<div class="columns">

<div>

**Without ORM (raw SQL):**
```python
import sqlite3
conn = sqlite3.connect("db.sqlite3")
cur = conn.cursor()
cur.execute(
    "SELECT * FROM blog_post WHERE published=1"
)
rows = cur.fetchall()
```

</div>

<div>

**With Django ORM:**
```python
from blog.models import Post

posts = Post.objects.filter(
    published=True
)
```

</div>
</div>

Django maps:
- Python **class** → SQL **table**
- Class **attribute** → table **column**
- Class **instance** → table **row**
- Calling `.save()` → `INSERT` or `UPDATE`

The ORM generates correct SQL for your database backend — switch from SQLite to PostgreSQL by changing one setting.

---

# Defining a Django Model

Create a file `blog/models.py` and subclass `models.Model`:

```python
from django.db import models

class Post(models.Model):
    title      = models.CharField(max_length=200)
    slug       = models.SlugField(max_length=200, unique=True)
    body       = models.TextField()
    created_at = models.DateTimeField(auto_now_add=True)
    updated_at = models.DateTimeField(auto_now=True)
    published  = models.BooleanField(default=False)

    def __str__(self):
        return self.title
```

Common field types:

| Field class | SQL type | Use for |
|---|---|---|
| `CharField` | `VARCHAR` | short text, requires `max_length` |
| `TextField` | `TEXT` | long text, no length limit |
| `IntegerField` | `INTEGER` | whole numbers |
| `BooleanField` | `BOOLEAN` | true/false flags |
| `DateTimeField` | `DATETIME` | timestamps |
| `SlugField` | `VARCHAR` | URL-friendly identifiers |

---

# Field Options

Every field accepts keyword arguments that add constraints and behaviour:

```python
class Article(models.Model):
    title    = models.CharField(max_length=255, unique=True)
    subtitle = models.CharField(max_length=255, blank=True, default="")
    views    = models.IntegerField(default=0)
    deleted  = models.BooleanField(default=False, db_index=True)
    note     = models.TextField(null=True, blank=True)
    created  = models.DateTimeField(auto_now_add=True)
    updated  = models.DateTimeField(auto_now=True)
```

| Option | Meaning |
|---|---|
| `max_length=N` | Maximum string length (required for `CharField`) |
| `unique=True` | Adds a `UNIQUE` constraint to the column |
| `null=True` | Allows `NULL` in the database |
| `blank=True` | Allows empty value in **form** validation |
| `default=…` | Value used when not supplied |
| `auto_now_add=True` | Set to `now()` on **create** only |
| `auto_now=True` | Set to `now()` on every **save** |
| `db_index=True` | Creates a database index for faster lookups |

**Rule of thumb:** use `blank=True, default=""` for optional `CharField`; use `null=True, blank=True` for optional `TextField` or numeric fields.

---

# Relationships — ForeignKey

A `ForeignKey` creates a many-to-one relationship: many posts belong to one category.

```python
class Category(models.Model):
    name = models.CharField(max_length=100)
    slug = models.SlugField(unique=True)

    def __str__(self):
        return self.name


class Post(models.Model):
    category = models.ForeignKey(
        Category,
        on_delete=models.CASCADE,
        related_name="posts",
    )
    title = models.CharField(max_length=200)
    body  = models.TextField()
```

Django adds a `category_id` column to the `post` table automatically.

**`on_delete` choices:**

| Option | Behaviour when the Category is deleted |
|---|---|
| `CASCADE` | Also delete all related Posts |
| `SET_NULL` | Set `category_id` to `NULL` (requires `null=True`) |
| `PROTECT` | Raise an error — prevents deletion |
| `SET_DEFAULT` | Set to the field's `default` value |

---

# The Meta Class

Add an inner `Meta` class to control table-level behaviour:

```python
class Post(models.Model):
    title     = models.CharField(max_length=200)
    slug      = models.SlugField(unique=True)
    published = models.BooleanField(default=False)
    created   = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering      = ["-created"]       # newest first by default
        verbose_name  = "post"             # singular label in admin
        verbose_name_plural = "posts"      # plural label in admin
        db_table      = "blog_post"        # explicit table name
        indexes = [
            models.Index(fields=["slug"]),
        ]

    def __str__(self):
        return self.title
```

- **`ordering`** — default sort for `Post.objects.all()`. Prefix `-` for descending.
- **`verbose_name`** — shown in the Django admin and error messages.
- **`db_table`** — override the auto-generated name (`appname_modelname`).
- **`indexes`** — define composite or single-column database indexes.

---

# The `__str__` Method

`__str__` controls the human-readable representation of a model instance.

```python
class Post(models.Model):
    title = models.CharField(max_length=200)

    def __str__(self):
        return self.title
```

Without `__str__` you get useless output everywhere:

```
# Admin list page shows:    Post object (1)
# Shell shows:              <Post: Post object (1)>
# ForeignKey dropdown shows: Post object (1)
```

With `__str__` you get:

```
# Admin list page shows:    Hello Django
# Shell shows:              <Post: Hello Django>
# ForeignKey dropdown shows: Hello Django
```

Always define `__str__`. It is the single highest-value method you can add to any model.

---

# Migrations

A **migration** is a Python file that describes a change to the database schema.

```
blog/
  migrations/
    0001_initial.py        ← create Post table
    0002_post_add_slug.py  ← add slug column
    0003_category.py       ← create Category, add FK
```

Two commands:

```bash
# 1. Inspect your models and generate a new migration file
python manage.py makemigrations

# 2. Apply all unapplied migrations to the database
python manage.py migrate
```

**Workflow:**
1. Edit `models.py`
2. Run `makemigrations` → creates a migration file (commit this to git)
3. Run `migrate` → alters the database

Django records which migrations have been applied in a `django_migrations` table.  
If you forget to run `migrate`, your code and your database are **out of sync** and you will get errors.

---

# Reading a Migration File

`makemigrations` generates plain Python — you can (and should) read it:

```python
# blog/migrations/0001_initial.py
from django.db import migrations, models

class Migration(migrations.Migration):

    initial = True

    dependencies = []

    operations = [
        migrations.CreateModel(
            name="Post",
            fields=[
                ("id", models.BigAutoField(primary_key=True)),
                ("title", models.CharField(max_length=200)),
                ("slug", models.SlugField(unique=True)),
                ("body", models.TextField()),
                ("published", models.BooleanField(default=False)),
                ("created_at", models.DateTimeField(auto_now_add=True)),
            ],
        ),
    ]
```

Each migration lists its **dependencies** (which migrations must run first) and a list of **operations** (`CreateModel`, `AddField`, `AlterField`, `DeleteModel`, …).

Never edit model fields directly in the database — always go through migrations.

---

# `sqlmigrate` — Seeing the Raw SQL

Curious what SQL Django will run? Use `sqlmigrate`:

```bash
python manage.py sqlmigrate blog 0001
```

Output (SQLite):
```sql
BEGIN;
--
-- Create model Post
--
CREATE TABLE "blog_post" (
    "id"         integer NOT NULL PRIMARY KEY AUTOINCREMENT,
    "title"      varchar(200) NOT NULL,
    "slug"       varchar(50) NOT NULL UNIQUE,
    "body"       text NOT NULL,
    "published"  bool NOT NULL,
    "created_at" datetime NOT NULL
);
COMMIT;
```

This is read-only — it shows what **would** run without touching the database.  
Use it to:
- Double-check Django generates the SQL you expect
- Understand what `UNIQUE`, `NOT NULL`, etc. map to
- Debug migration issues before applying them

---

# The Django Admin

Django ships with a fully-featured admin interface.  
Register your models to manage them through a web UI — no code beyond one line.

```python
# blog/admin.py
from django.contrib import admin
from .models import Post, Category

admin.site.register(Category)
admin.site.register(Post)
```

Navigate to `http://127.0.0.1:8000/admin/` after creating a superuser.

The admin gives you for free:
- **List view** — paginated table of all rows
- **Detail/edit view** — form for every field
- **Create** and **Delete** actions
- **Search** and **filter** sidebar
- Full **history** of every change

It is not meant for end users — it is a **developer and editor tool**.

---

# `@admin.register` and `ModelAdmin`

Use the decorator form and a `ModelAdmin` subclass for more control:

```python
from django.contrib import admin
from .models import Post

@admin.register(Post)
class PostAdmin(admin.ModelAdmin):
    list_display  = ["title", "category", "published", "created_at"]
    list_filter   = ["published", "category"]
    search_fields = ["title", "body"]
    prepopulated_fields = {"slug": ("title",)}
    date_hierarchy = "created_at"
    ordering       = ["-created_at"]
```

| Option | What it does |
|---|---|
| `list_display` | Columns shown in the list view |
| `list_filter` | Right-side filter panel |
| `search_fields` | Enables a search box; searches these fields |
| `prepopulated_fields` | Auto-fills slug from title via JavaScript |
| `date_hierarchy` | Drill-down navigation by date |

---

# Creating a Superuser

The admin requires a user with `is_staff=True` and `is_superuser=True`.

```bash
python manage.py createsuperuser
```

```
Username: admin
Email address: admin@example.com
Password: ********
Password (again): ********
Superuser created successfully.
```

Then start the dev server and visit `/admin/`:

```bash
python manage.py runserver
# Open: http://127.0.0.1:8000/admin/
```

You can also create staff users (with limited permissions) in the admin under **Authentication → Users**.

Superusers bypass all permission checks — never use one as the main application user in production.

---

# The ORM — `Model.objects`

Every model has a **manager** attached as `Model.objects`.  
The manager is the entry point for all database queries.

```python
from blog.models import Post

# All posts
all_posts = Post.objects.all()

# One specific post
post = Post.objects.get(id=1)

# Posts matching a condition
published = Post.objects.filter(published=True)

# Posts NOT matching a condition
drafts = Post.objects.exclude(published=True)

# Count without fetching rows
n = Post.objects.filter(published=True).count()

# Does any matching row exist?
exists = Post.objects.filter(slug="hello").exists()
```

You can define **custom managers** to encapsulate common queries:

```python
class PublishedManager(models.Manager):
    def get_queryset(self):
        return super().get_queryset().filter(published=True)

class Post(models.Model):
    objects   = models.Manager()      # default
    published = PublishedManager()    # custom
```

---

# QuerySets — Lazy Evaluation

A `QuerySet` is **lazy** — it does not hit the database until you actually need the data.

```python
qs = Post.objects.filter(published=True)   # no SQL yet
qs = qs.order_by("-created_at")            # still no SQL
qs = qs.exclude(title__startswith="Draft") # still no SQL

# SQL fires here, when the QuerySet is evaluated:
for post in qs:          # iteration
    print(post.title)

list(qs)                 # force evaluation
qs[0]                    # slicing
len(qs)                  # len()
bool(qs)                 # bool check
```

This makes **chaining** efficient — build up a query across multiple lines, then execute once:

```python
def post_list(request):
    qs = Post.objects.filter(published=True)
    if request.GET.get("q"):
        qs = qs.filter(title__icontains=request.GET["q"])
    qs = qs.order_by("-created_at")[:10]
    return render(request, "blog/list.html", {"posts": qs})
```

---

# Basic Queries

```python
# Return ALL rows — QuerySet
Post.objects.all()

# Return ONE row — raises DoesNotExist or MultipleObjectsReturned
post = Post.objects.get(id=42)

# Return rows matching conditions — QuerySet
Post.objects.filter(published=True)

# Return rows NOT matching conditions — QuerySet
Post.objects.exclude(category__name="Drafts")

# Count rows
Post.objects.filter(published=True).count()

# Order results
Post.objects.all().order_by("title")        # A→Z
Post.objects.all().order_by("-created_at")  # newest first

# Limit results (SQL LIMIT/OFFSET)
Post.objects.all()[:5]         # first 5
Post.objects.all()[10:20]      # rows 11–20

# Return distinct values
Post.objects.values("category_id").distinct()
```

`get()` is for when you expect **exactly one** result.  
For user-facing lookups always use `get_object_or_404()` (covered shortly).

---

# Field Lookups

Django appends lookup operators to field names with double underscores (`__`):

```python
# Exact match (default, same as filter(title="Hello"))
Post.objects.filter(title__exact="Hello")

# Case-insensitive match
Post.objects.filter(title__iexact="hello")

# Contains substring
Post.objects.filter(body__contains="Django")
Post.objects.filter(body__icontains="django")   # case-insensitive

# Comparison operators
Post.objects.filter(views__gt=100)     # greater than
Post.objects.filter(views__gte=100)    # greater than or equal
Post.objects.filter(views__lt=10)      # less than
Post.objects.filter(views__lte=10)     # less than or equal

# Value in a list
Post.objects.filter(id__in=[1, 2, 3])

# NULL check
Post.objects.filter(subtitle__isnull=True)

# Starts / ends with
Post.objects.filter(slug__startswith="2024-")
Post.objects.filter(title__endswith="Guide")
```

---

# Related Object Queries

Follow `ForeignKey` relationships in filter arguments using `__`:

```python
# Posts whose category name is "Python"
Post.objects.filter(category__name="Python")

# Case-insensitive
Post.objects.filter(category__name__icontains="python")

# Traverse multiple levels (if Category had a parent FK)
Post.objects.filter(category__parent__slug="tech")

# Reverse relation: from Category, get posts
category = Category.objects.get(slug="python")
category.posts.all()          # uses related_name="posts"
category.posts.filter(published=True)
category.posts.count()
```

Django generates a `JOIN` automatically.  
You never write `SELECT … JOIN … ON …` by hand.

```python
# Accessing the related object on an instance
post = Post.objects.get(id=1)
print(post.category.name)   # → "Python"  (one extra query!)
```

---

# Q Objects — Complex Queries

`filter()` chains are AND conditions. For **OR** (or negation), use `Q` objects:

```python
from django.db.models import Q

# OR: published OR created this year
Post.objects.filter(
    Q(published=True) | Q(created_at__year=2025)
)

# AND explicitly (same as keyword arguments)
Post.objects.filter(
    Q(published=True) & Q(category__name="Python")
)

# NOT: exclude published posts
Post.objects.filter(~Q(published=True))

# Combine freely
Post.objects.filter(
    Q(title__icontains="django") | Q(body__icontains="django"),
    published=True,       # keyword args are always AND
)
```

`Q` objects support `|` (OR), `&` (AND), and `~` (NOT).  
Keyword arguments in the same `filter()` call are AND-ed with the `Q` objects.

---

# `get_object_or_404`

In views, you often want to fetch one object and return a 404 if it doesn't exist.

**Without helper:**
```python
from django.http import Http404
from .models import Post

def post_detail(request, slug):
    try:
        post = Post.objects.get(slug=slug, published=True)
    except Post.DoesNotExist:
        raise Http404("Post not found")
    return render(request, "blog/detail.html", {"post": post})
```

**With helper (preferred):**
```python
from django.shortcuts import get_object_or_404, render
from .models import Post

def post_detail(request, slug):
    post = get_object_or_404(Post, slug=slug, published=True)
    return render(request, "blog/detail.html", {"post": post})
```

`get_object_or_404` raises `Http404` automatically.  
Django turns `Http404` into a proper 404 response page.  
Always use it in user-facing views — never let `DoesNotExist` propagate as a 500.

---

# What is a Django Form?

A **Form** is a Python class that:
1. Describes which fields to render and validate
2. Accepts user input (from `request.POST`)
3. Validates that input and returns clean Python values

<div class="columns">

<div>

**`Form`** — build fields manually:
```python
from django import forms

class ContactForm(forms.Form):
    name    = forms.CharField(max_length=100)
    email   = forms.EmailField()
    message = forms.CharField(
        widget=forms.Textarea
    )
```

</div>

<div>

**`ModelForm`** — derive fields from a model:
```python
from django import forms
from .models import Post

class PostForm(forms.ModelForm):
    class Meta:
        model  = Post
        fields = ["title", "body", "published"]
```

</div>
</div>

Use plain `Form` for things like search, login, or contact.  
Use `ModelForm` whenever the form directly creates or updates a model instance — it eliminates redundancy.

---

# ModelForm

`ModelForm` introspects your model and generates fields automatically:

```python
# forms.py
from django import forms
from .models import Post

class PostForm(forms.ModelForm):
    class Meta:
        model   = Post
        fields  = ["title", "slug", "body", "category", "published"]
        widgets = {
            "body": forms.Textarea(attrs={"rows": 10, "class": "form-control"}),
            "title": forms.TextInput(attrs={"class": "form-control"}),
        }
        labels = {
            "published": "Publish immediately",
        }
        help_texts = {
            "slug": "URL-friendly identifier, auto-filled from title.",
        }
```

- **`fields`** — explicit allowlist (preferred over `exclude`).  
  Never use `fields = "__all__"` — it exposes every column including sensitive ones.
- **`widgets`** — override the default HTML widget for any field.
- **`labels`** — override the human-readable label.
- **`help_texts`** — add `<small>` hint text below the field.

---

# Form Validation — `is_valid()`

The standard form-handling pattern in a view:

```python
# views.py
from django.shortcuts import render, redirect
from .forms import PostForm

def post_create(request):
    if request.method == "POST":
        form = PostForm(request.POST)
        if form.is_valid():
            form.save()
            return redirect("blog:list")
        # form is not valid — fall through to render with errors
    else:
        form = PostForm()

    return render(request, "blog/post_form.html", {"form": form})
```

What `is_valid()` does:
1. Runs each field's built-in validators (`max_length`, `required`, type coercion)
2. Runs any custom `clean_<field>()` methods
3. Runs the form-wide `clean()` method
4. Populates `form.cleaned_data` with safe Python values on success
5. Populates `form.errors` with error messages on failure

`form.cleaned_data["title"]` is always a sanitised Python value — never use raw `request.POST` values.

---

# Custom Validators — `clean_<field>` and `clean`

Add custom validation by overriding methods on the form:

```python
from django import forms
from .models import Post

class PostForm(forms.ModelForm):
    class Meta:
        model  = Post
        fields = ["title", "slug", "body"]

    def clean_slug(self):
        slug = self.cleaned_data["slug"]
        if Post.objects.filter(slug=slug).exclude(pk=self.instance.pk).exists():
            raise forms.ValidationError("A post with this slug already exists.")
        if not slug.replace("-", "").isalnum():
            raise forms.ValidationError("Slug may only contain letters, digits, and hyphens.")
        return slug

    def clean(self):
        cleaned = super().clean()
        title = cleaned.get("title", "")
        body  = cleaned.get("body", "")
        if title and body and title.lower() in body.lower():
            raise forms.ValidationError("Body should not simply repeat the title.")
        return cleaned
```

- `clean_<field>()` — validates one field; receives the value **after** built-in validation.
- `clean()` — cross-field validation; use `self.add_error(field, msg)` to attach errors to a specific field.

---

# CSRF Protection

**Cross-Site Request Forgery (CSRF):** an attacker tricks a logged-in user's browser into sending a forged POST request to your site.

Without protection: visiting `evil.com` could silently submit a form on your behalf.

Django's defence: a **secret token** is embedded in every form and verified on submit.

```html
<form method="post" action="/posts/create/">
  {% csrf_token %}
  {{ form.as_p }}
  <button type="submit">Save</button>
</form>
```

`{% csrf_token %}` renders a hidden input:
```html
<input type="hidden" name="csrfmiddlewaretoken" value="abc123…xyz">
```

Django's `CsrfViewMiddleware` checks this token on every `POST`, `PUT`, `PATCH`, `DELETE` request.  
If the token is missing or wrong → **403 Forbidden**.

**When you don't need it:**
- GET requests (search forms, filters) — no side effects, no CSRF risk
- APIs using token-based auth (JWT, API keys) — typically disable CSRF for those endpoints

---

# The POST-Redirect-GET Pattern

**Problem:** if the user refreshes after a `POST`, the browser resends the form.  
This causes duplicate records — e.g., two identical posts created.

```
Browser                    Server
  │─── POST /posts/create ──▶│  Creates post
  │◀── 200 OK (HTML page) ───│
  │
  │  (user presses F5)
  │─── POST /posts/create ──▶│  Creates ANOTHER post  ← bug!
```

**PRG fix:** after a successful POST, **redirect** to a GET:

```python
def post_create(request):
    if request.method == "POST":
        form = PostForm(request.POST)
        if form.is_valid():
            post = form.save()
            return redirect("blog:detail", slug=post.slug)  # ← redirect!
    else:
        form = PostForm()
    return render(request, "blog/post_form.html", {"form": form})
```

Now F5 just re-requests the detail page (GET) — harmless.  
Always redirect after a successful POST. This is one of the most important web patterns.

---

# `form.save(commit=False)`

Sometimes you need to set extra fields before writing to the database:

```python
def post_create(request):
    if request.method == "POST":
        form = PostForm(request.POST)
        if form.is_valid():
            post = form.save(commit=False)   # creates instance, no DB write yet
            post.author = request.user       # set the logged-in user
            post.ip_address = request.META.get("REMOTE_ADDR")
            post.save()                      # now write to DB
            post.tags.set(form.cleaned_data["tags"])  # save M2M after save()
            return redirect("blog:detail", slug=post.slug)
    else:
        form = PostForm()
    return render(request, "blog/post_form.html", {"form": form})
```

Use cases for `commit=False`:
- Set `author` / `owner` from `request.user` (never expose this as a form field)
- Compute derived fields (e.g., `slug` from `title`)
- Set server-side metadata (IP, user-agent)
- Save ManyToMany relations manually after the instance has a primary key

---

# Rendering Forms in Templates

Django forms render themselves, but you can control exactly how:

```html
<!-- Automatic rendering — quick, but limited control -->
{{ form.as_p }}       <!-- wraps each field in <p> -->
{{ form.as_ul }}      <!-- wraps each field in <li> -->
{{ form.as_table }}   <!-- wraps in <tr> -->

<!-- Manual rendering — full control -->
<form method="post">
  {% csrf_token %}

  <div class="field">
    {{ form.title.label_tag }}
    {{ form.title }}
    {% if form.title.errors %}
      <ul class="errors">
        {% for error in form.title.errors %}
          <li>{{ error }}</li>
        {% endfor %}
      </ul>
    {% endif %}
    <small>{{ form.title.help_text }}</small>
  </div>

  <!-- Display non-field errors (from clean()) -->
  {% if form.non_field_errors %}
    <div class="alert">{{ form.non_field_errors }}</div>
  {% endif %}

  <button type="submit">Save</button>
</form>
```

---

# GET Forms — Search

Forms that only **read** data (search, filter) use `method="get"`.  
No CSRF token is needed — GET requests have no side effects.

```html
<form method="get" action="/posts/">
  <input type="text" name="q" value="{{ request.GET.q }}">
  <button type="submit">Search</button>
</form>
```

The search term appears in the URL: `/posts/?q=django`  
Users can bookmark and share search results.

```python
# views.py
def post_list(request):
    posts = Post.objects.filter(published=True)
    query = request.GET.get("q", "")
    if query:
        posts = posts.filter(
            Q(title__icontains=query) | Q(body__icontains=query)
        )
    return render(request, "blog/list.html", {
        "posts": posts,
        "query": query,
    })
```

Keep the `value="{{ request.GET.q }}"` on the input so the search term stays visible after submit.

---

# The N+1 Problem

Fetching a list, then hitting the DB *again* for each row = **N+1 queries**.

```python
# views.py
posts = Post.objects.filter(published=True)[:20]   # 1 query
```

```html
<!-- template -->
{% for post in posts %}
  {{ post.category.name }}   {# ← 1 extra query PER post! #}
{% endfor %}
```

With 20 posts → **21 queries**. With 1 000 posts → **1 001 queries**.  
The template looks innocent but is silently hammering the database.

---

# `select_related` — SQL JOIN

Use when traversing a **ForeignKey** or **OneToOneField** in a loop.

```python
# 1 query: SELECT post.*, category.* FROM blog_post
#          INNER JOIN blog_category ON ...
posts = Post.objects.filter(published=True).select_related("category")[:20]
```

You can chain multiple levels:

```python
# Post → Category → ParentCategory  (two JOINs, still 1 query)
Post.objects.select_related("category__parent")
```

**When to use it:**
- FK / OneToOne relationships
- You *know* you will always access the related object
- Small-to-medium number of related rows

**When NOT to use it:**
- Reverse FK (many side) — use `prefetch_related` instead
- ManyToMany — use `prefetch_related` instead

---

# `prefetch_related` — Separate Query + Python Join

Use for **reverse ForeignKey** (all comments for a post) and **ManyToMany**.

```python
# 2 queries total:
#   1) SELECT * FROM blog_post WHERE published = 1
#   2) SELECT * FROM blog_comment WHERE post_id IN (1, 2, 3, …)
#      → Django stitches results together in Python
posts = Post.objects.filter(published=True).prefetch_related("comments")[:20]
```

Access in the template — zero extra queries:

```html
{% for post in posts %}
  {% for comment in post.comments.all %}
    {{ comment.author }}: {{ comment.body }}
  {% endfor %}
{% endfor %}
```

**Custom prefetch with `Prefetch` object** — filter the related set:

```python
from django.db.models import Prefetch

active_comments = Comment.objects.filter(active=True)
posts = Post.objects.prefetch_related(
    Prefetch("comments", queryset=active_comments, to_attr="active_comments")
)
# post.active_comments ← pre-filtered list, no extra queries
```

---

# Advanced Filtering — `F`, `annotate`, `aggregate`

**`F()` expressions** — reference another column without fetching the row:

```python
from django.db.models import F

# Posts where view_count > like_count (two columns, no Python loop)
Post.objects.filter(view_count__gt=F("like_count"))

# Increment a counter atomically (avoids race conditions)
Post.objects.filter(pk=post.pk).update(view_count=F("view_count") + 1)
```

**`annotate()`** — add a computed column to each row:

```python
from django.db.models import Count

# Each category gets a post_count attribute
categories = Category.objects.annotate(post_count=Count("posts"))
# Template: {{ cat.post_count }} posts
```

**`aggregate()`** — compute a single value across the whole QuerySet:

```python
from django.db.models import Avg, Max, Sum

Post.objects.aggregate(avg_len=Avg("body_length"), latest=Max("pub_date"))
# → {"avg_len": 342.5, "latest": datetime(...)}
```

**Chaining filters** — each `filter()` call narrows the QuerySet:

```python
Post.objects.filter(published=True) \
            .filter(pub_date__year=2024) \
            .exclude(category__name="Draft") \
            .order_by("-pub_date")[:10]
```

---

# Django Debug Toolbar

The **Django Debug Toolbar** adds an in-browser panel showing:

- Number of SQL queries (and their actual SQL)
- Request / response headers
- Template rendering time and context variables
- Cache hits / misses
- Signal calls

**Install:**

```bash
uv add django-debug-toolbar
```

**Configure `settings.py`:**

```python
INSTALLED_APPS += ["debug_toolbar"]
MIDDLEWARE  = ["debug_toolbar.middleware.DebugToolbarMiddleware"] + MIDDLEWARE
INTERNAL_IPS = ["127.0.0.1"]   # toolbar only shows for these IPs
```

**Wire up `urls.py` (project-level):**

```python
from django.conf import settings

if settings.DEBUG:
    import debug_toolbar
    urlpatterns = [
        path("__debug__/", include(debug_toolbar.urls)),
    ] + urlpatterns
```

Now visit any page in dev — the toolbar appears on the right.  
Open the **SQL** panel: every query, its duration, and a full stack trace.

---

# Summary

Today we covered the full Django data layer:

| Topic | Key takeaway |
|---|---|
| **Models** | Subclass `models.Model`; each attribute is a column |
| **Field types** | `CharField`, `TextField`, `BooleanField`, `DateTimeField`, `ForeignKey` |
| **Migrations** | `makemigrations` → generates file; `migrate` → applies to DB |
| **Admin** | Register with `@admin.register`; customise with `ModelAdmin` |
| **ORM queries** | `filter()`, `exclude()`, `get()`, field lookups with `__` |
| **Q objects** | Complex OR/AND/NOT queries without raw SQL |
| **Forms** | `ModelForm` derives fields from your model automatically |
| **Validation** | `is_valid()` → `cleaned_data`; override `clean_<field>()` for custom rules |
| **CSRF** | Always add `{% csrf_token %}` to POST forms |
| **PRG** | Redirect after successful POST to prevent duplicate submissions |
| **N+1** | Use `select_related` (FK/O2O JOIN) or `prefetch_related` (reverse FK/M2M) |
| **F / annotate** | Column-level expressions and per-row computed values without Python loops |
| **aggregate** | Whole-QuerySet computation — `Count`, `Avg`, `Max`, `Sum` |
| **Debug Toolbar** | Install `django-debug-toolbar`; SQL panel shows every query + duration |

---

# Lab 5 Preview

## Build a Blog with Models, Admin & Forms

You will:

1. **Define models** — `Category` and `Post` with a `ForeignKey` relationship
2. **Run migrations** — create the schema with `makemigrations` + `migrate`
3. **Register in admin** — add `list_display`, `search_fields`, `prepopulated_fields`
4. **Write ORM queries** — list published posts, filter by category, search by keyword
5. **Create a `PostForm`** with `ModelForm`, custom slug validation, and `{% csrf_token %}`
6. **Implement PRG** — redirect to detail view after successful creation
7. **Add a search form** (GET) — filter posts by title/body without CSRF
8. **Install Django Debug Toolbar** — count queries on the list page
9. **Fix N+1** — add `select_related("category")` and `prefetch_related("comments")` to QuerySets

**Starter repo:** `labs/lab5/` in the course repository.

```bash
cd labs/lab5
uv run python manage.py migrate
uv run python manage.py createsuperuser
uv run python manage.py runserver
```

---

# Questions?

## Topics covered today

- Django models and field types
- Migrations (`makemigrations`, `migrate`, `sqlmigrate`)
- Django Admin (`@admin.register`, `ModelAdmin`)
- ORM: QuerySets, field lookups, `Q` objects, `select_related`, `prefetch_related`
- `F()` expressions, `annotate()`, `aggregate()`
- Django Debug Toolbar
- `get_object_or_404`
- `ModelForm`, `is_valid()`, `cleaned_data`, custom validators
- CSRF protection and `{% csrf_token %}`
- POST-Redirect-GET pattern
- `form.save(commit=False)`
- Rendering forms manually in templates
- GET search forms

**Next lecture:** Django — Class-Based Views, URL namespaces, and Authentication

---
{% endraw %}
