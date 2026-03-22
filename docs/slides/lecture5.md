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

# Lecture 5

## Django — Models, ORM & Forms

**WWW 25/26**
ORM · Migrations · Admin · ModelForms · Validation · CSRF

---

# Why Databases?

## The Persistence Problem

Every Python variable lives in **RAM**.  
Server restart, crash, redeploy → it is **gone**.

```python
# views.py — DO NOT do this
posts = []   # ← wiped on every restart

def create_post(request):
    posts.append({"title": request.POST["title"]})
```

You already know the solution: **a relational database**.  
PostgreSQL in production, SQLite in development (Django switches with one setting change).

Django handles the connection, query building, and schema management for you — but you need to understand what it does under the hood, because the ORM can hide expensive queries.

---

# The Object-Relational Impedance Mismatch

Relational databases store data as **flat rows in tables**. Programs work with **graphs of objects** — with behaviour, references, and inheritance. These two models do not map cleanly onto each other. This tension has a name: the **object-relational impedance mismatch**.

| Relational world | Object-oriented world |
|---|---|
| Table (fixed schema) | Class (with methods, inheritance) |
| Row | Object instance |
| Foreign key (integer) | Direct object reference |
| `NULL` | `None` / absent attribute |
| No identity beyond primary key | Object identity (`is`) |

---

# Bridging the Gap — Three Strategies

1. **Raw SQL** — full control, all boilerplate on you
2. **Query builder** — compose SQL in Python, still think in tables
3. **Full ORM** — think in objects; hidden SQL, hidden cost

Django gives you the full ORM. Understanding the mismatch explains *why* the ORM makes the design choices it does.

---

# What is an ORM? — Python ↔ SQL Mapping

You already write SQL. The ORM lets you express the same operations as Python objects:

| SQL concept | Django ORM |
|---|---|
| `CREATE TABLE blog_post (…)` | `class Post(models.Model): …` |
| column | class attribute (`title = models.CharField(…)`) |
| row | model instance (`post = Post(title="Hello")`) |
| `INSERT INTO …` | `post.save()` (on a new instance) |
| `UPDATE … SET …` | `post.save()` (on an existing instance) |
| `DELETE FROM … WHERE id=1` | `post.delete()` |
| `SELECT … WHERE …` | `Post.objects.filter(…)` |

---

# What is an ORM? — Why bother / the tradeoff

**Why bother?** You get type safety, Python-level validation, automatic escaping (no SQL injection), and backend portability — swap SQLite for PostgreSQL by changing one setting.

**The tradeoff:** the ORM hides what SQL is actually running.  
A single innocent-looking line of Python can fire dozens of queries.  
This is why **Django Debug Toolbar** exists — you need visibility into what the ORM actually does.

---

# Defining a Model — Python → SQL

```python
from django.db import models

class Post(models.Model):
    title      = models.CharField(max_length=200)   # VARCHAR(200) NOT NULL
    slug       = models.SlugField(unique=True)       # VARCHAR(50) NOT NULL UNIQUE
    body       = models.TextField()                  # TEXT NOT NULL
    created_at = models.DateTimeField(auto_now_add=True)  # TIMESTAMP, set on INSERT
    updated_at = models.DateTimeField(auto_now=True)      # TIMESTAMP, set on every UPDATE
    published  = models.BooleanField(default=False)  # BOOLEAN NOT NULL DEFAULT false

    def __str__(self):       # controls how the object prints (admin, shell, FK dropdowns)
        return self.title
```

Always define `__str__` — without it, the admin and shell show `Post object (1)` everywhere.

---

# Django field → SQL type cheatsheet *(reference)*

Each field encodes a **database constraint in Python code** — the interesting design decision is that schema lives alongside application logic, not in separate SQL scripts.

| Django field | SQL type | Notes |
|---|---|---|
| `CharField(max_length=N)` | `VARCHAR(N) NOT NULL` | `max_length` required |
| `TextField()` | `TEXT NOT NULL` | no length limit |
| `IntegerField()` | `INTEGER NOT NULL` | |
| `BooleanField()` | `BOOLEAN NOT NULL` | |
| `DateTimeField()` | `TIMESTAMP NOT NULL` | |
| `SlugField()` | `VARCHAR(50) NOT NULL` | URL-safe subset of CharField |

`null=True` → allows `NULL`. `blank=True` → allows empty in **form** validation (not the DB).  
`unique=True` → `UNIQUE` constraint. `db_index=True` → `CREATE INDEX`.

---

# Relationships — ForeignKey

A `ForeignKey` = a foreign key column + an `ON DELETE` rule.

```python
class Post(models.Model):
    category = models.ForeignKey(
        Category,
        on_delete=models.CASCADE,   # ON DELETE CASCADE
        related_name="posts",       # reverse accessor: category.posts.all()
    )
```

Django adds a `category_id INTEGER` column to `blog_post` automatically.

`on_delete` maps directly to SQL referential actions:

| Django | SQL | What happens when Category row is deleted |
|---|---|---|
| `CASCADE` | `ON DELETE CASCADE` | Also delete all related Posts |
| `SET_NULL` | `ON DELETE SET NULL` | Set `category_id` to `NULL` (requires `null=True`) |
| `PROTECT` | *(enforced in Python)* | Raise an error — prevents deletion |
| `SET_DEFAULT` | `ON DELETE SET DEFAULT` | Set to the field's `default` value |

---

# Reverse access on ForeignKey

**Reverse access** — no extra query syntax needed:

```python
category = Category.objects.get(slug="python")
category.posts.all()            # SELECT * FROM blog_post WHERE category_id = …
category.posts.filter(published=True)
```

---

# Model Housekeeping — `Meta` and `__str__`

**`Meta`** controls table-level behaviour — things you'd write in SQL DDL or index definitions:

```python
class Post(models.Model):
    title     = models.CharField(max_length=200)
    slug      = models.SlugField(unique=True)
    created   = models.DateTimeField(auto_now_add=True)

    class Meta:
        ordering  = ["-created"]    # DEFAULT ORDER BY created DESC
        db_table  = "blog_post"     # explicit table name (default: appname_modelname)
        indexes   = [models.Index(fields=["slug"])]  # CREATE INDEX

    def __str__(self):
        return self.title           # shown in admin, shell, FK dropdowns
```

`ordering` sets the default `ORDER BY` for every `Post.objects.all()` call — no need to add `.order_by()` everywhere. Prefix `-` for `DESC`.

`verbose_name` / `verbose_name_plural` control the labels shown in the Django admin UI.

---

# Migrations — Version Control for Schema

Your Python code and your database schema are **two separate things**.  
If you add a `slug` column to a model but don't add it to the database, you get a runtime error.  
In a team, every developer and every production server must apply the same changes in the same order.

**Migrations are to your schema what git commits are to your code.**

This is easy in development. The hard problem is **live systems**: a production database with real data and active users cannot be taken offline to apply schema changes. Zero-downtime migrations require backward-compatible steps — add a column as nullable first, backfill data, then add the constraint. Django migrations give you the *mechanism*; knowing when to split a change into multiple safe steps requires the *judgement*.

```
blog/migrations/
  0001_initial.py        ← CREATE TABLE blog_post (…)
  0002_post_add_slug.py  ← ALTER TABLE blog_post ADD COLUMN slug VARCHAR(50)
  0003_category.py       ← CREATE TABLE blog_category (…); ALTER TABLE blog_post ADD …
```

- Commit migration files to git alongside your code changes
- `git clone` + `migrate` → every developer gets an identical schema
- Production deploy runs `migrate` → database catches up automatically

---

# Migrations — Commands & Workflow

Two commands:

```bash
python manage.py makemigrations   # inspect models → generate migration file
python manage.py migrate          # apply unapplied migrations to the database
```

**Workflow:** edit `models.py` → `makemigrations` → commit → `migrate`

Django records applied migrations in a `django_migrations` table.

---

# `sqlmigrate` — Read the Generated SQL

Since you know SQL, use `sqlmigrate` to verify what Django will actually run:

```bash
python manage.py sqlmigrate blog 0001
```

```sql
BEGIN;
CREATE TABLE "blog_post" (
    "id"         integer NOT NULL PRIMARY KEY AUTOINCREMENT,
    "title"      varchar(200) NOT NULL,
    "slug"        varchar(50) NOT NULL UNIQUE,
    "body"       text NOT NULL,
    "published"  bool NOT NULL,
    "created_at" datetime NOT NULL
);
COMMIT;
```

This is **read-only** — shows what would run without touching the database.

Use it to:

- Confirm that `unique=True` → `UNIQUE`, `db_index=True` → `CREATE INDEX`
- Understand the exact column types Django picks for your backend
- Catch unexpected schema changes before applying them to production

---

# The Django Admin

**Idea:** given a model definition, Django can *introspect* the schema at runtime and generate a fully functional CRUD interface automatically — no template writing, no boilerplate views. This is the same principle behind Rails scaffolding, Swagger UI from an OpenAPI spec, or any system that generates UI from a data model.

The admin is not magic — it is a practical demonstration that **a well-defined schema is itself a specification** for what a CRUD interface should look like. For developers and editors, not end users.

Register your models, then visit `http://127.0.0.1:8000/admin/`:

```python
# blog/admin.py
from django.contrib import admin
from .models import Post, Category

@admin.register(Post)
class PostAdmin(admin.ModelAdmin):
    list_display  = ["title", "category", "published", "created_at"]
    list_filter   = ["published", "category"]
    search_fields = ["title", "body"]
    prepopulated_fields = {"slug": ("title",)}   # JS auto-fills slug from title
    date_hierarchy = "created_at"
```

| Option | What it does |
|---|---|
| `list_display` | Columns in the list view |
| `list_filter` | Right-side filter panel |
| `search_fields` | Search box (generates `WHERE … LIKE …`) |
| `prepopulated_fields` | Auto-fills slug from title via JavaScript |

---

# The Django Admin — Superuser

Create the required superuser account:

```bash
python manage.py createsuperuser
# Username: admin  /  Password: ********
```

Superusers bypass all permission checks — never use one as the main application user in production.

---

# The ORM — SQL in Python

Every model has a **manager** (`Model.objects`) that is the entry point for queries.  
Each ORM call maps to SQL you already know:

```python
# SELECT * FROM blog_post
Post.objects.all()

# SELECT * FROM blog_post WHERE published = true
Post.objects.filter(published=True)

# SELECT * FROM blog_post WHERE NOT published = true
Post.objects.exclude(published=True)

# SELECT * FROM blog_post WHERE id = 42   (raises if 0 or 2+ rows)
Post.objects.get(id=42)

# SELECT COUNT(*) FROM blog_post WHERE published = true
Post.objects.filter(published=True).count()

# SELECT * FROM blog_post WHERE published = true LIMIT 5
Post.objects.filter(published=True)[:5]

# SELECT * FROM blog_post WHERE published = true LIMIT 10 OFFSET 10
Post.objects.filter(published=True)[10:20]

# SELECT EXISTS(SELECT 1 FROM blog_post WHERE slug = 'hello')
Post.objects.filter(slug="hello").exists()
```

---

# The ORM — `get()` and Custom Managers

`get()` raises `DoesNotExist` if no row, `MultipleObjectsReturned` if several.  
For user-facing views, use `get_object_or_404()` instead (covered shortly).

You can also define **custom managers** to encapsulate common query patterns:

```python
class PublishedManager(models.Manager):
    def get_queryset(self):
        return super().get_queryset().filter(published=True)

class Post(models.Model):
    objects   = models.Manager()      # default
    published = PublishedManager()    # Post.published.all() → only published
```

---

# QuerySets — Lazy Evaluation

**Lazy (deferred) evaluation** is a general CS strategy: build a *description* of a computation, then execute it exactly once when the result is actually needed. Haskell is lazy by default. Python generators are lazy. SQL query builders are lazy. The benefit is the same in each case: you can compose and transform the description before paying the execution cost.

A `QuerySet` is **lazy** — it does not hit the database until you actually need the data.  
This lets you build up a query across multiple lines and execute it exactly once.

```python
qs = Post.objects.filter(published=True)   # no SQL yet
qs = qs.order_by("-created_at")            # still no SQL  (ORDER BY)
qs = qs.exclude(title__startswith="Draft") # still no SQL  (AND NOT)

# SQL fires here, when the QuerySet is evaluated:
for post in qs:     # iteration
list(qs)            # explicit conversion
qs[0]               # index / slice
len(qs)             # len()
bool(qs)            # truthiness check
```

Practical example — conditionally narrow the query in a view:

```python
def post_list(request):
    qs = Post.objects.filter(published=True)
    if request.GET.get("q"):
        qs = qs.filter(title__icontains=request.GET["q"])
    qs = qs.order_by("-created_at")[:10]
    return render(request, "blog/list.html", {"posts": qs})
```

Django composes all the chained calls into **one** SQL query — not one per `.filter()` call.

---

# Field Lookups — WHERE Operators

Django appends SQL operators to field names with `__` (double underscore):

| Django `filter(field__lookup=val)` | SQL equivalent |
|---|---|
| `title__exact="Hello"` | `WHERE title = 'Hello'` |
| `title__iexact="hello"` | `WHERE LOWER(title) = 'hello'` |
| `body__contains="Django"` | `WHERE body LIKE '%Django%'` |
| `body__icontains="django"` | `WHERE LOWER(body) LIKE '%django%'` |
| `views__gt=100` | `WHERE views > 100` |
| `views__gte=100` | `WHERE views >= 100` |
| `views__lt=10` / `__lte` | `WHERE views < 10` / `<= 10` |
| `id__in=[1, 2, 3]` | `WHERE id IN (1, 2, 3)` |
| `subtitle__isnull=True` | `WHERE subtitle IS NULL` |
| `slug__startswith="2024-"` | `WHERE slug LIKE '2024-%'` |
| `created__year=2025` | `WHERE EXTRACT(year FROM created) = 2025` |

---

# Field Lookups — FK Traversal

The `__` syntax also **traverses FK relationships** — Django generates the `JOIN` for you:

```python
# WHERE blog_category.name = 'Python'  (implicit JOIN)
Post.objects.filter(category__name="Python")

# Traverse multiple levels (two JOINs, one query)
Post.objects.filter(category__parent__slug="tech")
```

SQL generated for the first example:

```sql
SELECT blog_post.*
FROM   blog_post
INNER JOIN blog_category ON blog_post.category_id = blog_category.id
WHERE  blog_category.name = 'Python'
```

---

# Related Object Queries — Reverse Relation

**Reverse relation** — from the "one" side to the "many" side:

```python
category = Category.objects.get(slug="python")
category.posts.all()                  # SELECT * FROM blog_post WHERE category_id = …
category.posts.filter(published=True)
category.posts.count()
```

**Warning:** accessing a FK attribute on a single instance fires an extra query:

```python
post = Post.objects.get(id=1)
print(post.category.name)   # ← 1 extra SELECT for category!
```

This is the seed of the N+1 problem — covered shortly.

---

# Q Objects — OR / AND / NOT

`filter()` keyword arguments are always `AND`. For SQL `OR` or `NOT`, use `Q` objects:

```python
from django.db.models import Q

# WHERE published = true OR EXTRACT(year FROM created_at) = 2025
Post.objects.filter(
    Q(published=True) | Q(created_at__year=2025)
)

# WHERE NOT published = true   (same as .exclude(published=True))
Post.objects.filter(~Q(published=True))

# WHERE (title ILIKE '%django%' OR body ILIKE '%django%') AND published = true
Post.objects.filter(
    Q(title__icontains="django") | Q(body__icontains="django"),
    published=True,       # keyword args are AND-ed with Q objects
)
```

---

# Q Objects — Operator Reference

`Q` operators mirror SQL boolean operators:

| Q operator | SQL |
|---|---|
| `Q(a) \| Q(b)` | `a OR b` |
| `Q(a) & Q(b)` | `a AND b` *(same as two keyword args)* |
| `~Q(a)` | `NOT a` |

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

# The N+1 Problem — A Query Complexity Bug

Consider rendering a list of 20 posts, each with its category name:

```python
posts = Post.objects.filter(published=True)[:20]  # 1 query
for post in posts:
    print(post.category.name)   # 1 extra query per post → 20 more queries
```

That's **21 queries** to display 20 posts. In general, N rows → N+1 queries.

| Posts shown | Queries fired |
|---|---|
| 20 | 21 |
| 100 | 101 |
| 1 000 | 1 001 |

Each query has network round-trip overhead. This is an asymptotic problem: performance degrades *linearly with data size* where it should be constant. The ORM's laziness makes it easy to write N+1 bugs accidentally — you write one line, but it triggers a loop of hidden queries.

**The fix:** tell the ORM to fetch related data upfront in *one* query, not lazily one-by-one.

---

# `select_related` — INNER JOIN

`select_related` tells the ORM to do a JOIN and fetch the related row in the same query.

```python
# Before: 1 + N queries
posts = Post.objects.filter(published=True)[:20]

# After: 1 query with INNER JOIN
posts = Post.objects.filter(published=True).select_related("category")[:20]
```

Generated SQL:

```sql
SELECT blog_post.*, blog_category.*
FROM   blog_post
INNER  JOIN blog_category ON blog_post.category_id = blog_category.id
WHERE  blog_post.published = true
LIMIT  20
```

Chain multiple levels (multiple JOINs, still one query):

```python
Post.objects.select_related("category__parent")
```

**Use for:** FK / OneToOne — you always know the related side has exactly one row.  
**Do not use for:** reverse FK or ManyToMany — a JOIN multiplies rows. Use `prefetch_related` instead.

---

# `prefetch_related` — Two Queries, Python Join

For **reverse FK** (all comments for a post) and **ManyToMany**, a JOIN would multiply rows.  
`prefetch_related` fires a *separate* query and stitches results in Python:

```python
# Query 1: SELECT * FROM blog_post WHERE published = true
# Query 2: SELECT * FROM blog_comment WHERE post_id IN (1, 2, 3, …)
# Python:  attach comments to each post object
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

---

# `prefetch_related` — Custom Prefetch

**Custom prefetch** — filter the related set (e.g. only active comments):

```python
from django.db.models import Prefetch

active = Comment.objects.filter(active=True)
posts  = Post.objects.prefetch_related(
    Prefetch("comments", queryset=active, to_attr="active_comments")
)
# post.active_comments ← pre-filtered list, no extra queries
```

---

# `F()` Expressions

**Why not just `post.views += 1; post.save()`?**

This is a classic **read-modify-write race condition**:

```
Request A reads:  views = 100
Request B reads:  views = 100
Request A writes: views = 101
Request B writes: views = 101   ← one increment lost
```

Two concurrent requests both read the same stale value, compute the same result, and one update overwrites the other. The fix is to move the computation *into the database*, where it executes atomically without a round-trip to Python.

**`F()` expressions** reference a column directly in a SQL expression:

```python
from django.db.models import F

# WHERE view_count > like_count   (compare two columns — no Python round-trip)
Post.objects.filter(view_count__gt=F("like_count"))

# UPDATE blog_post SET view_count = view_count + 1 WHERE id = ?
# Atomic — no race condition, no round-trip to Python
Post.objects.filter(pk=post.pk).update(view_count=F("view_count") + 1)
```

---

# `annotate()` — Per-Row Aggregation

**`annotate()`** — SQL `GROUP BY` equivalent: add a computed column to each row:

```python
from django.db.models import Count

# SELECT blog_category.*, COUNT(blog_post.id) AS post_count
# FROM blog_category LEFT JOIN blog_post ON …
# GROUP BY blog_category.id
categories = Category.objects.annotate(post_count=Count("posts"))
# Template: {{ cat.post_count }} posts
```

---

# `aggregate()` — Whole-QuerySet Aggregation

**`aggregate()`** returns a single dict across the whole QuerySet (no `GROUP BY`):

```python
from django.db.models import Avg, Max, Count

Post.objects.filter(published=True).aggregate(
    total=Count("id"),
    avg_views=Avg("view_count"),
    latest=Max("pub_date"),
)
# → {"total": 42, "avg_views": 1250.3, "latest": datetime(…)}
```

**Rule of thumb:** `annotate` = per-row column added to each object. `aggregate` = one dict for the whole set.

---

# Django Debug Toolbar

An in-browser panel that shows SQL queries, timings, templates, cache hits, and more.

```bash
uv add django-debug-toolbar
```

```python
# settings.py
INSTALLED_APPS += ["debug_toolbar"]
MIDDLEWARE = ["debug_toolbar.middleware.DebugToolbarMiddleware"] + MIDDLEWARE
INTERNAL_IPS = ["127.0.0.1"]
```

```python
# urls.py (project-level)
if settings.DEBUG:
    urlpatterns = [path("__debug__/", include("debug_toolbar.urls"))] + urlpatterns
```

Visit any page in dev — the toolbar appears on the right. Open the **SQL** panel to see every query and its duration.

---

# What is a Django Form? — Trust Boundaries

`request.POST` is a dictionary of **strings from the internet**.  
Anyone can send anything — empty strings, SQL fragments, absurdly long text, wrong types.

`form.cleaned_data` is validated **Python data** you can trust.

```
request.POST["author"]  →  "'; DROP TABLE blog_post; --"   ← untrusted string
form.cleaned_data["author"]  →  "Alice"                    ← validated, type-coerced
```

**Never** use raw `request.POST` values directly in your views or models.  
Always pass input through a form first.

A **Form** is a Python class that:

1. Describes which fields to render and validate
2. Accepts user input (from `request.POST`)
3. Validates that input and returns clean Python values

---

# What is a Django Form? — `Form` vs `ModelForm`

**`Form`** — build fields manually:

```python
from django import forms

class ContactForm(forms.Form):
    name    = forms.CharField(max_length=100)
    email   = forms.EmailField()
    message = forms.CharField(widget=forms.Textarea)
```

**`ModelForm`** — derive fields from a model:

```python
from django import forms
from .models import Post

class PostForm(forms.ModelForm):
    class Meta:
        model  = Post
        fields = ["title", "body", "published"]
```

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
        fields  = ["title", "slug", "body", "published"]
        widgets = {
            "body": forms.Textarea(attrs={"rows": 6}),
        }
```

- **`fields`** — explicit allowlist (preferred over `exclude`). Never use `"__all__"` — it exposes sensitive columns.
- **`widgets`** — override the default HTML widget for any field.
- **`labels`** / **`help_texts`** — override labels and add hint text.

---

# Form Validation — `is_valid()`

The standard form-handling pattern in a view:

```python
def post_create(request):
    if request.method == "POST":
        form = PostForm(request.POST)
        if form.is_valid():
            form.save()
            return redirect("blog:list")
    else:
        form = PostForm()
    return render(request, "blog/post_form.html", {"form": form})
```

What `is_valid()` does:

1. Runs field built-in validators (`max_length`, `required`, type coercion)
2. Runs custom `clean_<field>()` and form-wide `clean()` methods
3. On success → `form.cleaned_data`; on failure → `form.errors`

Always use `form.cleaned_data["title"]` — never raw `request.POST`.

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

# Where Validation Lives — Three Layers

**Principle: never trust that an outer layer will always run.**

A form can be bypassed by a direct API call. A view can be skipped by a management command or a background job. Only database constraints are truly inescapable. This is **defence in depth** — each layer validates independently, so a bug or shortcut in one layer does not compromise data integrity.

| Layer | Where | Runs when |
|---|---|---|
| **Database constraints** | `UNIQUE`, `NOT NULL`, `CHECK` | On every `INSERT`/`UPDATE` |
| **Model validators** | `Model.clean()`, `validators=` | When you call `model.full_clean()` |
| **Form validators** | `Form.clean()`, `clean_<field>()` | When you call `form.is_valid()` |

---

# Where Validation Lives — In Practice

- **Forms** handle the web request — run first, produce user-facing error messages.
- **Model validators** catch programmatic misuse (e.g., a background job saving invalid data).
- **Database constraints** are the safety net if both layers above are bypassed.

A `ModelForm` automatically applies the model's field-level constraints (`max_length`, `unique`) as form validation — so you rarely need to duplicate them.

---

# CSRF Protection — The Attack

You are logged in to `bank.com`. Your browser stores a session cookie.  
You visit `evil.com`. It contains a hidden form that auto-submits to `bank.com/transfer`.  
Your browser sends the request — and **attaches the session cookie automatically**.  
From `bank.com`'s perspective the request looks legitimate.

```
1. You log in to bank.com  → browser stores session cookie
2. You visit evil.com
3. evil.com's page silently sends:
     POST bank.com/transfer
     Cookie: session=abc123   ← browser adds this automatically
     amount=10000&to=attacker
4. bank.com processes it — you never clicked anything
```

**Cross-Site Request Forgery (CSRF):** the server cannot tell a legitimate form from a forged one — unless it adds a secret that `evil.com` cannot know.

---

# CSRF Protection — The Same-Origin Policy

The browser enforces a **same-origin policy (SOP)**: JavaScript on `evil.com` cannot read content — cookies, HTML, responses — from `bank.com`. This is a browser security primitive, not a Django feature.

The CSRF defence exploits the SOP: Django generates a secret token, embeds it in the HTML page, and verifies it on every state-changing request. Since `evil.com` cannot read `bank.com`'s HTML, it cannot extract the token and cannot forge a valid request.

---

# CSRF Protection — Django's Defence

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

# The POST-Redirect-GET Pattern — The Problem

PRG is not a Django concept — it applies to every web framework (Rails, Laravel, Flask, Express…).

**Root cause — HTTP semantics:** the HTTP specification distinguishes **safe** methods (GET, HEAD — no side effects) from **unsafe** methods (POST — causes a state change). It also defines **idempotent** methods (GET, PUT, DELETE — repeating them has the same effect as doing them once). POST is *neither safe nor idempotent*: two identical POST requests are supposed to produce two separate effects. The browser is therefore correct to warn before replaying a POST.

**Problem:** the browser's "last request" after a POST is the POST itself.  
Pressing F5 (refresh) replays it — duplicate records, duplicate payments, duplicate emails.

```
Without PRG:
  Browser                    Server
    │─── POST /posts/create ──▶│  Creates post, returns HTML
    │◀── 200 OK (HTML page) ───│
    │
    │  (user presses F5)
    │─── POST /posts/create ──▶│  Creates ANOTHER post  ← bug!
```

---

# The POST-Redirect-GET Pattern — The Fix

**PRG fix:** after a successful POST, respond with a **redirect**, not an HTML page.  
The browser then follows the redirect with a GET. F5 replays the GET — harmless.

```
With PRG:
  Browser                    Server
    │─── POST /posts/create ──▶│  Creates post
    │◀── 302 redirect ─────────│
    │─── GET  /posts/hello/ ──▶│  Fetches detail page
    │◀── 200 OK (HTML page) ───│
    │
    │  (user presses F5)
    │─── GET  /posts/hello/ ──▶│  Fetches same page — no duplicate
```

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

Always redirect after a successful POST. This is one of the most important patterns in web development.

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

# Summary

Today we covered the full Django data layer:

| Topic | Key takeaway |
|---|---|
| **Schema design** | One entity = one table; normalise by storing each fact once |
| **Models** | Subclass `models.Model`; each attribute is a column |
| **Field types** | `CharField`, `TextField`, `BooleanField`, `DateTimeField`, `ForeignKey` |
| **Migrations** | Version control for schema: `makemigrations` → file; `migrate` → DB |
| **Admin** | Register with `@admin.register`; customise with `ModelAdmin` |
| **ORM queries** | `filter()`, `exclude()`, `get()`, field lookups with `__` |
| **Q objects** | Complex OR/AND/NOT queries without raw SQL |
| **Forms as trust boundaries** | `request.POST` is untrusted; `cleaned_data` is validated Python |
| **Validation layers** | DB constraints → model validators → form validators |
| **ModelForm** | Derives fields from your model automatically |
| **Validation** | `is_valid()` → `cleaned_data`; override `clean_<field>()` for custom rules |
| **CSRF** | Secret token in form blocks cross-site forged POSTs; always `{% csrf_token %}` |
| **PRG** | Redirect after successful POST — universal web pattern, not Django-specific |
| **N+1** | Use `select_related` (FK/O2O JOIN) or `prefetch_related` (reverse FK/M2M) |
| **F / annotate** | Column-level expressions and per-row computed values without Python loops |
| **aggregate** | Whole-QuerySet computation — `Count`, `Avg`, `Max`, `Sum` |
| **Debug Toolbar** | Install `django-debug-toolbar`; SQL panel shows every query + duration |
