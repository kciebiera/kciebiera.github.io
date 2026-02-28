---
render_with_liquid: false
---

{% raw %}

# Lab 5: Django — Models, ORM & Forms

## Introduction

So far every piece of data in your site lives in Python code. The moment the server restarts, it is gone. Databases solve this. Django's **ORM** (Object-Relational Mapper) lets you work with a database using Python classes. And once data is stored, you need users to be able to *submit* new data — Django's form system handles validation, rendering, and security in one place.

The Goal: Define database models for a blog, query them in views, and add a comment form with server-side validation.

### The Theory

A **model** is a Python class that maps to a database table:

```
class Post(models.Model):          CREATE TABLE blog_post (
    title = CharField(max_length=200)   id      INTEGER PRIMARY KEY,
    body  = TextField()        →        title   VARCHAR(200),
    pub_date = DateTimeField()          body    TEXT,
                                        pub_date DATETIME
)                                  );
```

Every HTML form that changes data must use `method="POST"`. Django enforces **CSRF protection** by default: a hidden token in the form is checked server-side to prevent cross-site request forgery.

The **Post-Redirect-Get** (PRG) pattern prevents duplicate submissions on browser refresh:

```
Browser POST /post/slug/  → server validates
    if valid    → save, redirect
Browser GET /post/slug/   → server sends updated page
```

## Setup

Create a new app for the blog:

```bash
uv run python manage.py startapp blog
```

Register it in `settings.py`:

```python
INSTALLED_APPS = [
    ...
    'blog',
]
```

## Phase 1: Define Models & Migrations

Open `blog/models.py`:

```python
from django.db import models

class Category(models.Model):
    name = models.CharField(max_length=100)
    slug = models.SlugField(unique=True)

    def __str__(self):
        return self.name

class Post(models.Model):
    title    = models.CharField(max_length=200)
    slug     = models.SlugField(unique=True)
    body     = models.TextField()
    pub_date = models.DateTimeField(auto_now_add=True)
    # TODO: Add a ForeignKey to Category, with on_delete=models.SET_NULL,
    #       null=True, blank=True

    class Meta:
        ordering = ["-pub_date"]   # newest first

    def __str__(self):
        return self.title

class Comment(models.Model):
    post    = models.ForeignKey(Post, on_delete=models.CASCADE,
                                related_name="comments")
    author  = models.CharField(max_length=100)
    email   = models.EmailField()
    body    = models.TextField()
    created = models.DateTimeField(auto_now_add=True)
    active  = models.BooleanField(default=True)

    class Meta:
        ordering = ["created"]

    def __str__(self):
        return f"Comment by {self.author} on {self.post}"
```

Create and apply migrations:

```bash
uv run python manage.py makemigrations
uv run python manage.py migrate
```

🧪 A new file `blog/migrations/0001_initial.py` should appear. Open it — it contains the SQL-generation instructions Django derived from your model. Run `uv run python manage.py sqlmigrate blog 0001` to see the raw SQL Django generates.

## Phase 2: The Admin Panel

Django ships with a fully functional CRUD admin UI, free.

Register your models in `blog/admin.py`:

```python
from django.contrib import admin
from .models import Category, Post, Comment

@admin.register(Category)
class CategoryAdmin(admin.ModelAdmin):
    list_display = ["name", "slug"]
    prepopulated_fields = {"slug": ("name",)}

@admin.register(Post)
class PostAdmin(admin.ModelAdmin):
    list_display  = ["title", "pub_date", "category"]
    list_filter   = ["category"]
    search_fields = ["title", "body"]
    prepopulated_fields = {"slug": ("title",)}

@admin.register(Comment)
class CommentAdmin(admin.ModelAdmin):
    list_display = ["author", "post", "created", "active"]
    list_filter  = ["active"]
```

Create a superuser and add sample data:

```bash
uv run python manage.py createsuperuser
```

🧪 Visit `http://127.0.0.1:8000/admin/`. Create 2 Categories and 3 Posts (at least one per category).

## Phase 3: Views, URLs & Templates

Create `blog/views.py`:

```python
from django.shortcuts import render, get_object_or_404
from .models import Post, Category

def post_list(request):
    posts      = Post.objects.all()
    categories = Category.objects.all()
    return render(request, "blog/post_list.html", {
        "posts": posts, "categories": categories,
    })

def post_detail(request, slug):
    post = get_object_or_404(Post, slug=slug)
    # TODO: fetch active comments and an empty CommentForm (Phase 4)
    return render(request, "blog/post_detail.html", {"post": post})

def category_posts(request, slug):
    category = get_object_or_404(Category, slug=slug)
    # TODO: Filter posts by category: Post.objects.filter(category=category)
    posts = []
    return render(request, "blog/post_list.html", {
        "posts": posts,
        "categories": Category.objects.all(),
        "active_category": category,
    })
```

Create `blog/urls.py`:

```python
from django.urls import path
from . import views

urlpatterns = [
    path("",                      views.post_list,     name="post-list"),
    path("post/<slug:slug>/",     views.post_detail,   name="post-detail"),
    # TODO: path for "category/<slug:slug>/" → category_posts
]
```

Include it in `mysite/urls.py`: `path("blog/", include("blog.urls"))`.

Create `blog/templates/blog/post_list.html` extending `pages/base.html`:

```html
{% extends "pages/base.html" %}
{% block title %}Blog{% endblock %}

{% block content %}
<h1>
    {% if active_category %}Posts in: {{ active_category.name }}
    {% else %}All Posts{% endif %}
</h1>

<aside>
    <h3>Categories</h3>
    <ul>
        {% for cat in categories %}
        <li><a href="{% url 'category-posts' cat.slug %}">{{ cat.name }}</a></li>
        {% endfor %}
    </ul>
</aside>

<section>
    {% for post in posts %}
    <article>
        <h2><a href="{% url 'post-detail' post.slug %}">{{ post.title }}</a></h2>
        <small>{{ post.pub_date|date:"d M Y" }}</small>
        <p>{{ post.body|truncatewords:30 }}</p>
    </article>
    {% empty %}
    <p>No posts yet.</p>
    {% endfor %}
</section>
{% endblock %}
```

🧪 In `uv run python manage.py shell`, explore the ORM:

```python
from blog.models import Post
Post.objects.count()
Post.objects.filter(category__name="Tech")
Post.objects.filter(title__icontains="django")
```

## Phase 4: Django Forms & Validation

Create `blog/forms.py`:

```python
from django import forms
from .models import Comment

class CommentForm(forms.ModelForm):
    class Meta:
        model  = Comment
        fields = ["author", "email", "body"]
        widgets = {
            "body": forms.Textarea(attrs={"rows": 4}),
        }
        labels = {
            "author": "Your Name",
        }

    def clean_author(self):
        # TODO: Strip whitespace; raise forms.ValidationError
        # if the result is empty or shorter than 2 characters.
        author = self.cleaned_data["author"]
        return author

    def clean_body(self):
        # TODO: Raise ValidationError if body is longer than 1000 characters.
        body = self.cleaned_data["body"]
        return body
```

A `clean_<fieldname>` method runs *after* the field's built-in validation. Return the cleaned value or raise `forms.ValidationError("message")`.

Update `post_detail` in `blog/views.py`:

```python
from django.shortcuts import render, get_object_or_404, redirect
from .models import Post, Category, Comment
from .forms import CommentForm

def post_detail(request, slug):
    post     = get_object_or_404(Post, slug=slug)
    comments = post.comments.filter(active=True)
    form     = CommentForm()

    if request.method == "POST":
        form = CommentForm(request.POST)
        if form.is_valid():
            comment      = form.save(commit=False)
            comment.post = post
            comment.save()
            # TODO: Redirect to the same URL (PRG pattern):
            # return redirect("post-detail", slug=post.slug)
            pass

    return render(request, "blog/post_detail.html", {
        "post": post, "comments": comments, "form": form,
    })
```

Create `blog/templates/blog/post_detail.html`:

```html
{% extends "pages/base.html" %}
{% block title %}{{ post.title }}{% endblock %}

{% block content %}
<article>
    <h1>{{ post.title }}</h1>
    <small>{{ post.pub_date|date:"d M Y" }} — {{ post.category.name }}</small>
    <div>{{ post.body|linebreaks }}</div>
</article>

<section id="comments">
    <h2>{{ comments.count }} comment{{ comments.count|pluralize }}</h2>
    {% for comment in comments %}
    <div class="comment">
        <strong>{{ comment.author }}</strong>
        <small>{{ comment.created|date:"d M Y H:i" }}</small>
        <p>{{ comment.body }}</p>
    </div>
    {% empty %}
    <p>Be the first to comment.</p>
    {% endfor %}
</section>

<section id="add-comment">
    <h2>Leave a comment</h2>
    <form method="POST">
        {% csrf_token %}
        {% for field in form %}
        <div>
            {{ field.label_tag }}
            {{ field }}
            {% if field.errors %}
            <ul class="errorlist">
                {% for error in field.errors %}<li>{{ error }}</li>{% endfor %}
            </ul>
            {% endif %}
        </div>
        {% endfor %}
        <button type="submit">Post Comment</button>
    </form>
</section>

<p><a href="{% url 'post-list' %}">← All Posts</a></p>
{% endblock %}
```

🧪 Submit a comment with valid data — it should appear. Try submitting with an empty author — you should see your validation error message.

## Submission

Final checks:

1. `/blog/` lists all posts with category filter links.
2. `/blog/post/<slug>/` shows a full post with its comments.
3. Submitting a valid comment saves it and redirects (refresh doesn't double-submit).
4. Submitting an invalid comment re-renders the form with inline error messages.
5. The `{% csrf_token %}` tag is present in the POST form.

**Exploration:** Add a search view using `Q` objects:

```python
from django.db.models import Q

def search(request):
    query   = request.GET.get("q", "")
    results = Post.objects.filter(
        Q(title__icontains=query) | Q(body__icontains=query)
    ) if query else []
    return render(request, "blog/search.html", {"results": results, "query": query})
```

A search form uses `method="GET"` — no CSRF token needed, and the query appears in the URL so the page is bookmarkable.


{% endraw %}
