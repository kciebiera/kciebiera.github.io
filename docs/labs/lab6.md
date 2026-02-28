---
render_with_liquid: false
---

{% raw %}

# Lab 6: Django Authentication — Users, Login, and Permissions

## Introduction

Anyone on the internet can currently post comments and read all content. Production apps need **authentication** (who are you?) and **authorisation** (what are you allowed to do?). Django ships a complete auth system — user accounts, sessions, password hashing, and permissions — all ready to use.

The Goal: Restrict post creation to logged-in users. Implement registration, login, logout, and a user profile page. Understand how sessions work under the hood.

### The Theory

When a user logs in, Django:

1. Verifies username + password against the hashed value in the database.
2. Creates a **session** — a random key stored server-side (in the database by default).
3. Sends a **cookie** (`sessionid`) to the browser.

On every subsequent request, the browser sends the cookie → Django looks up the session → Django knows who the user is. This is `request.user`.

## Setup

Django's auth system is already installed (`django.contrib.auth` in `INSTALLED_APPS`). Ensure `django.contrib.sessions.middleware.SessionMiddleware` is in `MIDDLEWARE` (it is by default).

Create a new `accounts` app:

```bash
uv run python manage.py startapp accounts
```

Register it in `settings.py` and add two URL settings:

```python
LOGIN_URL          = "/accounts/login/"
LOGIN_REDIRECT_URL = "/"
LOGOUT_REDIRECT_URL = "/"
```

## Phase 1: Registration

Django provides a `User` model. You only need a form for new users.

Create `accounts/forms.py`:

```python
from django import forms
from django.contrib.auth.models import User

class RegisterForm(forms.Form):
    username  = forms.CharField(max_length=150)
    email     = forms.EmailField()
    password1 = forms.CharField(widget=forms.PasswordInput, label="Password")
    password2 = forms.CharField(widget=forms.PasswordInput, label="Confirm Password")

    def clean_username(self):
        username = self.cleaned_data["username"]
        # TODO: Raise ValidationError if User.objects.filter(username=username).exists()
        return username

    def clean(self):
        cleaned = super().clean()
        p1 = cleaned.get("password1")
        p2 = cleaned.get("password2")
        # TODO: Raise ValidationError if p1 != p2
        return cleaned

    def save(self):
        data = self.cleaned_data
        return User.objects.create_user(
            username=data["username"],
            email=data["email"],
            password=data["password1"],
        )
```

Create `accounts/views.py`:

```python
from django.shortcuts import render, redirect
from django.contrib.auth import login
from .forms import RegisterForm

def register(request):
    form = RegisterForm()
    if request.method == "POST":
        form = RegisterForm(request.POST)
        if form.is_valid():
            user = form.save()
            login(request, user)         # log in immediately after registration
            return redirect("home")
    return render(request, "accounts/register.html", {"form": form})
```

Create `accounts/templates/accounts/register.html` extending `base.html` with the form and `{% csrf_token %}`.

## Phase 2: Login and Logout

Django provides built-in `LoginView` and `LogoutView`. You only need to supply templates.

Create `accounts/urls.py`:

```python
from django.urls import path
from django.contrib.auth import views as auth_views
from . import views

urlpatterns = [
    path("register/", views.register,                                       name="register"),
    path("login/",    auth_views.LoginView.as_view(
                          template_name="accounts/login.html"),             name="login"),
    path("logout/",   auth_views.LogoutView.as_view(),                      name="logout"),
    path("profile/",  views.profile,                                        name="profile"),
]
```

Include in `mysite/urls.py`: `path("accounts/", include("accounts.urls"))`.

Create `accounts/templates/accounts/login.html`:

```html
{% extends "pages/base.html" %}
{% block title %}Login{% endblock %}
{% block content %}
<h1>Login</h1>
<form method="POST">
    {% csrf_token %}
    {{ form.as_p }}
    <button type="submit">Log In</button>
</form>
<p>No account? <a href="{% url 'register' %}">Register</a></p>
{% endblock %}
```

`{{ form.as_p }}` renders each field wrapped in a `<p>` — quick and functional.

## Phase 3: Protecting Views with @login_required

Add a `profile` view in `accounts/views.py`:

```python
from django.contrib.auth.decorators import login_required

@login_required
def profile(request):
    comment_count = request.user.comment_set.count() \
        if hasattr(request.user, "comment_set") else 0
    return render(request, "accounts/profile.html", {
        "user": request.user,
        "comment_count": comment_count,
    })
```

`@login_required` redirects unauthenticated users to `LOGIN_URL` automatically.

Create `accounts/templates/accounts/profile.html` displaying:
- `user.username`, `user.email`
- `user.date_joined|date:"d M Y"`
- The comment count

🧪 Log out, then visit `/accounts/profile/` directly. You should be redirected to the login page. After login, you return to `/accounts/profile/`.

## Phase 4: Showing User State in the Base Template

Update `pages/templates/pages/base.html` nav to show login state:

```html
<nav>
    <a href="{% url 'home' %}">Home</a>
    <a href="{% url 'blog:post-list' %}">Blog</a>

    {% if user.is_authenticated %}
        <span>Hello, {{ user.username }}</span>
        <a href="{% url 'profile' %}">Profile</a>
        <form method="POST" action="{% url 'logout' %}" style="display:inline">
            {% csrf_token %}
            <button type="submit">Logout</button>
        </form>
    {% else %}
        <a href="{% url 'login' %}">Login</a>
        <a href="{% url 'register' %}">Register</a>
    {% endif %}
</nav>
```

`user` is available in every template automatically via Django's context processor.

## Phase 5: Restricting Blog Post Creation

Add a view that allows only staff users to create posts from the browser:

```python
# blog/views.py
from django.contrib.auth.decorators import login_required, user_passes_test

@login_required
@user_passes_test(lambda u: u.is_staff)
def post_create(request):
    # TODO: build a PostForm (ModelForm for Post with fields title, slug, body, category)
    # Handle GET (empty form) and POST (save + redirect to post_detail)
    pass
```

🧪 Log in as a regular user and try visiting `/blog/new/`. You should get a redirect to the login page (or a 403 if you customise `raise_exception=True` on `user_passes_test`).

## Submission

Final checks:

1. Registration creates a user, logs them in, and redirects.
2. The nav dynamically shows Login/Register vs. Hello/Logout.
3. `/accounts/profile/` is only accessible when logged in.
4. In `uv run python manage.py shell`, verify passwords are hashed:

```python
from django.contrib.auth.models import User
u = User.objects.first()
print(u.password)   # starts with pbkdf2_sha256$ or argon2$
```

**Exploration:** Open the browser's DevTools → Application → Cookies. Find the `sessionid` cookie. Copy its value and look it up in the database:

```bash
uv run python manage.py shell
from django.contrib.sessions.models import Session
Session.objects.get(session_key="<your_value>").get_decoded()
```


{% endraw %}
