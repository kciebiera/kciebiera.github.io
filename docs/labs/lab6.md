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

## Phase 6: GitHub OAuth2 with django-allauth

So far users register with a username and password you manage. **OAuth2** lets you delegate identity verification to GitHub — users prove who they are to GitHub, and GitHub tells you. You never handle their GitHub password.

### Install

```bash
uv add django-allauth
```

### Configure `settings.py`

```python
INSTALLED_APPS = [
    ...
    "django.contrib.sites",
    "allauth",
    "allauth.account",
    "allauth.socialaccount",
    "allauth.socialaccount.providers.github",
]

SITE_ID = 1

AUTHENTICATION_BACKENDS = [
    "django.contrib.auth.backends.ModelBackend",          # username/password
    "allauth.account.auth_backends.AuthenticationBackend", # social login
]

LOGIN_REDIRECT_URL  = "/"
LOGOUT_REDIRECT_URL = "/"

SOCIALACCOUNT_PROVIDERS = {
    "github": {
        "APP": {
            "client_id": os.environ.get("GITHUB_CLIENT_ID", ""),
            "secret":    os.environ.get("GITHUB_CLIENT_SECRET", ""),
            "key":       "",
        },
        "SCOPE": ["read:user", "user:email"],
    }
}
```

Add `import os` at the top of `settings.py`.

### Wire up URLs

In `mysite/urls.py`, replace or supplement the `accounts/` include:

```python
path("accounts/", include("allauth.urls")),
```

> This provides `/accounts/login/`, `/accounts/logout/`, and the social callback `/accounts/github/login/callback/` automatically.

Apply the new migrations:

```bash
uv run python manage.py migrate
```

### Register a GitHub OAuth App

1. Go to **GitHub → Settings → Developer Settings → OAuth Apps → New OAuth App**
2. Fill in:

| Field | Value |
|-------|-------|
| Application name | My Django Blog (dev) |
| Homepage URL | `http://127.0.0.1:8000` |
| Authorization callback URL | `http://127.0.0.1:8000/accounts/github/login/callback/` |

3. Click **Register application**, then copy the **Client ID** and click **Generate a new client secret**.

### Provide credentials

Create a `.env` file in your project root (add it to `.gitignore`):

```bash
GITHUB_CLIENT_ID=Ov23liXXXXXXXXXXXXXX
GITHUB_CLIENT_SECRET=xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

Load it when running the server:

```bash
GITHUB_CLIENT_ID=… GITHUB_CLIENT_SECRET=… uv run python manage.py runserver
```

Or install `python-dotenv` (`uv add python-dotenv`) and add to the top of `manage.py`:

```python
from dotenv import load_dotenv
load_dotenv()
```

### Add the Login button to your template

In `pages/templates/pages/base.html`, add alongside your existing login link:

```html
{% load socialaccount %}
{% if not user.is_authenticated %}
    <a href="{% url 'account_login' %}">Login</a>
    <a href="{% url 'register' %}">Register</a>
    <a href="{% provider_login_url 'github' %}">Login with GitHub</a>
{% endif %}
```

🧪 Visit `http://127.0.0.1:8000/accounts/github/login/` — you should be redirected to GitHub's authorization page. After approving, you land back on your site as a logged-in user.

### What allauth does for you

| Step | Manual | allauth |
|------|--------|---------|
| Redirect to GitHub | You write | Automatic |
| Handle `?code=` callback | You write | Automatic |
| Exchange code for access token | You write | Automatic |
| Fetch user from `api.github.com/user` | You write | Automatic |
| Create/find Django `User` | You write | Automatic |
| Call `login()` | You write | Automatic |

🧪 After logging in with GitHub, run:

```bash
uv run python manage.py shell
```

```python
from allauth.socialaccount.models import SocialAccount
SocialAccount.objects.all().values("provider", "uid", "user__username")
```

You should see an entry for `provider="github"` with your GitHub username.



## Submission

Final checks:

1. Registration creates a user, logs them in, and redirects.
2. The nav dynamically shows Login/Register vs. Hello/Logout.
3. `/accounts/profile/` is only accessible when logged in.
4. Clicking "Login with GitHub" redirects to GitHub and returns you as a logged-in user.
5. In `uv run python manage.py shell`, verify passwords are hashed:

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
