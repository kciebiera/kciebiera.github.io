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

# Lecture 6
## Django — Authentication & Authorisation

**WWW 25/26**
Sessions · Cookies · Users · Login · Permissions

---

# Two Different Questions

<div class="columns">
<div>

## Authentication
**Who are you?**

Proving identity.

- "I am alice@example.com"
- Verified by a password, token, or certificate

</div>
<div>

## Authorisation
**What can you do?**

Enforcing access control.

- "Alice may read articles"
- "Alice may NOT delete other users"

</div>
</div>

> You must authenticate before you can be authorised.
> Getting these two concepts mixed up is a classic source of security bugs.

---

# The Web Is Stateless

HTTP does **not** remember previous requests.

```
GET /dashboard HTTP/1.1
Host: mysite.com
```

Every single request arrives with zero context.
The server has no idea whether this is the same browser that sent a request one second ago.

**Why is that a problem?**

- A login page sets a password. The very next request must *somehow* prove the user already authenticated.
- Without extra mechanisms the user would have to send credentials on *every* request.

**Solutions:**
1. Send credentials every time → bad (password travels constantly)
2. Client-side tokens (e.g. JWT) → covered in later lectures
3. **Server-side sessions + cookies** → today's topic

---

# Sessions — The Big Picture

A **session** is a small piece of state the server keeps on behalf of one browser.

```
Browser                          Server
  |                                |
  |  POST /login  (user+password)  |
  |------------------------------->|
  |                                |  creates session record in DB
  |                                |  session_key = "a8f3c9..."
  |  200 OK                        |
  |  Set-Cookie: sessionid=a8f3c9  |
  |<-------------------------------|
  |                                |
  |  GET /dashboard                |
  |  Cookie: sessionid=a8f3c9      |
  |------------------------------->|
  |                                |  look up session_key in DB
  |                                |  → user_id = 42 → Alice
  |  200 OK  (Alice's dashboard)   |
  |<-------------------------------|
```

The **session key** is just a random string — it has no meaning on its own.
The sensitive data (who is logged in) stays safely on the server.

---

# The Session Table in the Database

Django stores sessions in the `django_session` table.

```sql
SELECT session_key, session_data, expire_date
FROM django_session
LIMIT 1;
```

| session_key | session_data | expire_date |
|---|---|---|
| `a8f3c9b2…` | `eyJ1c2VyX2lkIjogNDJ9…` (base64) | 2025-12-01 10:00 |

The `session_data` is a base64-encoded, HMAC-signed JSON blob.
Django verifies the signature before trusting the data.

**Django setup — `settings.py`:**
```python
INSTALLED_APPS = [
    ...
    'django.contrib.sessions',      # must be present
    ...
]
MIDDLEWARE = [
    ...
    'django.contrib.sessions.middleware.SessionMiddleware',
    ...
]
SESSION_ENGINE = 'django.contrib.sessions.backends.db'  # default
```

---

# Cookies — What Are They?

A **cookie** is a small key-value pair the server asks the browser to store and re-send.

**Server → browser (response header):**
```
Set-Cookie: sessionid=a8f3c9b2; Path=/; HttpOnly; SameSite=Lax
```

**Browser → server (every subsequent request header):**
```
Cookie: sessionid=a8f3c9b2
```

The browser sends the cookie automatically — no JavaScript needed.

**Important cookie attributes:**

| Attribute | Meaning |
|---|---|
| `HttpOnly` | JavaScript cannot read this cookie (blocks XSS theft) |
| `Secure` | Only sent over HTTPS |
| `SameSite=Lax` | Not sent on cross-site POST (blocks CSRF) |
| `Expires` / `Max-Age` | When the browser deletes it |

---

# The Login Flow — Step by Step

```
1.  User visits /login  →  GET /login
    Server returns an HTML form with a CSRF token

2.  User fills in username + password, clicks "Log in"
    POST /login   body: username=alice&password=hunter2&csrfmiddlewaretoken=XYZ

3.  Django checks the CSRF token  (middleware)

4.  Django calls authenticate(request, username='alice', password='hunter2')
       → hashes the submitted password
       → compares with the stored hash
       → returns a User object, or None

5.  If User is None → re-render form with error message

6.  If User is valid:
       login(request, user)          # creates / updates session record
       redirect(next or '/')         # send browser elsewhere

7.  Django sets  Set-Cookie: sessionid=<new key>  on the response
```

Every step matters. Skip CSRF → vulnerable. Skip hashing → catastrophic.

---

# Password Hashing — Never Store Plaintext

Storing passwords in plaintext is **the single most catastrophic mistake** in web development.
If your database leaks, every password is immediately exposed.

**What to do instead — hash + salt:**

```
stored = HASH( salt + password )
```

- **Hash:** one-way function — you cannot reverse it
- **Salt:** random bytes prepended before hashing — prevents rainbow-table attacks

**Django's default: PBKDF2-SHA256**

```
pbkdf2_sha256$720000$rAnDoMsAlT$<base64-digest>
```

- 720 000 iterations → deliberately slow to brute-force
- Each password has its own salt

**Even stronger (optional):**
```python
# settings.py
PASSWORD_HASHERS = [
    'django.contrib.auth.hashers.Argon2PasswordHasher',  # winner of Password Hashing Competition
    'django.contrib.auth.hashers.PBKDF2PasswordHasher',  # fallback
]
```

---

# `django.contrib.auth` — What Ships Out of the Box

Django's auth system is a batteries-included module.

**Add to `INSTALLED_APPS`:**
```python
INSTALLED_APPS = [
    'django.contrib.auth',
    'django.contrib.contenttypes',  # required by auth
    ...
]
```

**What you get for free:**

| Feature | Where |
|---|---|
| `User` model | `django.contrib.auth.models.User` |
| Password hashing | automatic on `create_user()` |
| Session-based login/logout | `authenticate()`, `login()`, `logout()` |
| Built-in views | `LoginView`, `LogoutView`, `PasswordChangeView`, … |
| Permissions system | `Permission`, `has_perm()`, `@permission_required` |
| Admin integration | staff/superuser flags, per-model permissions |

Run migrations to create the tables:
```bash
uv run python manage.py migrate
```

---

# The `User` Model

```python
from django.contrib.auth.models import User

u = User.objects.get(username='alice')

u.username        # 'alice'
u.email           # 'alice@example.com'
u.password        # 'pbkdf2_sha256$720000$...' (hashed!)
u.first_name      # 'Alice'
u.last_name       # 'Smith'
u.is_active       # True  — False = soft-deleted / banned
u.is_staff        # True  — can log in to /admin/
u.is_superuser    # True  — bypasses all permission checks
u.date_joined     # datetime(2025, 1, 15, 9, 30)
u.last_login      # datetime(2025, 6, 1, 14, 22)
```

**Never set `u.password` directly** — that would store a plaintext string.

Use the proper methods instead (next slide).

---

# Creating Users

<div class="columns">
<div>

**Regular user**
```python
from django.contrib.auth.models import User

user = User.objects.create_user(
    username='alice',
    email='alice@example.com',
    password='s3cr3t!',
)
# password is hashed automatically
```

**Change password later**
```python
user.set_password('newpassword123')
user.save()
# set_password hashes; save() writes to DB
```

</div>
<div>

**Superuser (from shell)**
```bash
uv run python manage.py createsuperuser
```

**Or programmatically**
```python
admin = User.objects.create_superuser(
    username='admin',
    email='admin@example.com',
    password='adminpass',
)
# is_staff=True, is_superuser=True set automatically
```

</div>
</div>

> `create_user()` and `create_superuser()` both call `set_password()` internally — that is the only safe way to set a password.

---

# `authenticate()` and `login()`

```python
from django.contrib.auth import authenticate, login
from django.shortcuts import render, redirect

def login_view(request):
    if request.method == 'POST':
        username = request.POST['username']
        password = request.POST['password']

        # authenticate() hashes the submitted password and compares
        user = authenticate(request, username=username, password=password)

        if user is not None:
            login(request, user)          # creates session, sets cookie
            next_url = request.GET.get('next', '/')
            return redirect(next_url)
        else:
            return render(request, 'login.html', {'error': 'Invalid credentials'})

    return render(request, 'login.html')
```

**`authenticate()`** returns a `User` object on success, `None` on failure.
**`login()`** writes the user's ID into the session and calls `request.session.cycle_key()` to prevent session fixation.

---

# `logout()` and Clearing the Session

```python
from django.contrib.auth import logout
from django.shortcuts import redirect

def logout_view(request):
    if request.method == 'POST':        # must be POST (see later slide)
        logout(request)                 # flushes the entire session
        return redirect('/login/')
```

**What `logout()` does internally:**

1. Calls `request.session.flush()` — deletes the session record from the DB and generates a new empty session key
2. Replaces `request.user` with `AnonymousUser`
3. The browser still has the old `sessionid` cookie, but it points to nothing

**Why `flush()` instead of just removing the user id?**

If you only removed the user id, an attacker with the cookie could wait for someone else to get assigned that session key (extremely unlikely but possible). Deleting the session record is the safe choice.

---

# `request.user`

Django's `AuthenticationMiddleware` runs on every request.
It reads the session, looks up the user, and attaches them to the request.

```python
def my_view(request):
    print(request.user)          # <User: alice> or <AnonymousUser>
    print(request.user.is_authenticated)  # True or False

    if request.user.is_authenticated:
        print(f"Hello, {request.user.username}!")
    else:
        print("You are not logged in.")
```

**`AnonymousUser`** is a sentinel object — it has:
- `is_authenticated = False`
- `is_staff = False`
- `is_superuser = False`
- No `id`, no `email`

You can always safely call `request.user.is_authenticated` without first checking whether the user is anonymous.

---

# Built-in Views — `LoginView` and `LogoutView`

Django provides class-based views so you only need to supply a template.

**`urls.py`:**
```python
from django.contrib.auth import views as auth_views

urlpatterns = [
    path('login/',  auth_views.LoginView.as_view(template_name='login.html'),  name='login'),
    path('logout/', auth_views.LogoutView.as_view(),                           name='logout'),
]
```

**`login.html` (minimal):**
```html
<form method="post">
  { % csrf_token % }
  {{ form.as_p }}
  <button type="submit">Log in</button>
</form>
```

`LoginView` handles GET (show form) and POST (validate + login) automatically.
After login it redirects to `settings.LOGIN_REDIRECT_URL` (default: `/accounts/profile/`).
After logout it redirects to `settings.LOGOUT_REDIRECT_URL`.

---

# Writing a Registration View from Scratch

<div class="columns">
<div>

**`forms.py`**
```python
from django import forms
from django.contrib.auth.models import User

class RegisterForm(forms.Form):
    username = forms.CharField(max_length=150)
    email    = forms.EmailField()
    password = forms.CharField(widget=forms.PasswordInput)
    password2 = forms.CharField(
        widget=forms.PasswordInput,
        label='Confirm password'
    )

    def clean(self):
        data = super().clean()
        if data.get('password') != data.get('password2'):
            raise forms.ValidationError('Passwords do not match.')
        if User.objects.filter(username=data.get('username')).exists():
            raise forms.ValidationError('Username already taken.')
        return data
```

</div>
<div>

**`views.py`**
```python
from django.contrib.auth import login
from django.contrib.auth.models import User
from django.shortcuts import render, redirect
from .forms import RegisterForm

def register(request):
    form = RegisterForm(request.POST or None)
    if form.is_valid():
        user = User.objects.create_user(
            username=form.cleaned_data['username'],
            email=form.cleaned_data['email'],
            password=form.cleaned_data['password'],
        )
        login(request, user)   # log in immediately
        return redirect('/')
    return render(request, 'register.html', {'form': form})
```

</div>
</div>

---

# `@login_required` — The Simplest Guard

```python
from django.contrib.auth.decorators import login_required

@login_required
def dashboard(request):
    return render(request, 'dashboard.html', {'user': request.user})
```

**What happens if the user is not authenticated?**

1. Django redirects to `settings.LOGIN_URL` (default: `/accounts/login/`)
2. It appends `?next=/dashboard/` so after login the user returns to the right page
3. The login view must honour the `next` parameter (built-in `LoginView` does this automatically)

**Custom redirect target:**
```python
@login_required(login_url='/my-custom-login/')
def secret_page(request):
    ...
```

**In `settings.py`:**
```python
LOGIN_URL = '/login/'           # where unauthenticated users are sent
```

---

# `user_passes_test` — Arbitrary Checks

When `@login_required` is not specific enough:

```python
from django.contrib.auth.decorators import user_passes_test

def is_moderator(user):
    return user.is_authenticated and user.groups.filter(name='moderators').exists()

@user_passes_test(is_moderator)
def moderate_comments(request):
    ...
```

**Raise a 403 instead of redirecting:**
```python
from django.core.exceptions import PermissionDenied

def is_adult(user):
    return user.is_authenticated and user.profile.age >= 18

@user_passes_test(is_adult, raise_exception=True)
def adult_content(request):
    ...
```

If the test returns `False` and `raise_exception=True`, Django returns a **403 Forbidden** response instead of redirecting to the login page. Useful when the user is already logged in but simply lacks the required attribute.

---

# `is_staff` and `is_superuser`

Django has two built-in boolean permission levels on the `User` model.

<div class="columns">
<div>

**`is_staff`**
- Grants access to `/admin/`
- Does **not** automatically grant any model-level permissions
- You still need to assign those explicitly in the admin UI
- Typically: site editors, content moderators

```python
@user_passes_test(lambda u: u.is_staff)
def staff_only_view(request):
    ...
```

</div>
<div>

**`is_superuser`**
- Bypasses **every** permission check
- `has_perm()` always returns `True`
- `has_module_perms()` always returns `True`
- Use only for developer / sysadmin accounts

```python
# Superuser check (rarely needed — 
# superusers pass all perm checks anyway)
@user_passes_test(lambda u: u.is_superuser)
def superuser_view(request):
    ...
```

</div>
</div>

---

# Django's Permission System

Every model automatically gets four permissions:

| Codename | Meaning |
|---|---|
| `add_<model>` | Can create new instances |
| `change_<model>` | Can update existing instances |
| `delete_<model>` | Can delete instances |
| `view_<model>` | Can read instances |

**Checking permissions:**
```python
user.has_perm('blog.add_post')      # app_label.codename
user.has_perm('blog.change_post')
user.has_perm('blog.delete_post')
```

**Assigning permissions:**
```python
from django.contrib.auth.models import Permission

perm = Permission.objects.get(codename='add_post')
user.user_permissions.add(perm)

# Or via groups:
from django.contrib.auth.models import Group
editors = Group.objects.get(name='editors')
editors.permissions.add(perm)
user.groups.add(editors)
```

---

# `@permission_required`

The easiest way to guard a view with a Django permission:

```python
from django.contrib.auth.decorators import permission_required

@permission_required('blog.add_post')
def create_post(request):
    ...

@permission_required('blog.delete_post', raise_exception=True)
def delete_post(request, pk):
    ...
```

**Custom permissions on a model:**
```python
class Post(models.Model):
    title   = models.CharField(max_length=200)
    content = models.TextField()

    class Meta:
        permissions = [
            ('publish_post', 'Can publish a post'),
            ('feature_post', 'Can mark a post as featured'),
        ]
```

After adding custom permissions, run `makemigrations` + `migrate`.

```python
@permission_required('blog.publish_post')
def publish(request, pk):
    ...
```

---

# Login State in Templates

Django passes `request.user` to every template via `django.template.context_processors.request`.

**Ensure the processor is active (`settings.py`):**
```python
TEMPLATES = [{
    ...
    'OPTIONS': {'context_processors': [
        'django.template.context_processors.request',
        'django.contrib.auth.context_processors.auth',
        ...
    ]},
}]
```

**`base.html` — navigation bar:**
```html
<nav>
  <a href="/">Home</a>

  { % if user.is_authenticated % }
    <span>Hello, {{ user.username }}!</span>
    <form method="post" action="{ % url 'logout' % }" style="display:inline">
      { % csrf_token % }
      <button type="submit">Log out</button>
    </form>
  { % else % }
    <a href="{ % url 'login' % }">Log in</a>
    <a href="{ % url 'register' % }">Register</a>
  { % endif % }
</nav>
```

---

# Why Logout Must Be POST, Not GET

**The vulnerability with GET logout:**

```html
<!-- Attacker's page embeds: -->
<img src="https://yoursite.com/logout/" width="0" height="0">
```

When a logged-in user visits the attacker's page, their browser fetches the image URL — logging them out silently. This is a **Cross-Site Request Forgery (CSRF)** attack.

**POST + CSRF token is the defence:**

```html
<form method="post" action="/logout/">
  { % csrf_token % }
  <button type="submit">Log out</button>
</form>
```

The CSRF token is a secret tied to the session; an attacker cannot forge it from another origin.

Django's `LogoutView` only accepts POST since Django 5.0.
In earlier versions you had to opt into this behaviour manually.

```python
# urls.py
path('logout/', auth_views.LogoutView.as_view(), name='logout'),
# LogoutView returns 405 Method Not Allowed on GET (Django 5+)
```

---

# Auth URL Settings

Three key settings that control where users are sent:

```python
# settings.py

LOGIN_URL = '/login/'
# Where @login_required redirects unauthenticated users.
# Default: '/accounts/login/'

LOGIN_REDIRECT_URL = '/dashboard/'
# Where LoginView sends users after a successful login
# (if no ?next= parameter is present).
# Default: '/accounts/profile/'

LOGOUT_REDIRECT_URL = '/login/'
# Where LogoutView sends users after logout.
# Default: None  (shows a "Logged out" page)
```

**The `next` parameter flow:**

```
User visits /dashboard/  (not logged in)
→ redirect to /login/?next=/dashboard/
→ user logs in
→ redirect to /dashboard/   (from ?next=)
```

Always validate the `next` URL before redirecting — never redirect to an external domain.

```python
from django.utils.http import url_has_allowed_host_and_scheme

next_url = request.GET.get('next', '/')
if not url_has_allowed_host_and_scheme(next_url, allowed_hosts={request.get_host()}):
    next_url = '/'
return redirect(next_url)
```

---

# Password Change — Built-in View

Django ships a complete password-change flow.

**`urls.py`:**
```python
from django.contrib.auth import views as auth_views

urlpatterns = [
    path('password-change/',
         auth_views.PasswordChangeView.as_view(
             template_name='password_change.html',
             success_url='/password-change/done/',
         ),
         name='password_change'),
    path('password-change/done/',
         auth_views.PasswordChangeDoneView.as_view(
             template_name='password_change_done.html',
         ),
         name='password_change_done'),
]
```

**`password_change.html`:**
```html
<form method="post">
  { % csrf_token % }
  {{ form.as_p }}
  <button type="submit">Change password</button>
</form>
```

`PasswordChangeView` requires the user to enter their **old password** — it is automatically protected by `@login_required`.

---

# Password Reset — Email Flow

For users who forgot their password.

**`urls.py`:**
```python
urlpatterns = [
    path('password-reset/',
         auth_views.PasswordResetView.as_view(template_name='pw_reset.html'),
         name='password_reset'),
    path('password-reset/done/',
         auth_views.PasswordResetDoneView.as_view(template_name='pw_reset_done.html'),
         name='password_reset_done'),
    path('reset/<uidb64>/<token>/',
         auth_views.PasswordResetConfirmView.as_view(template_name='pw_reset_confirm.html'),
         name='password_reset_confirm'),
    path('reset/done/',
         auth_views.PasswordResetCompleteView.as_view(template_name='pw_reset_complete.html'),
         name='password_reset_complete'),
]
```

**Email backend for development (`settings.py`):**
```python
EMAIL_BACKEND = 'django.core.mail.backends.console.EmailBackend'
# Prints emails to the terminal instead of sending them
```

The reset link contains a cryptographically signed token that expires after 3 days (`PASSWORD_RESET_TIMEOUT`).

---

# Custom User Model — The Right Way to Extend

Django's built-in `User` is fine for many projects, but often you need extra fields (e.g., `bio`, `avatar`).

**Option 1 — OneToOne "profile" model (easy, no migration pain):**
```python
from django.contrib.auth.models import User
from django.db import models

class Profile(models.Model):
    user   = models.OneToOneField(User, on_delete=models.CASCADE)
    bio    = models.TextField(blank=True)
    avatar = models.ImageField(upload_to='avatars/', blank=True)
```

**Option 2 — AbstractUser (set BEFORE first migration):**
```python
# models.py
from django.contrib.auth.models import AbstractUser

class User(AbstractUser):
    bio = models.TextField(blank=True)

# settings.py
AUTH_USER_MODEL = 'myapp.User'
```

> **Warning:** Swapping `AUTH_USER_MODEL` after running the initial migration is very painful. Decide early.

---

# Middleware Execution Order

Authentication depends on middleware running in the right order.

```python
MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',       # 1
    'django.contrib.sessions.middleware.SessionMiddleware',  # 2 — must come before auth
    'django.middleware.common.CommonMiddleware',            # 3
    'django.middleware.csrf.CsrfViewMiddleware',            # 4
    'django.contrib.auth.middleware.AuthenticationMiddleware',  # 5 — reads session
    'django.contrib.messages.middleware.MessageMiddleware', # 6
    'django.middleware.clickjacking.XFrameOptionsMiddleware', # 7
]
```

**Why order matters:**
- `SessionMiddleware` **must** come before `AuthenticationMiddleware`
- `AuthenticationMiddleware` reads the session to populate `request.user`
- `CsrfViewMiddleware` must be before any view that processes POST data
- `SecurityMiddleware` should be first so security headers are always set

---

# The Complete Login Example — Putting It Together

<div class="columns">
<div>

**`views.py`**
```python
from django.contrib.auth import authenticate, login
from django.shortcuts import render, redirect
from django.utils.http import url_has_allowed_host_and_scheme
from .forms import LoginForm

def login_view(request):
    if request.user.is_authenticated:
        return redirect('/')       # already logged in

    form = LoginForm(request.POST or None)
    if request.method == 'POST' and form.is_valid():
        user = authenticate(
            request,
            username=form.cleaned_data['username'],
            password=form.cleaned_data['password'],
        )
        if user:
            login(request, user)
            next_url = request.GET.get('next', '/')
            if not url_has_allowed_host_and_scheme(
                    next_url, {request.get_host()}):
                next_url = '/'
            return redirect(next_url)
        form.add_error(None, 'Invalid username or password.')
    return render(request, 'login.html', {'form': form})
```

</div>
<div>

**`forms.py`**
```python
from django import forms

class LoginForm(forms.Form):
    username = forms.CharField(
        max_length=150,
        widget=forms.TextInput(
            attrs={'autofocus': True}
        )
    )
    password = forms.CharField(
        widget=forms.PasswordInput
    )
```

**`login.html`**
```html
<form method="post">
  { % csrf_token % }
  {{ form.as_p }}
  { % if form.non_field_errors % }
    <p class="error">
      {{ form.non_field_errors }}
    </p>
  { % endif % }
  <button type="submit">Log in</button>
  <a href="{ % url 'register' % }">
    Register
  </a>
</form>
```

</div>
</div>

---

# Protecting a View — Multiple Strategies

```python
from django.contrib.auth.decorators import (
    login_required,
    permission_required,
    user_passes_test,
)

# 1. Just be logged in
@login_required
def my_profile(request):
    ...

# 2. Have a specific Django permission
@permission_required('blog.change_post', raise_exception=True)
def edit_post(request, pk):
    ...

# 3. Arbitrary predicate
@user_passes_test(lambda u: u.is_staff)
def staff_dashboard(request):
    ...

# 4. Manual check inside the view (most flexible)
from django.core.exceptions import PermissionDenied

def delete_post(request, pk):
    post = get_object_or_404(Post, pk=pk)
    if post.author != request.user and not request.user.is_staff:
        raise PermissionDenied
    post.delete()
    return redirect('/')
```

---

# Showing Permissions in Templates

```html
{ % if user.is_authenticated % }

  <p>Welcome, {{ user.username }}</p>

  { % if user.is_staff % }
    <a href="/admin/">Admin panel</a>
  { % endif % }

  { % if perms.blog.add_post % }
    <a href="{ % url 'create_post' % }">Write new post</a>
  { % endif % }

  { % if perms.blog.delete_post % }
    <a href="{ % url 'delete_post' pk=post.pk % }">Delete</a>
  { % endif % }

{ % else % }

  <a href="{ % url 'login' % }">Log in to continue</a>

{ % endif % }
```

`perms` is a template variable automatically injected by `django.contrib.auth.context_processors.auth`.
It provides a lazy per-user permission lookup without extra queries.

---

# Groups — Managing Permissions at Scale

Assigning permissions to individual users gets messy fast.
**Groups** let you manage permissions for roles.

```python
from django.contrib.auth.models import Group, Permission, User

# Create a group
editors = Group.objects.create(name='editors')

# Assign permissions to the group
perm_add    = Permission.objects.get(codename='add_post')
perm_change = Permission.objects.get(codename='change_post')
editors.permissions.set([perm_add, perm_change])

# Add a user to the group
alice = User.objects.get(username='alice')
alice.groups.add(editors)

# Check — alice now has both permissions via group membership
alice.has_perm('blog.add_post')    # True
alice.has_perm('blog.delete_post') # False
```

Groups map well to real-world roles: `editors`, `moderators`, `subscribers`, `managers`.
You manage permissions in one place (the group) and just move users in and out.

---

# Session Security — Important Details

**Session fixation attack:**
An attacker tricks you into using a session key they already know,
then after you log in they have a valid authenticated session.

**Django's defence:** `login()` calls `request.session.cycle_key()` which generates a new session key while preserving session data. The old key is invalidated.

**Session expiry settings (`settings.py`):**
```python
SESSION_COOKIE_AGE = 1209600      # seconds = 2 weeks (default)
SESSION_EXPIRE_AT_BROWSER_CLOSE = False  # keep cookie across browser restarts
SESSION_SAVE_EVERY_REQUEST = False       # only save session when modified
```

**Limit session lifetime for sensitive apps:**
```python
SESSION_COOKIE_AGE = 3600         # 1 hour
SESSION_EXPIRE_AT_BROWSER_CLOSE = True
```

**Force re-authentication after sensitive actions (e.g., changing email):**
```python
from django.contrib.auth import update_session_auth_hash
update_session_auth_hash(request, request.user)
# regenerates session hash so other sessions are invalidated
```

---

# CSRF — Cross-Site Request Forgery

Closely related to auth — if CSRF is missing, logout (and other actions) can be forced.

**The attack:**
```html
<!-- On evil.com -->
<form action="https://yourbank.com/transfer/" method="post" id="f">
  <input name="amount" value="10000">
  <input name="to_account" value="attacker">
</form>
<script>document.getElementById('f').submit();</script>
```

If you are logged in to `yourbank.com`, your browser attaches the session cookie automatically.

**Django's defence — CSRF middleware + token:**
```html
<form method="post">
  { % csrf_token % }
  ...
</form>
```

The `{% csrf_token %}` renders a hidden input:
```html
<input type="hidden" name="csrfmiddlewaretoken" value="abc123XYZ...">
```

This value is tied to your session. `evil.com` cannot read it (same-origin policy), so it cannot forge a valid request.

---

# Security Checklist — Production Hardening

```python
# settings.py

DEBUG = False                    # NEVER True in production

SECRET_KEY = os.environ['DJANGO_SECRET_KEY']  # from environment, not source code

ALLOWED_HOSTS = ['mysite.com', 'www.mysite.com']

# HTTPS
SECURE_SSL_REDIRECT = True               # redirect all HTTP → HTTPS
SESSION_COOKIE_SECURE = True             # sessionid only over HTTPS
CSRF_COOKIE_SECURE = True                # CSRF cookie only over HTTPS
SECURE_HSTS_SECONDS = 31536000           # tell browsers: HTTPS only for 1 year
SECURE_HSTS_INCLUDE_SUBDOMAINS = True
SECURE_HSTS_PRELOAD = True

# Cookie hardening
SESSION_COOKIE_HTTPONLY = True           # JS cannot read sessionid
SESSION_COOKIE_SAMESITE = 'Lax'         # blocks cross-site POST

# Clickjacking
X_FRAME_OPTIONS = 'DENY'
```

Run Django's built-in check:
```bash
uv run python manage.py check --deploy
```

---

# OWASP Top 10 — Auth-Related Vulnerabilities

The Open Web Application Security Project publishes the most critical web security risks.

**Directly relevant to authentication:**

| # | Vulnerability | Example |
|---|---|---|
| A07:2021 | **Identification and Authentication Failures** | Weak passwords, no rate-limiting, session not invalidated on logout |
| A01:2021 | **Broken Access Control** | Normal user accessing `/admin/`; IDOR (accessing `?id=42` when you own `id=17`) |
| A02:2021 | **Cryptographic Failures** | Storing plaintext passwords; using MD5 for hashing |
| A03:2021 | **Injection** | SQL injection bypassing login: `' OR 1=1 --` |

**Django protects you from many of these by default**, but you must:
- Enforce strong password policies (`AUTH_PASSWORD_VALIDATORS`)
- Rate-limit login attempts (use `django-axes` or similar)
- Never expose stack traces (`DEBUG=False`)
- Keep Django and dependencies updated

---

# AUTH_PASSWORD_VALIDATORS

Django can enforce password strength rules.

```python
# settings.py
AUTH_PASSWORD_VALIDATORS = [
    {
        'NAME': 'django.contrib.auth.password_validation.UserAttributeSimilarityValidator',
        # Rejects passwords too similar to username, email, etc.
    },
    {
        'NAME': 'django.contrib.auth.password_validation.MinimumLengthValidator',
        'OPTIONS': {'min_length': 12},
    },
    {
        'NAME': 'django.contrib.auth.password_validation.CommonPasswordValidator',
        # Rejects the 20 000 most common passwords (e.g. "password", "123456")
    },
    {
        'NAME': 'django.contrib.auth.password_validation.NumericPasswordValidator',
        # Rejects entirely numeric passwords
    },
]
```

These validators run automatically when you call:
- `User.objects.create_user()`
- `user.set_password()`
- `PasswordChangeForm` / `SetPasswordForm`

You can also run them manually:
```python
from django.contrib.auth.password_validation import validate_password
validate_password('hunter2', user=request.user)  # raises ValidationError if weak
```

---

# Rate Limiting Login — `django-axes`

Brute-force protection is not built into Django — use `django-axes`.

```bash
uv add django-axes
uv run python manage.py migrate
```

**`settings.py`:**
```python
INSTALLED_APPS = [..., 'axes']

MIDDLEWARE = [
    ...
    'axes.middleware.AxesMiddleware',   # should be last
]

AUTHENTICATION_BACKENDS = [
    'axes.backends.AxesStandaloneBackend',  # must be first
    'django.contrib.auth.backends.ModelBackend',
]

AXES_FAILURE_LIMIT = 5          # lock after 5 failed attempts
AXES_COOLOFF_TIME  = 1          # locked for 1 hour
AXES_LOCKOUT_PARAMETERS = ['username', 'ip_address']
```

`authenticate()` now automatically checks axes — no view changes needed.
Locked accounts return `None` from `authenticate()` just like bad passwords.

---

# Putting It All Together — URL Configuration

A typical auth URL setup:

```python
# urls.py
from django.urls import path
from django.contrib.auth import views as auth_views
from . import views

urlpatterns = [
    # Custom views
    path('register/',  views.register,    name='register'),
    path('login/',     views.login_view,  name='login'),
    path('logout/',    views.logout_view, name='logout'),

    # OR use Django's built-in class-based views:
    path('login/',  auth_views.LoginView.as_view(template_name='login.html'),  name='login'),
    path('logout/', auth_views.LogoutView.as_view(),                           name='logout'),

    # Password management
    path('password-change/', auth_views.PasswordChangeView.as_view(
        template_name='password_change.html'), name='password_change'),
    path('password-change/done/', auth_views.PasswordChangeDoneView.as_view(
        template_name='password_change_done.html'), name='password_change_done'),
    path('password-reset/', auth_views.PasswordResetView.as_view(
        template_name='password_reset.html'), name='password_reset'),
    path('password-reset/done/', auth_views.PasswordResetDoneView.as_view(
        template_name='password_reset_done.html'), name='password_reset_done'),
    path('reset/<uidb64>/<token>/', auth_views.PasswordResetConfirmView.as_view(
        template_name='password_reset_confirm.html'), name='password_reset_confirm'),
    path('reset/done/', auth_views.PasswordResetCompleteView.as_view(
        template_name='password_reset_complete.html'), name='password_reset_complete'),
]
```

---

# Common Mistakes and How to Avoid Them

| Mistake | Consequence | Fix |
|---|---|---|
| `user.password = 'secret'` then `save()` | Plaintext password in DB | Use `set_password()` |
| Logout via GET | CSRF logout attack | Use POST + CSRF token |
| No `@login_required` on sensitive views | Unauthenticated access | Add decorator or mixin |
| `DEBUG=True` in production | Stack traces expose secrets | `DEBUG=False`, env vars |
| Trusting `?next=` blindly | Open redirect to evil.com | Validate with `url_has_allowed_host_and_scheme` |
| `SECRET_KEY` in source code | Anyone with the repo can forge sessions | Load from environment variable |
| No HTTPS | Session cookie intercepted | `SECURE_SSL_REDIRECT=True` |
| Using `MD5`/`SHA1` for passwords | Fast to brute-force | Use Django's default PBKDF2 or Argon2 |

---

# Quick Reference — Key Imports

```python
# Models
from django.contrib.auth.models import User, Group, Permission

# Functions
from django.contrib.auth import (
    authenticate,      # verify credentials, returns User or None
    login,             # create session, attach user to request
    logout,            # destroy session
    update_session_auth_hash,  # keep user logged in after password change
)

# Decorators
from django.contrib.auth.decorators import (
    login_required,
    permission_required,
    user_passes_test,
)

# Class-based view mixins
from django.contrib.auth.mixins import (
    LoginRequiredMixin,
    PermissionRequiredMixin,
    UserPassesTestMixin,
)

# Built-in auth views
from django.contrib.auth import views as auth_views
# auth_views.LoginView, LogoutView, PasswordChangeView, PasswordResetView, …

# Validation
from django.contrib.auth.password_validation import validate_password
```

---

# Class-Based Views and Auth Mixins

If you prefer class-based views, use mixins instead of decorators:

```python
from django.contrib.auth.mixins import (
    LoginRequiredMixin,
    PermissionRequiredMixin,
    UserPassesTestMixin,
)
from django.views.generic import ListView, CreateView

class PostListView(LoginRequiredMixin, ListView):
    model = Post
    login_url = '/login/'           # override default LOGIN_URL
    redirect_field_name = 'next'


class CreatePostView(PermissionRequiredMixin, CreateView):
    model = Post
    permission_required = 'blog.add_post'
    raise_exception = True          # return 403 instead of redirecting


class StaffView(UserPassesTestMixin, ListView):
    model = Post

    def test_func(self):
        return self.request.user.is_staff
```

Mixins must come **before** the generic view class in the MRO (left of `ListView`).

---

# OAuth2 — The Problem It Solves

**Traditional login:** user creates a username + password on *your* site. You store a hash. You are responsible for security.

**OAuth2:** user proves their identity to a trusted third party (GitHub, Google). They give *you* a token. You never see a password.

```
User         Your site         GitHub
 │                │                │
 │──"Login with──►│                │
 │   GitHub"      │                │
 │                │──redirect ────►│
 │◄──────────────────────────────  │
 │  github.com/login               │
 │──(enter GitHub password)───────►│
 │                │◄──auth code ───│
 │                │──exchange ────►│
 │                │◄──access token─│
 │                │  (fetch user)  │
 │◄──logged in ───│                │
```

---

# OAuth2 — The Four Roles

| Role | In practice |
|------|-------------|
| **Resource Owner** | The user (owns their GitHub account) |
| **Client** | Your Django app |
| **Authorization Server** | GitHub's auth endpoint (`github.com/login/oauth`) |
| **Resource Server** | GitHub's API (`api.github.com/user`) |

**Two key endpoints at the provider:**

| Endpoint | Purpose |
|----------|---------|
| Authorization URL | Redirect the user here to ask for permission |
| Token URL | Exchange the auth code for an access token |

Your app gets a **Client ID** (public) and **Client Secret** (private) when you register it with GitHub.

---

# OAuth2 — Authorization Code Flow (Step by Step)

1. **User clicks "Login with GitHub"**
   - Your app redirects to:
     `https://github.com/login/oauth/authorize?client_id=…&scope=read:user&state=…`

2. **User approves** on GitHub

3. **GitHub redirects back** to your `callback_url` with a short-lived `?code=…&state=…`

4. **Your server exchanges the code** (server-to-server, never exposed to the browser):
   ```
   POST https://github.com/login/oauth/access_token
   { client_id, client_secret, code }
   → { access_token: "gho_…", scope: "read:user" }
   ```

5. **Your server calls the GitHub API** with the token:
   ```
   GET https://api.github.com/user
   Authorization: Bearer gho_…
   → { login: "alice", email: "alice@example.com", ... }
   ```

6. **Create or find the Django `User`**, call `login()`, set session cookie.

---

# OAuth2 — Security Details

**The `state` parameter:**
- Random string your app generates and stores in the session before the redirect
- GitHub echoes it back in the callback — you verify it matches
- Prevents **CSRF on the callback**: attacker cannot forge a callback for a different user

**Access token ≠ session:**
- The access token is a credential for GitHub's API — it is **not** the Django session
- Store it only if you need to call the GitHub API later; otherwise discard it
- Your Django session is still just a `sessionid` cookie pointing to a DB row

**Scopes:**
- Request only what you need: `read:user` for login (name, email, avatar)
- Avoid `repo`, `write:org` — users will refuse over-scoped apps

**Never expose `CLIENT_SECRET`:**
- Store in environment variables, not in source code
- The secret is exchanged server-to-server — the browser never sees it

---

# `django-allauth` — What It Does

`django-allauth` implements the full OAuth2 flow (and 80+ providers) so you write zero redirect/token/callback code.

```bash
uv add django-allauth
```

```python
# settings.py
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
    "django.contrib.auth.backends.ModelBackend",
    "allauth.account.auth_backends.AuthenticationBackend",
]
```

```python
# urls.py
path("accounts/", include("allauth.urls")),
```

```bash
uv run python manage.py migrate
```

---

# `django-allauth` — GitHub Provider Setup

**1. Register a GitHub OAuth App:**
Go to GitHub → Settings → Developer Settings → OAuth Apps → New OAuth App

| Field | Value |
|-------|-------|
| Application name | My Django Blog (dev) |
| Homepage URL | `http://127.0.0.1:8000` |
| Authorization callback URL | `http://127.0.0.1:8000/accounts/github/login/callback/` |

Copy the **Client ID** and generate a **Client Secret**.

**2. Add the credentials to Django:**

In the Django Admin (`/admin/`), go to:
`Social Applications → Add Social Application`

| Field | Value |
|-------|-------|
| Provider | GitHub |
| Name | GitHub |
| Client ID | (from GitHub) |
| Secret key | (from GitHub) |
| Sites | move `example.com` to Chosen sites |

---

# `django-allauth` — Login Button & Flow

Add the button to any template:

```html
<a href="{% url 'github_login' %}">Login with GitHub</a>
```

Or if you're using the allauth namespace:

```html
<a href="{% url 'socialaccount_signup' %}">Sign up</a>

{% load socialaccount %}
<a href="{% provider_login_url 'github' %}">Login with GitHub</a>
```

**What allauth does automatically:**
1. Redirects the user to GitHub's authorization page
2. Handles the callback, exchanges the code for a token
3. Fetches the user's profile from `api.github.com/user`
4. Creates (or finds) a Django `User` matching the GitHub account
5. Calls Django's `login()` — `request.user` is now set
6. Redirects to `LOGIN_REDIRECT_URL`

You can customise behaviour (require email, auto-connect accounts) via `SOCIALACCOUNT_PROVIDERS` in settings.

---

# `django-allauth` — Environment Variable Best Practice

Never commit `CLIENT_SECRET` to git. Use environment variables:

```bash
# .env  (add to .gitignore)
GITHUB_CLIENT_ID=Ov23liABC123
GITHUB_CLIENT_SECRET=abc123secret
```

```python
# settings.py
import os

SOCIALACCOUNT_PROVIDERS = {
    "github": {
        "APP": {
            "client_id":     os.environ["GITHUB_CLIENT_ID"],
            "secret":        os.environ["GITHUB_CLIENT_SECRET"],
            "key":           "",
        },
        "SCOPE": ["read:user", "user:email"],
    }
}
```

This bypasses the Admin DB entry and keeps secrets out of your database too.

Run with:
```bash
GITHUB_CLIENT_ID=… GITHUB_CLIENT_SECRET=… uv run python manage.py runserver
# or use python-dotenv / direnv to load .env automatically
```

---

<div class="columns">
<div>

**Core concepts**
- Authentication = who you are
- Authorisation = what you can do
- HTTP is stateless — sessions + cookies bridge the gap
- Session key is random; sensitive data stays server-side
- Passwords must be hashed (PBKDF2, Argon2)

**Django tools**
- `User` model — `create_user()`, `set_password()`
- `authenticate()`, `login()`, `logout()`
- `request.user`, `is_authenticated`
- `AnonymousUser` sentinel

</div>
<div>

**Protecting views**
- `@login_required` → redirect or 403
- `@permission_required` → model-level permissions
- `@user_passes_test` → arbitrary predicate
- Mixins for class-based views

**Security**
- Always POST for logout (CSRF)
- `DEBUG=False` in production
- `SECRET_KEY` from environment
- `SESSION_COOKIE_SECURE`, `HTTPS`
- Validate `?next=` before redirect
- Rate-limit login attempts

**OAuth2 / Social Login**
- `django-allauth` — plug-in social auth
- GitHub, Google, etc. as identity providers
- Access token ≠ session — don't confuse them

</div>
</div>

---

# Lab 6 Preview

**You will build a complete authentication system on top of your existing blog project.**

Tasks:
1. Enable `django.contrib.auth` and run migrations
2. Create a **registration page** with a custom form and immediate login
3. Create a **login page** using `LoginView` (supply the template)
4. Add a **logout button** (POST form) in the navbar
5. Protect the "create post" and "delete post" views with `@login_required`
6. Show the author's username on each post
7. Restrict delete to the post's own author (`request.user == post.author`)
8. Add **GitHub OAuth2 login** using `django-allauth`
9. *Bonus:* Add password change view using `PasswordChangeView`

**Deliverable:** A running Django site where users can log in with username/password *or* with their GitHub account.

Start with:
```bash
uv run python manage.py migrate
uv run python manage.py createsuperuser
uv run python manage.py runserver
```

---

# Questions?

<br>

## Key takeaways

1. **Stateless HTTP + sessions = stateful web** — the cookie holds only a random key, the data lives in the DB
2. **Never store plaintext passwords** — Django hashes automatically if you use `create_user()` / `set_password()`
3. **Logout must be POST** — GET logout is a CSRF vulnerability
4. **`@login_required` is your first line of defence** — every sensitive view needs it
5. **OAuth2 delegates identity** — the provider (GitHub) verifies the user; you just receive a token
6. **`DEBUG=False` + HTTPS + env vars** — the production checklist matters

<br>

> Further reading:
> - Django docs: *Authentication in Django* — docs.djangoproject.com/en/5.0/topics/auth/
> - OWASP Authentication Cheat Sheet — cheatsheetseries.owasp.org
> - RFC 6749 — The OAuth 2.0 Authorization Framework
> - django-allauth docs — docs.allauth.org
{% endraw %}
