# Lab 2: HTML — Structuring the Web

## Introduction

In Lab 1 you built a server that speaks HTTP. Now you will fill that server with real content. HTML (HyperText Markup Language) is the language browsers use to understand structure and meaning — not just appearance.

**Goal:** Build a multi-page HTML site served by your Lab 1 server. You will use semantic structure, hyperlinks, forms, and tables.

### The Theory

HTML is a tree of *elements*. Every element has:

- An **opening tag**: `<p>`
- A **closing tag**: `</p>`
- Optional **attributes**: `<a href="https://example.com">`
- A **text node** or nested child elements in between.

The browser turns this tree into the DOM (Document Object Model). DevTools let you inspect it live.

## Setup

Extend your `server.py` from Lab 1. Instead of returning a hardcoded `<h1>` string, we will serve `.html` files from disk.

Add the following imports at the top of `server.py`. You will use `unquote_plus` later when parsing form data:

```python
import os
from urllib.parse import unquote_plus
```

Add a MIME-type map and a `read_file` helper:

```python
MIME_TYPES = {
    ".html": "text/html",
    ".css":  "text/css",
    ".js":   "application/javascript",
    ".png":  "image/png",
    ".jpg":  "image/jpeg",
    ".ico":  "image/x-icon",
}

def read_file(path):
    public_root = os.path.abspath("public")
    abs_path    = os.path.abspath(os.path.join("public", path.lstrip("/")))

    # Reject paths that escape public/
    if os.path.commonpath([public_root, abs_path]) != public_root:
        return b"<h1>403 Forbidden</h1>", "403 Forbidden", "text/html"

    ext  = os.path.splitext(abs_path)[1].lower()
    mime = MIME_TYPES.get(ext, "application/octet-stream")

    try:
        with open(abs_path, "rb") as f:   # binary mode works for text AND images
            return f.read(), "200 OK", mime
    except (FileNotFoundError, IsADirectoryError):
        return b"<h1>404 Not Found</h1>", "404 Not Found", "text/html"
```

> **Why binary mode?**  Opening a `.png` in text mode raises `UnicodeDecodeError`. Binary mode (`"rb"`) is safe for every file type — the browser interprets the bytes using the `Content-Type` header you set.

> **Why `os.path.abspath`?**  Without this check, a browser request for `/../../../etc/passwd` could read files outside `public/`. This is called a *path traversal* vulnerability.

Update `generate_response` to accept and forward the MIME type:

```python
def generate_response(content, status, mime="text/html"):
    if isinstance(content, str):
        content = content.encode()
    response_line    = f"HTTP/1.1 {status}\r\n"
    response_headers = f"Content-Type: {mime}\r\nContent-Length: {len(content)}\r\n\r\n"
    return response_line.encode() + response_headers.encode() + content
```

Update your request handler loop for normal file requests:

```python
path = parse_request(request_data)
content, status, mime = read_file(path if path != "/" else "/index.html")
response = generate_response(content, status, mime)
client_connection.sendall(response)
client_connection.close()
```

Create a folder `public/` next to `server.py`. All HTML files will live there.

## Phase 1: Basic HTML Page

Every valid HTML page starts with the same boilerplate. Create `public/index.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>My Profile</title>
</head>
<body>

    <!-- TODO: Add a level-1 heading with your name -->

    <!-- TODO: Add a paragraph describing yourself in 2 sentences -->

</body>
</html>
```

**Check:** Run your server and open `http://localhost:8000` in a browser. You should see your heading and paragraph. Open DevTools → Elements and inspect the DOM tree.

## Phase 2: Semantic Elements

Generic `<div>` tags carry no meaning. Semantic elements tell both the browser and screen readers *what* a section is.

Replace your `<body>` content with a properly structured page:

```html
<body>

    <header>
        <!-- TODO: Add a <nav> with an unordered list <ul> of 3 links:
             Home (index.html), Projects (projects.html), Contact (contact.html) -->
    </header>

    <main>
        <article>
            <h1>About Me</h1>
            <!-- TODO: Add a <section> for "Education" with an <h2> heading
                 and a paragraph. -->

            <!-- TODO: Add a <section> for "Skills" with an <h2> heading
                 and an unordered list <ul> of at least 4 skills. -->
        </article>

        <aside>
            <details>
                <summary>Note</summary>
                <!-- TODO: Replace this comment with a <p> containing one short note about yourself. -->
            </details>
        </aside>
    </main>

    <footer>
        <!-- TODO: Add a paragraph: "© 2026 Your Name" -->
    </footer>

</body>
```

**Check:** In DevTools, confirm `<header>`, `<main>`, `<article>`, `<aside>`, and `<footer>` all appear as separate nodes.

## Phase 3: Tables

Create `public/projects.html`. Display a table of your (real or imaginary) projects.

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Projects</title>
</head>
<body>
    <h1>My Projects</h1>

    <table>
        <thead>
            <tr>
                <!-- TODO: Add <th> headers: Name, Language, Status, Link.
                     Use scope="col" on each <th>. -->
            </tr>
        </thead>
        <tbody>
            <!-- TODO: Add at least 3 <tr> rows.
                 Each row needs 4 <td> cells.
                 The Link cell should contain an <a> tag.
                 Use target="_blank" to open in a new tab. -->
        </tbody>
    </table>

    <p><a href="index.html">← Back to Home</a></p>
</body>
</html>
```

**Check:** Navigate from your index page to projects and back using only the links you created.

## Phase 4: Forms

Forms are how users send data to the server. Create `public/contact.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Contact</title>
</head>
<body>
    <h1>Contact Me</h1>

    <form action="/submit" method="POST">
        <fieldset>
            <legend>Your Details</legend>

            <!-- TODO: Add a label + text input for "Name"
                 Hint: <label for="name">Name</label>
                       <input type="text" id="name" name="name" required> -->

            <!-- TODO: Add a label + email input for "Email" -->

            <!-- TODO: Add a label + <textarea> for "Message" (rows=5, cols=40) -->

            <!-- TODO: Add a label + <select> dropdown for "Subject"
                 with options: General, Bug Report, Feature Request -->

        </fieldset>

        <!-- TODO: Add a submit button -->

    </form>

    <p><a href="index.html">Back to Home</a></p>
</body>
</html>
```

Now update your server to handle the `/submit` path. Parse the POST body (it arrives as `key=value&key=value` pairs):

```python
def parse_post_body(request_data):
    # The body is separated from headers by a blank line
    parts = request_data.split("\r\n\r\n", 1)
    if len(parts) < 2:
        return {}
    body = parts[1]
    # TODO: Split body by '&', then each pair by the first '='.
    # Apply unquote_plus() to both key and value — browsers encode
    # spaces as '+' and special chars as %XX (e.g. "Hello+World%21").
    # Return a dict like {"name": "Alice", "email": "..."}
    return {}
```

Your existing `parse_request` function probably returns only the path. For form handling, you also need to know the HTTP method from the first request line:

```text
POST /submit HTTP/1.1
```

You can either update `parse_request` to return both method and path, or create a separate `parse_method` helper:

```python
def parse_request(request_data):
    if not request_data:
        return "", ""
    request_line = request_data.splitlines()[0]
    parts = request_line.split()
    if len(parts) < 2:
        return "", ""
    method = parts[0]
    path = parts[1]
    return method, path
```

Then branch in the handler:

```python
method, path = parse_request(request_data)

if method == "POST" and path == "/submit":
    fields = parse_post_body(request_data)
    name = fields.get("name", "")
    html = f"<h1>Thanks, {name}</h1>"
    response = generate_response(html, "200 OK")
else:
    content, status, mime = read_file(path if path != "/" else "/index.html")
    response = generate_response(content, status, mime)

client_connection.sendall(response)
client_connection.close()
```

When the path is `/submit` and the method is `POST`, respond with an HTML confirmation page showing the submitted name.

**Check:** Fill in the form and submit. Confirm your server prints the parsed fields and returns the confirmation page.

## Phase 5: Final Checks

Your `public/` folder should contain three linked HTML pages. Make sure:

1. All pages are reachable by clicking links only (no manually typing URLs after the first load).
2. The contact form shows a confirmation with the user's name.
3. The DevTools Accessibility tab shows no missing `alt` attributes. Add `alt` text to any `<img>` tags you used.

**Optional check:** Open any real website, press F12, and find one HTML tag you have not used yet. What does it do?
