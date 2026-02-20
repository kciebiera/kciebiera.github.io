# Lab 2: HTML — Structuring the Web

## Introduction

In Lab 1 you built a server that speaks HTTP. Now you will fill that server with real content. HTML (HyperText Markup Language) is the language browsers use to understand structure and meaning — not just appearance.

The Goal: Build a multi-page HTML site served by your Lab 1 server. By the end you will understand semantic structure, hyperlinks, forms, and tables well enough to use them confidently.

### The Theory

HTML is a tree of *elements*. Every element has:

- An **opening tag**: `<p>`
- A **closing tag**: `</p>`
- Optional **attributes**: `<a href="https://example.com">`
- A **text node** or nested child elements in between.

The browser turns this tree into the DOM (Document Object Model). DevTools let you inspect it live.

## Setup

Extend your `server.py` from Lab 1. Instead of returning a hardcoded `<h1>` string, we will serve `.html` files from disk.

Add a helper to your server:

```python
def read_file(path):
    try:
        with open("." + path, "r") as f:
            return f.read(), "200 OK"
    except FileNotFoundError:
        return "<h1>404 Not Found</h1>", "404 Not Found"
```

Update your request handler loop:

```python
path = parse_request(request_data)
content, status = read_file(path if path != "/" else "/index.html")
response = generate_response(content, status)
client_connection.sendall(response)
client_connection.close()
```

Create a folder `public/` next to `server.py`. All HTML files will live there. Adjust `read_file` to look inside `public/`.

## Phase 1: The Skeleton

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

🧪 Test It: Run your server and open `http://localhost:8000` in a browser. You should see your heading and paragraph. Open DevTools → Elements and inspect the DOM tree.

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
            <!-- TODO: Add a fun fact about yourself in a <p> tag. -->
        </aside>
    </main>

    <footer>
        <!-- TODO: Add a paragraph: "© 2025 Your Name" -->
    </footer>

</body>
```

🧪 Verify: In DevTools, confirm `<header>`, `<main>`, `<article>`, `<aside>`, and `<footer>` all appear as separate nodes.

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
                <!-- TODO: Add <th> headers: Name, Language, Status, Link -->
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

🧪 Test It: Navigate from your index page to projects and back using only the links you created.

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

        <!-- TODO: Add a label + text input for "Name"
             Hint: <label for="name">Name</label>
                   <input type="text" id="name" name="name" required> -->

        <!-- TODO: Add a label + email input for "Email" -->

        <!-- TODO: Add a label + <textarea> for "Message" (rows=5, cols=40) -->

        <!-- TODO: Add a label + <select> dropdown for "Subject"
             with options: General, Bug Report, Feature Request -->

        <!-- TODO: Add a submit button -->

    </form>
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
    # TODO: Split body by '&', then each pair by '=',
    # return a dict like {"name": "Alice", "email": "..."}
    return {}
```

When the path is `/submit` and you have parsed the body, respond with an HTML confirmation page showing the submitted name.

🧪 Fill in the form and submit. Confirm your server prints the parsed fields and returns the confirmation page.

## Submission

Your `public/` folder should contain three linked HTML pages. Verify:

1. All pages are reachable by clicking links only (no manually typing URLs after the first load).
2. The contact form submission shows a confirmation with the user's name.
3. The DevTools Accessibility tab shows no missing `alt` attributes. Add `alt` text to any `<img>` tags you used.

**Exploration:** Open any real website, press F12, and find one element whose tag surprised you. What does it do?
