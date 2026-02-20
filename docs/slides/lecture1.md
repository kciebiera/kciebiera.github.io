---
marp: true
theme: default
paginate: true
style: |
  section { font-size: 1.4rem; }
  pre { font-size: 0.85rem; }
  h1 { color: #1a6bbf; }
  h2 { color: #2c7be5; }
  .columns { display: grid; grid-template-columns: 1fr 1fr; gap: 1rem; }
---

# Lecture 1
## The Web from First Principles

**WWW 25/26**
HTTP · Sockets · Protocols

---

# Course Overview

**13 labs** building a full web stack from scratch:

| Labs 1–3 | HTML, CSS, raw HTTP |
|-----------|---------------------|
| Labs 4–8  | Django (Python framework) |
| Labs 9    | REST APIs |
| Labs 10–13 | TypeScript + full-stack project |

**Philosophy:** Build it by hand first. Then you'll know what frameworks are saving you from.

---

# What Happens When You Type a URL?

```
https://en.wikipedia.org/wiki/HTTP
```

1. Browser asks DNS: *what is the IP of `en.wikipedia.org`?*
2. DNS replies: `185.15.58.224`
3. Browser opens a **TCP connection** to that IP, port 443
4. Browser sends an **HTTP request** (text)
5. Server sends back an **HTTP response** (text + bytes)
6. Browser parses the response and renders the page

Every single step is a protocol. Today we focus on steps 3–5.

---

# The Internet vs the Web

<div class="columns">
<div>

**The Internet**
A global network of computers connected by cables, fibre, and radio.

Protocols: TCP/IP, UDP, DNS, …

It has existed since the 1970s.

</div>
<div>

**The World Wide Web**
One application *on top of* the internet.

Documents linked by hyperlinks, fetched over HTTP.

Invented by Tim Berners-Lee in **1989** at CERN.

</div>
</div>

> The web is to the internet as email is to the postal system.

---

# Clients and Servers

```
  CLIENT                          SERVER
  ┌──────────┐    HTTP request    ┌──────────────┐
  │ Browser  │ ──────────────────▶│  Web Server  │
  │ curl     │                    │  (your code) │
  │ mobile   │ ◀──────────────────│              │
  └──────────┘    HTTP response   └──────────────┘
```

- A **client** initiates connections and makes requests.
- A **server** listens, accepts connections, and sends responses.
- The same machine can be both (e.g. your laptop in development).
- Roles are *per-connection*, not per-machine.

---

# IP Addresses

Every device on the internet has a numeric address.

| Version | Format | Example |
|---------|--------|---------|
| IPv4 | 4 bytes, dotted decimal | `93.184.216.34` |
| IPv6 | 16 bytes, hex | `2606:2800:220:1:248:1893:25c8:1946` |

**Special addresses:**
- `127.0.0.1` / `localhost` — your own machine (loopback)
- `0.0.0.0` — "all interfaces" (listen on everything)

---

# Ports

IP addresses identify machines. **Ports** identify *services* on a machine.

```
93.184.216.34 : 443
     ▲               ▲
  machine          service
```

| Port | Service |
|------|---------|
| 80   | HTTP |
| 443  | HTTPS |
| 22   | SSH |
| 5432 | PostgreSQL |
| 8000 | Django dev server (convention) |

Ports 0–1023 are "well-known"; using them requires root. Use 1024+ in dev.

---

# TCP — Transmission Control Protocol

TCP is the transport layer under HTTP.

**Guarantees:**
- Data arrives **in order**
- Data arrives **complete** (retransmits on loss)
- Both sides know if the connection **breaks**

**Cost:** Connection setup takes a round trip (the 3-way handshake).

```
Client          Server
  │── SYN ──────────▶│
  │◀──── SYN-ACK ────│
  │── ACK ──────────▶│
  │                  │
  │  (data flows)    │
```

---

# What is a Protocol?

A **protocol** is an agreement between two parties on:
- The **format** of messages
- The **order** of messages
- What to do when things go **wrong**

Protocols are just text documents (RFCs — Request for Comments).

HTTP is defined in:
- RFC 7230–7235 (HTTP/1.1, 2014)
- RFC 9110–9114 (HTTP semantics, 2022)

You can read them at **rfc-editor.org** — they are public and free.

---

# HTTP — HyperText Transfer Protocol

HTTP is an **application-layer** protocol on top of TCP.

Key design decisions:
- **Text-based** — human-readable (intentionally)
- **Stateless** — each request is independent; the server forgets you
- **Request/Response** — one request, one response
- **Extensible** — headers let you add metadata without changing the format

HTTP/1.1 (1997) is what you'll implement in Lab 1.
HTTP/2 and HTTP/3 add performance, but same semantics.

---

# HTTP is Just Text

Open a TCP connection and type:

```
GET / HTTP/1.1
Host: example.com

```
*(blank line ends the headers)*

The server replies:

```
HTTP/1.1 200 OK
Content-Type: text/html
Content-Length: 1256

<!doctype html>
<html>...
```

That's it. Everything else is built on top of this.

---

# The HTTP Request

```
GET /search?q=http HTTP/1.1          ← Request Line
Host: www.google.com                 ─┐
User-Agent: Mozilla/5.0              │ Headers
Accept: text/html                    ─┘
                                     ← Blank line (end of headers)
                                     ← Body (empty for GET)
```

The **Request Line** has three parts:
1. **Method** — what action to perform
2. **Path** — which resource
3. **Version** — HTTP/1.1

---

# HTTP Request Methods

| Method | Meaning | Has Body? |
|--------|---------|-----------|
| `GET` | Fetch a resource | No |
| `POST` | Submit data / create | Yes |
| `PUT` | Replace a resource | Yes |
| `PATCH` | Partially update | Yes |
| `DELETE` | Remove a resource | No |
| `HEAD` | Like GET but no body | No |
| `OPTIONS` | Ask what's allowed | No |

**Rule of thumb:** `GET` should never change server state. It must be *safe* and *idempotent*.

---

# The HTTP Response

```
HTTP/1.1 200 OK                       ← Status Line
Content-Type: text/html; charset=utf-8 ─┐
Content-Length: 2048                    │ Headers
Date: Thu, 01 Jan 2026 12:00:00 GMT    ─┘
                                        ← Blank line
<!doctype html>                        ─┐
<html>                                  │ Body
  <body>Hello!</body>                   │
</html>                                ─┘
```

The **Status Line** has three parts:
1. **Version** — HTTP/1.1
2. **Status Code** — three-digit number
3. **Reason Phrase** — human-readable description

---

# HTTP Status Codes

Status codes are grouped by their first digit:

| Range | Category | Meaning |
|-------|----------|---------|
| 1xx | Informational | Request received, continuing |
| 2xx | Success | Request fulfilled |
| 3xx | Redirection | Do something else |
| 4xx | Client Error | You made a mistake |
| 5xx | Server Error | Server made a mistake |

---

# Common Status Codes

| Code | Name | When |
|------|------|------|
| 200 | OK | Normal success |
| 201 | Created | POST that created a resource |
| 204 | No Content | Success, no body (DELETE) |
| 301 | Moved Permanently | URL changed forever |
| 302 | Found | Temporary redirect |
| 400 | Bad Request | Malformed request |
| 401 | Unauthorized | Not logged in |
| 403 | Forbidden | Logged in but not allowed |
| 404 | Not Found | Resource doesn't exist |
| 500 | Internal Server Error | Bug in your code |

---

# HTTP Headers

Headers are **key: value** metadata attached to requests and responses.

**Common Request Headers:**
```
Host: example.com              (required in HTTP/1.1)
Accept: text/html, */*
Accept-Language: en-US
Cookie: sessionid=abc123
Authorization: Bearer <token>
Content-Type: application/json
Content-Length: 42
```

**Common Response Headers:**
```
Content-Type: text/html; charset=utf-8
Content-Length: 1024
Set-Cookie: sessionid=abc123; HttpOnly
Location: /new-url             (used with 3xx)
Cache-Control: max-age=3600
```

---

# The URL Structure

```
https://example.com:8080/path/to/page?key=val&x=1#section
  │          │        │       │           │         │
scheme      host     port    path       query    fragment
```

- **Scheme** — which protocol (`http`, `https`, `ftp`)
- **Host** — domain name or IP address
- **Port** — optional (defaults: 80 for http, 443 for https)
- **Path** — which resource on the server
- **Query String** — optional parameters (after `?`)
- **Fragment** — client-side anchor (never sent to server)

---

# DNS — Domain Name System

Humans remember names. Computers use numbers.

```
www.example.com  ──DNS query──▶  93.184.216.34
```

DNS is a distributed database replicated across thousands of servers.

**Resolution order:**
1. Browser cache
2. OS cache (`/etc/hosts`)
3. Local resolver (your router / ISP)
4. Recursive resolvers → Root → TLD → Authoritative nameserver

```bash
$ nslookup example.com
$ dig example.com
```

---

# HTTPS — HTTP Secure

HTTPS = HTTP + **TLS** (Transport Layer Security)

```
Client                     Server
  │──── TCP handshake ────────│
  │──── TLS handshake ────────│   (certificate, key exchange)
  │                           │
  │  HTTP request (encrypted) │
  │──────────────────────────▶│
  │  HTTP response (encrypted)│
  │◀──────────────────────────│
```

Everything inside TLS is encrypted — URLs, headers, body, cookies.

In Lab 1 we use plain HTTP on localhost. In production, always use HTTPS.

---

# HTTP is Stateless

Each HTTP request is **completely independent**. The server has no memory of previous requests.

```
Request 1: GET /page1  → Server: "Who are you? Don't care, here's page1"
Request 2: GET /page2  → Server: "Who are you? Don't care, here's page2"
```

**Problem:** How does a site know you're logged in?

**Solution:** The client sends proof of identity with every request.
- **Cookies** — browser stores a token, sends it automatically
- **Bearer tokens** — JS code sends a token in the `Authorization` header

---

# Python Sockets

A **socket** is an OS abstraction for a network connection.

```python
import socket

# AF_INET  = IPv4
# SOCK_STREAM = TCP (stream of bytes, reliable)
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
```

Every network library — `requests`, `urllib`, Django, Flask — uses sockets under the hood. You're cutting out the middlemen.

```python
# SO_REUSEADDR: allow reuse of port immediately after crash
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
```

---

# Binding and Listening

```python
# Bind: claim a port on this machine
s.bind(('localhost', 8000))

# Listen: enter a state of waiting for connections
# The argument (5) is the backlog — how many connections
# to queue before refusing new ones
s.listen(5)

print("Server running on http://localhost:8000")
```

After `listen()`, the socket is a **passive socket** — it only accepts connections, it doesn't send or receive data directly.

---

# Accepting Connections

```python
while True:
    # accept() blocks until a client connects
    # Returns a NEW socket for this specific connection
    conn, addr = s.accept()

    print(f"Connection from {addr}")

    # conn is a fresh socket just for this client
    # s is still listening for more connections
    data = conn.recv(4096)   # receive up to 4096 bytes
    print(data.decode('utf-8'))

    conn.close()
```

`s.accept()` → waits  
`conn.recv(n)` → waits for up to `n` bytes from this client

---

# Receiving and Sending Data

```python
# Receiving
raw_bytes = conn.recv(4096)
text = raw_bytes.decode('utf-8')

# Sending
response = "HTTP/1.1 200 OK\r\n\r\nHello!"
conn.sendall(response.encode('utf-8'))

# IMPORTANT: close the connection when done
conn.close()
```

**`recv()` vs `recvall()`:**  
There is no `recvall()` — `recv()` may return less than you asked for. For production you'd loop. In Lab 1, one `recv(4096)` is enough for typical requests.

---

# Parsing an HTTP Request

```python
def parse_request(raw):
    if not raw:
        return "GET", "/", {}

    lines = raw.split("\r\n")
    # First line: "GET /path HTTP/1.1"
    method, path, _ = lines[0].split(" ", 2)

    headers = {}
    for line in lines[1:]:
        if ": " in line:
            key, val = line.split(": ", 1)
            headers[key] = val
        else:
            break   # blank line = end of headers

    return method, path, headers
```

The body follows after the blank line (`\r\n\r\n`).

---

# Building an HTTP Response

```python
def make_response(body, status="200 OK",
                  content_type="text/html; charset=utf-8"):
    body_bytes = body.encode('utf-8')
    response  = f"HTTP/1.1 {status}\r\n"
    response += f"Content-Type: {content_type}\r\n"
    response += f"Content-Length: {len(body_bytes)}\r\n"
    response += "\r\n"          # mandatory blank line
    return response.encode('utf-8') + body_bytes
```

**`Content-Length` matters.** Without it, the browser doesn't know when the response ends and may hang waiting for more data — or truncate early.

---

# A Minimal Working Server

```python
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('localhost', 8000))
s.listen(5)

while True:
    conn, _ = s.accept()
    data = conn.recv(4096).decode('utf-8')
    _, path, _ = data.split("\r\n")[0].split(" ")

    if path == "/":
        body = "<h1>Hello from raw Python!</h1>"
    else:
        body = "<h1>404 Not Found</h1>"

    status = "200 OK" if path == "/" else "404 Not Found"
    conn.sendall(make_response(body, status))
    conn.close()
```

---

# Testing with Netcat

**Netcat (`nc`)** is a raw TCP client — no HTTP magic, no retries.

```bash
$ nc localhost 8000
GET / HTTP/1.1
Host: localhost

```
*(type the blank line and press Enter)*

You see exactly what the server sends back — raw bytes.

**Why not a browser?**
Browsers automatically add headers, retry, cache, and follow redirects. Netcat is transparent. Use it to debug when browsers hide the problem.

---

# The Visitor Counter Problem

```python
VISITORS = 0

# In the request loop:
path = parse_path(request_data)
if path == "/":
    VISITORS += 1
    body = f"<p>Visited {VISITORS} times</p>"
```

**Bug:** Browsers automatically request `/favicon.ico` on every page load.

First visit → `GET /` → count = 1  
              `GET /favicon.ico` → count = 2 ← wrong!

**Fix:** Only increment when `path == "/"`.

---

# The Single-Threaded Problem

```python
while True:
    conn, _ = s.accept()
    # --- everything below blocks the loop ---
    data = conn.recv(4096)
    time.sleep(5)        # simulating slow work
    conn.sendall(make_response("Done"))
    conn.close()
    # --- only now can the next client connect ---
```

Open two browser tabs simultaneously. The second tab **waits** for the first to finish.

This is fine for a single developer. In production, a server handles thousands of simultaneous clients.

---

# Threads to the Rescue

```python
import threading

def handle_client(conn):
    data = conn.recv(4096).decode('utf-8')
    time.sleep(5)
    conn.sendall(make_response("Done"))
    conn.close()

while True:
    conn, addr = s.accept()
    t = threading.Thread(target=handle_client, args=(conn,))
    t.start()
    # accept() is called again immediately
```

Each client gets its own thread. The main loop just dispatches.

---

# Race Conditions

```python
VISITORS = 0   # shared mutable state

def handle_client(conn):
    global VISITORS
    # Thread A reads VISITORS = 5
    # Thread B reads VISITORS = 5
    # Thread A writes VISITORS = 6
    # Thread B writes VISITORS = 6   ← lost an increment!
    VISITORS += 1
```

Two threads incrementing the same variable can **lose updates**.

**Fix:** Use a lock.

```python
lock = threading.Lock()

with lock:
    VISITORS += 1
```

---

# Locks

```python
import threading

VISITORS = 0
lock = threading.Lock()

def handle_client(conn):
    global VISITORS
    with lock:
        VISITORS += 1
        count = VISITORS
    conn.sendall(make_response(f"<p>Visitor #{count}</p>"))
    conn.close()
```

`with lock:` — only one thread can be inside this block at a time. Others wait.

This is the foundation of concurrent programming. Frameworks handle most of this for you automatically.

---

# What Frameworks Actually Do

You have now experienced:

| Problem | Framework solution |
|---------|--------------------|
| Parse HTTP request | Done for you |
| Route paths to functions | URL dispatcher |
| Build HTTP responses | `HttpResponse` / `render()` |
| Serve static files | Static file middleware |
| Handle threading | WSGI / ASGI worker pool |
| Prevent race conditions | ORM + database transactions |

Frameworks don't hide complexity. They **relocate** it to battle-tested code.

---

# HTTP/2 and HTTP/3 (Brief Overview)

**HTTP/1.1** — one request per TCP connection (at a time)

**HTTP/2** (2015):
- Binary framing instead of plain text
- **Multiplexing** — multiple requests over one TCP connection
- Header compression (HPACK)
- Server Push

**HTTP/3** (2022):
- Runs over **QUIC** (UDP-based) instead of TCP
- Faster connection setup
- No head-of-line blocking at transport level

All three use the same HTTP semantics (methods, status codes, headers).

---

# Tools You'll Use

| Tool | Purpose |
|------|---------|
| `nc` / `ncat` | Raw TCP client for manual testing |
| `curl` | HTTP client for scripting/testing |
| Browser DevTools → Network | See all real HTTP traffic |
| `python manage.py runserver` | Django dev server |
| Postman / Insomnia | GUI HTTP client |

```bash
# curl examples
curl http://localhost:8000/
curl -v http://localhost:8000/         # verbose: see headers
curl -X POST -d '{"x":1}' \
     -H "Content-Type: application/json" \
     http://localhost:8000/api/
```

---

# Browser DevTools — Network Tab

Open any website → F12 → Network tab → Reload

You can see every request:
- Status code
- Method and URL
- Request headers (what your browser sent)
- Response headers (what the server returned)
- Response body
- Timing breakdown (DNS, TCP, TLS, TTFB, download)

**Try it:** How many requests does a single page load trigger? What's the biggest file?

---

# Summary

```
You type a URL
    ↓
DNS resolves the hostname to an IP
    ↓
TCP connection opened (3-way handshake)
    ↓
Browser sends HTTP request (text)
    ↓
Server parses request → runs your code
    ↓
Server sends HTTP response (headers + body)
    ↓
Browser renders the response
```

HTTP is just formatted text over a TCP socket. You can implement a compliant server in ~50 lines of Python.

---

# Lab 1 Preview

**What you'll build:**
- A raw socket server in Python
- Parses incoming HTTP requests
- Serves different content based on the URL path
- Counts visitors correctly (ignoring `/favicon.ico`)
- Handles multiple simultaneous clients with threads

**Key milestones:**
1. TCP connection + see raw request
2. Parse path from request line
3. Build valid HTTP response
4. Visitor counter (no double-counting)
5. Thread-safe concurrent handling

**Run with:** `uv run server.py`

---

# Questions?

**Next lecture:** HTML — giving structure to the content your server will serve.

**Lab 1 is available now.** Try to get through at least Phases 1–3 before the next session.

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # install uv
```

---
