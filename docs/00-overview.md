# Overview

The **Omni-Wheel Bot (OWB)** is an open-source omnidirectional robot platform built
on Rust, `embedded-hal`, and Embassy. This site is the rendered design and
operations manual for the workspace: the same markdown files you see in the
repository are stitched together with [mdBook] for browser viewing.

If you are reading this in your IDE or on GitHub, every page is a plain
`.md` file under [`docs/`](https://github.com/Token-Thinker/omni-wheel-bot/tree/main/docs).
If you are reading the rendered site, the source link in the top-right corner
will take you back to the markdown.

## Where to start

- **Project README** at the repository root — quick start, workspace layout,
  WebSocket API.
- **Backlog** — the canonical task list. The same file feeds the GitHub Issues
  sync (`scripts/sync_backlog_issues.py`).
- **Changelog** — release history for the workspace.
- **Contributing** — how to propose changes.

## How this book is built

```
docs/                <-- markdown source (canonical)
  SUMMARY.md         <-- table of contents
  00-overview.md     <-- you are here
  backlog.md
  CHANGELOG.md
  CONTRIBUTING.md
book.toml            <-- mdBook config at the repo root
target/book/html/    <-- generated site (gitignored)
```

The `docs.yml` GitHub Actions workflow renders the book on every push to
`main` and publishes it via GitHub Pages. Pull requests build the artifact
without deploying so the rendered diff can be inspected.

[mdBook]: https://rust-lang.github.io/mdBook/
