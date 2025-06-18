# miniROS Documentation

This directory contains the source files for the miniROS documentation, built with [mdBook](https://rust-lang.github.io/mdBook/).

## 📚 Live Documentation

The documentation is automatically deployed to GitHub Pages:

**🔗 [https://ruziniuuuuu.github.io/miniROS-rs/](https://ruziniuuuuu.github.io/miniROS-rs/)**

## 🛠️ Local Development

### Prerequisites
```bash
# Install mdbook
cargo install mdbook
```

### Build and Serve
```bash
cd docs

# Serve with live reload (recommended for development)
mdbook serve --open

# Or just build static files
mdbook build
```

The documentation will be available at `http://localhost:3000`

## 📁 Structure

```
docs/
├── book.toml          # mdBook configuration
├── src/               # Markdown source files
│   ├── SUMMARY.md     # Table of contents
│   ├── introduction.md
│   ├── quick-start.md
│   ├── concepts.md
│   ├── examples.md
│   ├── python-bindings.md
│   ├── dds-transport.md
│   ├── performance.md
│   ├── api.md
│   └── visualization.md
└── book/              # Generated static site (auto-generated)
```

## 🚀 Automatic Deployment

Documentation is automatically built and deployed via GitHub Actions when:
- Changes are pushed to `main` or `develop` branches
- Changes are made to files in the `docs/` directory
- The workflow file `.github/workflows/docs.yml` is modified

## ✏️ Contributing to Documentation

1. Edit the Markdown files in `docs/src/`
2. Test locally with `mdbook serve`
3. Commit and push changes
4. GitHub Actions will automatically deploy updates

## 🎨 Customization

The documentation uses:
- **Theme**: Rust (default theme with dark mode support)
- **Search**: Enabled with full-text search
- **Print**: PDF-friendly print styles
- **Git Integration**: Links to edit pages on GitHub

To customize styling or behavior, edit `book.toml`. 