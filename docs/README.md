# miniROS-rs Documentation

This directory contains the complete documentation for miniROS-rs, built with [mdBook](https://rust-lang.github.io/mdBook/).

## Building the Documentation

### Prerequisites

Install mdBook:
```bash
cargo install mdbook
```

### Build Static Documentation

```bash
cd docs
mdbook build
```

The generated documentation will be in the `docs/book` directory.

### Serve Documentation Locally

For development and live preview:
```bash
cd docs
mdbook serve --open
```

This will start a local server at `http://localhost:3000` and automatically open your browser.

### Watch for Changes

The `mdbook serve` command automatically watches for file changes and rebuilds the documentation.

## Documentation Structure

```
docs/
├── book.toml                 # mdBook configuration
├── src/                      # Documentation source files
│   ├── SUMMARY.md           # Table of contents
│   ├── introduction.md      # Project introduction
│   ├── getting-started/     # Getting started guides
│   │   ├── installation.md
│   │   ├── quick-start.md
│   │   └── examples.md
│   ├── core-concepts/       # Core concepts documentation
│   ├── communication/       # Communication layer docs
│   ├── visualization/       # Visualization documentation
│   │   ├── rerun.md        # Rerun integration guide
│   │   ├── data-types.md   # Supported data types
│   │   └── examples.md     # Visualization examples
│   ├── advanced/           # Advanced topics
│   ├── api/               # API reference
│   └── development/       # Development guides
└── book/                  # Generated static site (git-ignored)
```

## Key Features

### English Documentation
- Complete English documentation for international accessibility
- Professional technical writing style
- Comprehensive examples and tutorials

### Visualization Integration
The documentation includes detailed coverage of the Rerun visualization integration:
- Setup and configuration guides
- Data type documentation
- Complete working examples
- Performance considerations
- Troubleshooting guides

### Interactive Features
- **Search**: Full-text search across all documentation
- **Syntax Highlighting**: Code blocks with Rust syntax highlighting
- **Navigation**: Easy navigation with sidebar and breadcrumbs
- **Print-Friendly**: Optimized for printing and PDF export
- **Responsive**: Works well on mobile and desktop

## Contributing to Documentation

### Adding New Pages

1. Create new `.md` files in the appropriate directory
2. Add entries to `src/SUMMARY.md` to include them in navigation
3. Use relative links for cross-references

### Documentation Guidelines

- Use clear, concise English
- Include working code examples
- Add appropriate section headers for navigation
- Link to related sections where relevant
- Keep examples up-to-date with the codebase

### Code Examples

Code examples should be:
- Complete and runnable
- Well-commented
- Following Rust best practices
- Including error handling

Example format:
```rust
//! Brief description of what this example demonstrates
//! Run with: cargo run --example example_name

use mini_ros::*;

#[tokio::main]
async fn main() -> mini_ros::error::Result<()> {
    // Your example code here
    Ok(())
}
```

## Deployment

### GitHub Pages

The documentation can be automatically deployed to GitHub Pages:

1. Build the documentation: `mdbook build`
2. Deploy the `book/` directory to GitHub Pages
3. Set up GitHub Actions for automatic deployment

### Custom Hosting

The generated `book/` directory contains a complete static website that can be hosted anywhere:
- Apache/Nginx
- Netlify/Vercel
- AWS S3 + CloudFront
- Any static hosting service

## Configuration

Key settings in `book.toml`:

```toml
[book]
title = "miniROS-rs Documentation"
authors = ["Chenyu Cao <ruziniuuuuu@gmail.com>"]
language = "en"

[output.html]
default-theme = "rust"
git-repository-url = "https://github.com/your-username/miniROS-rs"
```

## Maintenance

### Keeping Documentation Current

- Update examples when APIs change
- Review and update version numbers
- Test all code examples regularly
- Update external links periodically

### Performance

The documentation is optimized for:
- Fast loading times
- Efficient search indexing
- Mobile responsiveness
- SEO-friendly structure

## Links

- [mdBook Documentation](https://rust-lang.github.io/mdBook/)
- [miniROS-rs Repository](https://github.com/your-username/miniROS-rs)
- [Rerun Visualization](https://rerun.io/docs)
- [Rust Documentation Guidelines](https://doc.rust-lang.org/rustdoc/) 