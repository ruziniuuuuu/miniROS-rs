# miniROS-rs Documentation

This directory contains the source files for the miniROS-rs documentation, built with [mdBook](https://rust-lang.github.io/mdBook/).

## 📚 Live Documentation

The documentation is available in multiple formats:

- **🔗 [Official Documentation](https://ruziniuuuuu.github.io/miniROS-rs/)** - Complete API reference and guides (GitHub Pages)
- **📖 [DeepWiki](https://deepwiki.com/ruziniuuuuu/miniROS-rs)** - Community-driven knowledge base with tutorials and examples

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
├── book.toml              # mdBook configuration
├── src/                   # Markdown source files
│   ├── SUMMARY.md         # Table of contents
│   ├── introduction.md    # Complete system overview
│   ├── quick-start.md     # 5-minute tutorial
│   ├── concepts.md        # Core concepts and architecture
│   ├── examples.md        # Progressive learning examples
│   ├── python-bindings.md # ROS2 rclpy compatible Python API
│   ├── visualization.md   # Built-in 3D visualization
│   ├── api.md            # Complete API reference
│   ├── transport-architecture.md # Multi-transport design
│   ├── dds-transport.md   # ROS2 compatibility
│   ├── performance.md     # Benchmarks and optimization
│   └── cicd.md           # Development automation
└── book/                  # Generated static site (auto-generated)
```

## 📖 Documentation Overview

### **Complete Robotics Middleware**
The documentation now reflects miniROS-rs as a **comprehensive** robotics middleware with:

- **Complete ROS2 Patterns**: Pub/Sub, Services, Actions, Parameters
- **3D Visualization**: Built-in Rerun integration
- **Multiple Transports**: DDS, TCP, Zenoh support
- **Python Compatibility**: ROS2 rclpy drop-in replacement
- **Production Ready**: Real-world robotics system support

### **Updated Content**

#### 🚀 **[Introduction](src/introduction.md)**
- Complete feature overview
- Performance comparisons with ROS2
- When to use miniROS-rs vs ROS2
- Architecture diagrams

#### ⚡ **[Quick Start](src/quick-start.md)**
- 5-minute complete system demo
- Rust and Python examples
- All communication patterns
- Transport options
- Visualization basics

#### 📚 **[Examples](src/examples.md)**
- 8 progressive learning examples
- Rust and Python code
- Complete system integration
- Learning paths for different experience levels

#### 🐍 **[Python Bindings](src/python-bindings.md)**
- ROS2 rclpy compatibility
- Migration guide from ROS2
- Complete API reference
- Advanced patterns and best practices

#### 🎨 **[Visualization](src/visualization.md)**
- Built-in Rerun 3D visualization
- Robotics-specific features
- Real-time data streaming
- Integration examples

### **Key Updates**

1. **Comprehensive Features**: Updated from "minimal" to "complete" robotics middleware
2. **ROS2 Compatibility**: Emphasis on ROS2 rclpy API compatibility
3. **Visualization Integration**: Built-in 3D visualization documentation
4. **Multiple Transports**: DDS, TCP, Zenoh transport options
5. **Production Examples**: Real-world system integration patterns
6. **Performance Focus**: Benchmarks and optimization guides

## 🚀 Automatic Deployment

Documentation is automatically built and deployed via GitHub Actions when:
- Changes are pushed to `main` or `develop` branches
- Changes are made to files in the `docs/` directory
- The workflow file `.github/workflows/docs.yml` is modified

## ✏️ Contributing to Documentation

### Content Guidelines

1. **Complete Coverage**: Document all features and APIs
2. **Progressive Learning**: Start simple, build complexity
3. **Real Examples**: Use actual working code
4. **ROS2 Compatibility**: Emphasize familiar patterns
5. **Performance Focus**: Include benchmarks and tips

### Writing Style

- **Clear and Concise**: Easy to understand explanations
- **Code-Heavy**: Show don't just tell
- **Beginner Friendly**: Assume basic robotics knowledge only
- **Production Ready**: Include real-world considerations

### Adding New Content

1. Edit the Markdown files in `docs/src/`
2. Update `SUMMARY.md` if adding new pages
3. Test locally with `mdbook serve`
4. Ensure all code examples work
5. Commit and push changes
6. GitHub Actions will automatically deploy updates

## 🎨 Customization

The documentation uses:
- **Theme**: Rust (default theme with dark mode support)
- **Search**: Enabled with full-text search
- **Print**: PDF-friendly print styles
- **Git Integration**: Links to edit pages on GitHub
- **Syntax Highlighting**: Rust and Python code highlighting

To customize styling or behavior, edit `book.toml`.

## 📊 Documentation Metrics

The documentation covers:
- **8 Progressive Examples**: From basics to complete systems
- **Complete API Reference**: All public APIs documented
- **Multiple Languages**: Rust and Python with ROS2 compatibility
- **Visual Learning**: Built-in 3D visualization examples
- **Production Patterns**: Real-world system architecture

## 🔧 Development Workflow

### Local Testing
```bash
# Test all code examples
cd docs && ./test-examples.sh

# Check links
mdbook test

# Serve with live reload
mdbook serve --open
```

### Update Checklist

When updating documentation:
- [ ] All code examples are tested and working
- [ ] API changes are reflected in all relevant pages  
- [ ] Python examples maintain ROS2 compatibility
- [ ] Cross-references between pages are updated
- [ ] SUMMARY.md is updated if needed
- [ ] Performance benchmarks are current

## 📈 Future Documentation Plans

- [ ] **Interactive Examples**: Embedded code playground
- [ ] **Video Tutorials**: Step-by-step visual guides
- [ ] **Use Case Studies**: Real robotics deployments
- [ ] **Advanced Patterns**: Complex system architectures
- [ ] **Troubleshooting Guide**: Common issues and solutions
- [ ] **API Stability Guide**: Migration between versions

The documentation serves as both **learning resource** and **production reference** for building complete robotics systems with miniROS-rs! 📖🤖 