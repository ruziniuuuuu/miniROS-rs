# Installation

This guide will help you get miniROS-rs up and running on your system.

## Prerequisites

### Rust Installation

miniROS-rs requires Rust 2024 edition. Install Rust using rustup:

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
source ~/.cargo/env
```

Verify your installation:
```bash
rustc --version
cargo --version
```

### System Dependencies

miniROS-rs has minimal system dependencies, but some optional features require additional libraries:

#### Ubuntu/Debian
```bash
sudo apt update
sudo apt install build-essential pkg-config
```

#### macOS
```bash
# Install Xcode command line tools
xcode-select --install

# Or use Homebrew
brew install pkg-config
```

#### Windows
- Install [Visual Studio Build Tools](https://visualstudio.microsoft.com/downloads/#build-tools-for-visual-studio-2019)
- Or use [rustup](https://rustup.rs/) installer which handles most requirements

## Installing miniROS-rs

### From Source (Recommended)

Clone the repository and build:

```bash
git clone https://github.com/your-username/miniROS-rs.git
cd miniROS-rs
cargo build --release
```

### Using Cargo (Coming Soon)

Once published to crates.io:

```bash
cargo install mini-ros
```

### Adding to Your Project

Add miniROS-rs to your `Cargo.toml`:

```toml
[dependencies]
mini-ros = "0.1"

# Optional features
mini-ros = { version = "0.1", features = ["visualization", "zenoh"] }
```

## Feature Flags

miniROS-rs supports optional features you can enable based on your needs:

```toml
[dependencies]
mini-ros = { version = "0.1", features = [
    "visualization",  # Rerun visualization support
    "zenoh",         # Zenoh transport (high-performance)
    "full",          # All features enabled
] }
```

### Available Features

| Feature | Description | Dependencies |
|---------|-------------|--------------|
| `default` | Core functionality only | Minimal |
| `visualization` | Rerun integration | rerun, additional rendering libs |
| `zenoh` | Zenoh transport | zenoh (when available) |
| `full` | All features enabled | All optional dependencies |

## Optional Components

### Rerun Viewer (For Visualization)

If you plan to use real-time visualization, install the Rerun viewer:

```bash
# Option 1: Using cargo (recommended)
cargo install rerun-cli

# Option 2: Using pip
pip install rerun-sdk==0.20.3

# Option 3: Download from releases
# Visit: https://github.com/rerun-io/rerun/releases/0.20.3/
```

### Development Tools

For development and testing:

```bash
# Install additional tools
cargo install cargo-watch    # Auto-rebuild on changes
cargo install cargo-nextest  # Better test runner
```

## Verification

Verify your installation by running the examples:

```bash
# Clone the repository if you haven't already
git clone https://github.com/your-username/miniROS-rs.git
cd miniROS-rs

# Run basic test
cargo run --example basic_test

# Run simple pub/sub example
cargo run --example simple_pub_sub

# Test visualization (requires Rerun viewer for real-time mode)
cargo run --example visualization_demo

# Test Zenoh integration
cargo run --example zenoh_communication
```

Expected output for basic test:
```
INFO basic_test: Starting basic miniROS test
INFO basic_test: Node 'test_node' created successfully
INFO basic_test: Publisher created for topic: /test_topic
INFO basic_test: Subscriber created for topic: /test_topic
INFO basic_test: Published message: Hello, miniROS!
INFO basic_test: Test completed successfully
```

## Common Issues

### Rust Version

If you encounter compilation errors, ensure you're using a recent Rust version:

```bash
rustup update
rustc --version  # Should be 1.75+ for Rust 2024 edition
```

### Network Permissions

Some examples use UDP multicast. On restrictive networks:

```bash
# Linux: May need to allow multicast
sudo iptables -A INPUT -d 224.0.0.0/8 -j ACCEPT

# macOS: Usually works by default
# Windows: May need firewall configuration
```

### Rerun Viewer Not Found

If visualization examples fail with "Rerun Viewer executable not found":

1. Install the Rerun viewer (see above)
2. Ensure it's in your PATH
3. Or use buffered mode (default in examples)

### Build Errors

For build issues:

```bash
# Clean and rebuild
cargo clean
cargo build

# Update dependencies
cargo update

# Check for system dependencies
pkg-config --version  # Should be available
```

## Cross-Platform Notes

### Linux
- Works on most distributions
- No special configuration needed
- Good performance

### macOS
- Supports both Intel and Apple Silicon
- May need Xcode command line tools
- Excellent performance

### Windows
- Requires Visual Studio Build Tools
- WSL2 recommended for development
- Some network features may need configuration

## Next Steps

- Follow the [Quick Start](./quick-start.md) guide to create your first miniROS-rs application
- Explore [Examples](./examples.md) for common usage patterns
- Check out the [Architecture Overview](../core-concepts/architecture.md) to understand the system design

## Getting Help

If you encounter issues:

1. Check the [GitHub Issues](https://github.com/your-username/miniROS-rs/issues)
2. Review the [Troubleshooting Guide](../advanced/error-handling.md)
3. Join our [community discussions](https://github.com/your-username/miniROS-rs/discussions) 