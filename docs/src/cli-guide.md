# CLI Usage Guide

## 🤖 Two CLI Versions: Smart Design

miniROS provides two complementary CLI tools, following the "**mini**" philosophy: core functionality, minimum complexity.

### 🦀 Rust CLI: `mini_ros`
**Primary Choice - High-Performance System Operations**

```bash
# Core functionality - fast and reliable
mini_ros launch turtlebot simulation
mini_ros run turtlebot controller  
mini_ros pkg list
```

**Use Cases:**
- Production deployments
- High-performance requirements
- Complex system orchestration
- Package management and launch files

### 🐍 Python CLI: `mini_ros_py`
**Python Ecosystem Specialized Tool**

```bash
# Python-specific operations
mini_ros_py run minimal_publisher
mini_ros_py examples list
mini_ros_py test --coverage
```

**Use Cases:**
- Python development and testing
- Example code management
- Rapid prototyping
- Teaching and learning

## 🚀 Quick Start

### 1. Build Both CLIs
```bash
# Build Rust CLI (recommended first)
bash scripts/setup.sh

# Or manual build
cargo build --bin mini_ros
cd python && pip install -e .
```

### 2. Verify Installation
```bash
# Test Rust CLI
mini_ros --help

# Test Python CLI
mini_ros_py --help
```

### 3. Smart Wrapper (Optional)
```bash
# Use universal wrapper that auto-selects best CLI
bash scripts/mini_ros_wrapper.sh pkg list
```

## 📋 Command Comparison

| Feature | Rust (`mini_ros`) | Python (`mini_ros_py`) |
|---------|-------------------|------------------------|
| **Package Management** | ✅ Full support | ❌ Read-only |
| **Launch System** | ✅ Complete | ❌ Not available |
| **Node Execution** | ✅ Rust + Python | ✅ Python only |
| **Examples** | ❌ Basic | ✅ Advanced management |
| **Testing** | ❌ Not available | ✅ Full test suite |
| **Performance** | 🚀 Optimized | 🐍 Standard |
| **Completions** | ✅ All shells | ⚠️ Basic |

## 🔧 Advanced Usage

### Rust CLI Features

#### Package Management
```bash
# List all packages with details
mini_ros pkg list

# Create new package with Python support
mini_ros pkg create my_robot --path ./packages --python

# Show detailed package information
mini_ros pkg info turtlebot
```

#### Launch System
```bash
# Launch complete systems
mini_ros launch turtlebot full_system

# Launch with custom arguments
mini_ros launch my_pkg simulation --args debug verbose
```

#### Shell Completions
```bash
# Generate completions for your shell
mini_ros completions bash > ~/.bash_completion
mini_ros completions zsh > ~/.zsh_completions/_mini_ros
```

### Python CLI Features

#### Example Management
```bash
# List available Python examples
mini_ros_py examples list

# Run specific example with arguments
mini_ros_py run turtlebot_controller --args --speed 0.5

# Install examples to custom directory
mini_ros_py examples install ~/my_examples
```

#### Testing and Development
```bash
# Run full test suite with coverage
mini_ros_py test --coverage --verbose

# Run specific test pattern
mini_ros_py test test_basic

# Install package in development mode
mini_ros_py install --dev
```

## ⚡ Performance Tips

### When to Use Rust CLI
- Production deployments
- Multi-node systems
- High-frequency operations
- System integration

### When to Use Python CLI
- Python development workflow
- Testing and validation
- Learning and tutorials
- Quick prototyping

## 🐛 Troubleshooting

### Common Issues

#### CLI Not Found
```bash
# Rust CLI
bash scripts/setup.sh build
export PATH="$PWD/target/debug:$PATH"

# Python CLI
cd python && pip install -e .
```

#### Permission Errors
```bash
# Make scripts executable
chmod +x target/debug/mini_ros
chmod +x scripts/*.sh
```

#### Python Module Not Found
```bash
# Ensure Python package is installed
cd python && pip install -e .

# Check if mini_ros module is available
python -c "import mini_ros; print('✅ Available')"
```

### Environment Variables

```bash
# Control logging level
export MINI_ROS_LOG=debug

# Override default package paths
export MINI_ROS_PACKAGE_PATH=/path/to/packages
```

## 🎯 Best Practices

### 1. **Use the Right Tool**
- Rust CLI for system operations
- Python CLI for Python development

### 2. **Setup Shell Completions**
```bash
# Add to your shell config
mini_ros completions $(basename $SHELL) > ~/.completion_mini_ros
source ~/.completion_mini_ros
```

### 3. **Alias for Convenience**
```bash
# Add to ~/.bashrc or ~/.zshrc
alias mrs="mini_ros"
alias mrspy="mini_ros_py"
alias mrsw="bash scripts/mini_ros_wrapper.sh"
```

### 4. **Development Workflow**
```bash
# Start with Python for rapid development
mini_ros_py run my_example

# Move to Rust for production
mini_ros run my_pkg my_node
```

## 📚 Examples

### Complete Development Cycle
```bash
# 1. Create package (Rust CLI)
mini_ros pkg create my_robot --python

# 2. Develop with Python
mini_ros_py run minimal_publisher

# 3. Test thoroughly
mini_ros_py test --coverage

# 4. Deploy with Rust
mini_ros launch my_robot production
```

### Cross-Language Development
```bash
# Run Rust node
mini_ros run turtlebot controller

# In another terminal, run Python utilities
mini_ros_py run turtlebot_monitor

# Both communicate via miniROS middleware
```

---

**Philosophy**: *Maximum robotics performance, minimum complexity* 🤖⚡ 