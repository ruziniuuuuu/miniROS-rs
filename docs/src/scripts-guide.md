# Scripts Guide

## 🤖 Script Architecture

miniROS provides a fully integrated script system, following the "**mini**" philosophy: minimum scripts, maximum functionality.

### 📁 Scripts Directory Structure

```
scripts/
├── setup.sh              # 🔧 Unified setup script (all-in-one)
└── mini_ros_wrapper.sh    # 🤖 Smart CLI wrapper
```

## 🚀 Main Setup Script: `setup.sh`

### 🎯 Philosophy
**One script, all miniROS needs** - Unified script handles all installation, configuration, and maintenance tasks.

### 📋 Available Commands

```bash
# Complete setup (recommended for new users)
bash scripts/setup.sh

# Show help and all commands
bash scripts/setup.sh help

# Build CLIs only (no environment configuration)
bash scripts/setup.sh build

# Setup environment only (if CLIs already built)
bash scripts/setup.sh env

# Test current installation
bash scripts/setup.sh test

# Clean build artifacts
bash scripts/setup.sh clean
```

### 🔧 What It Does

#### Complete Setup (`bash scripts/setup.sh`)
1. **🔨 Builds Rust CLI** - Compiles `mini_ros` binary
2. **🐍 Installs Python CLI** - Installs `mini_ros_py` package
3. **🔧 Configures Environment** - Adds PATH, aliases, completions
4. **📄 Backs Up Configs** - Safely backs up shell config files
5. **🧪 Tests Installation** - Verifies everything works
6. **💡 Shows Next Steps** - Guides you on usage

#### Build Only (`bash scripts/setup.sh build`)
- Builds both Rust and Python CLIs
- No environment modification
- Perfect for CI/CD or when you want manual control

#### Environment Only (`bash scripts/setup.sh env`)
- Configures shell environment (PATH, aliases, completions)
- Assumes CLIs are already built
- Useful for setting up new terminals

#### Test (`bash scripts/setup.sh test`)
- Checks if CLIs are built and functional
- Verifies environment setup
- Great for troubleshooting

#### Clean (`bash scripts/setup.sh clean`)
- Removes build artifacts
- Cleans both Rust and Python builds
- Fresh start capability

### 🛡️ Safety Features

- **Automatic Backups** - Shell configs backed up with timestamps
- **Duplicate Detection** - Won't add duplicate PATH entries
- **Error Handling** - Graceful failure with helpful messages
- **Shell Detection** - Supports bash, zsh, fish automatically

## 🤖 Smart Wrapper: `mini_ros_wrapper.sh`

### 🎯 Philosophy
**Automatic tool selection** - Uses the best CLI for each task without user intervention.

### 🧠 Smart Logic

```bash
# System operations → Rust CLI (high performance)
bash scripts/mini_ros_wrapper.sh pkg list
bash scripts/mini_ros_wrapper.sh launch turtlebot simulation

# Python operations → Python CLI (specialized)
bash scripts/mini_ros_wrapper.sh examples list
bash scripts/mini_ros_wrapper.sh test --coverage

# Fallback → Python CLI if Rust unavailable
```

### 🔍 Selection Rules

| Command | CLI Choice | Reason |
|---------|------------|--------|
| `pkg` | Rust | Package management performance |
| `launch` | Rust | System orchestration |
| `run` | Rust | Node execution performance |
| `examples` | Python | Python-specific functionality |
| `test` | Python | Python testing suite |
| `install` | Python | Python package management |
| `run --list` | Python | Python examples listing |

### 💡 Usage Examples

```bash
# These automatically choose the optimal CLI:
bash scripts/mini_ros_wrapper.sh pkg list
bash scripts/mini_ros_wrapper.sh examples list
bash scripts/mini_ros_wrapper.sh launch turtlebot simulation
bash scripts/mini_ros_wrapper.sh test --verbose
```

## 🔄 Workflow Integration

### 🏁 First Time Setup

```bash
# Clone repository
git clone https://github.com/ruziniuuuuu/miniROS-rs
cd miniROS-rs

# One-command setup
bash scripts/setup.sh

# Restart terminal or source config
source ~/.zshrc
```

### 🔄 Daily Development

```bash
# Use direct commands (post-setup)
mini_ros pkg list
mini_ros_py examples list

# Or use smart wrapper
bash scripts/mini_ros_wrapper.sh pkg info turtlebot
bash scripts/mini_ros_wrapper.sh examples install ~/my-examples
```

### 🧹 Maintenance

```bash
# Test installation
bash scripts/setup.sh test

# Clean and rebuild
bash scripts/setup.sh clean
bash scripts/setup.sh build

# Update environment (new terminal)
bash scripts/setup.sh env
```

## 🐚 Shell Support

### ✅ Fully Supported

- **bash** - `.bashrc` configuration
- **zsh** - `.zshrc` configuration  
- **fish** - `config.fish` configuration

### 🔧 What Gets Added

#### Bash/Zsh Configuration
```bash
# miniROS CLI PATH (auto-generated)
if [ -d "/path/to/miniROS-rs/target/debug" ]; then
    export PATH="/path/to/miniROS-rs/target/debug:$PATH"
fi

# miniROS conveniences
alias mrs="mini_ros"
alias mrspy="mini_ros_py"
alias mrsw="bash /path/to/miniROS-rs/scripts/mini_ros_wrapper.sh"

# miniROS completion (if available)
if command -v mini_ros >/dev/null 2>&1; then
    if [ -n "$BASH_VERSION" ]; then
        eval "$(mini_ros completions bash 2>/dev/null || true)"
    elif [ -n "$ZSH_VERSION" ]; then
        eval "$(mini_ros completions zsh 2>/dev/null || true)"
    fi
fi
```

#### Fish Configuration
```fish
# miniROS CLI PATH (auto-generated)
if test -d "/path/to/miniROS-rs/target/debug"
    set -gx PATH "/path/to/miniROS-rs/target/debug" $PATH
end

# miniROS conveniences
alias mrs="mini_ros"
alias mrspy="mini_ros_py"
alias mrsw="bash /path/to/miniROS-rs/scripts/mini_ros_wrapper.sh"
```

## 🎯 Best Practices

### 1. **Use Complete Setup First**
```bash
bash scripts/setup.sh  # Recommended for all users
```

### 2. **Test Before Using**
```bash
bash scripts/setup.sh test  # Verify installation
```

### 3. **Use Aliases for Speed**
```bash
mrs pkg list      # Short for mini_ros
mrspy examples    # Short for mini_ros_py
mrsw pkg info     # Smart wrapper
```

### 4. **Clean When Issues Arise**
```bash
bash scripts/setup.sh clean
bash scripts/setup.sh build
```

## 🐛 Troubleshooting

### Command Not Found
```bash
# Check installation
bash scripts/setup.sh test

# Reinstall environment
bash scripts/setup.sh env

# Source config manually
source ~/.zshrc
```

### Build Failures
```bash
# Clean and rebuild
bash scripts/setup.sh clean
bash scripts/setup.sh build
```

### Permission Issues
```bash
# Ensure scripts are executable
chmod +x scripts/*.sh
```

---

**Philosophy**: *One script system, unlimited robotics potential* 🤖⚡ 