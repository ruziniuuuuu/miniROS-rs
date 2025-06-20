#!/bin/bash
# miniROS Unified Setup Script
# Philosophy: Minimal setup, maximum convenience

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# miniROS project root
MINIROS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo -e "${BLUE}🤖 miniROS Unified Setup${NC}"
echo -e "${BLUE}Philosophy: Maximum robotics performance, minimum complexity${NC}"
echo ""

# Check if script is run from correct directory
if [ ! -f "$MINIROS_ROOT/Cargo.toml" ]; then
    echo -e "${RED}❌ Please run this script from the miniROS-rs project root${NC}"
    echo -e "${YELLOW}💡 Usage: bash scripts/setup.sh${NC}"
    exit 1
fi

# Function to show help
show_help() {
    echo -e "${BLUE}📋 Available commands:${NC}"
    echo ""
    echo -e "  ${GREEN}bash scripts/setup.sh${NC}              - Complete setup (CLI + environment)"
    echo -e "  ${GREEN}bash scripts/setup.sh build${NC}        - Build CLIs only"
    echo -e "  ${GREEN}bash scripts/setup.sh env${NC}          - Setup environment only"
    echo -e "  ${GREEN}bash scripts/setup.sh test${NC}         - Test installation"
    echo -e "  ${GREEN}bash scripts/setup.sh clean${NC}        - Clean build artifacts"
    echo -e "  ${GREEN}bash scripts/setup.sh help${NC}         - Show this help"
    echo ""
    echo -e "${YELLOW}💡 Philosophy: One script, all miniROS needs${NC}"
}

# Function to detect shell
detect_shell() {
    local shell_name=$(basename "$SHELL")
    echo "$shell_name"
}

# Function to get shell config file
get_shell_config() {
    local shell_name="$1"
    case "$shell_name" in
        "bash")
            echo "$HOME/.bashrc"
            ;;
        "zsh")
            echo "$HOME/.zshrc"
            ;;
        "fish")
            echo "$HOME/.config/fish/config.fish"
            ;;
        *)
            echo "$HOME/.profile"
            ;;
    esac
}

# Function to add PATH export for bash/zsh
add_bash_zsh_path() {
    local config_file="$1"
    local miniros_path="$MINIROS_ROOT/target/debug"
    
    # Create backup
    if [ -f "$config_file" ]; then
        cp "$config_file" "$config_file.backup.$(date +%Y%m%d_%H%M%S)"
        echo -e "${YELLOW}📄 Backed up $config_file${NC}"
    fi
    
    # Check if already added
    if grep -q "miniROS CLI PATH" "$config_file" 2>/dev/null; then
        echo -e "${YELLOW}⚠️  miniROS CLI PATH already configured in $config_file${NC}"
        return 0
    fi
    
    # Add PATH configuration
    cat >> "$config_file" << EOF

# miniROS CLI PATH (auto-generated $(date))
if [ -d "$miniros_path" ]; then
    export PATH="$miniros_path:\$PATH"
fi

# miniROS conveniences
alias mrs="mini_ros"
alias mrspy="mini_ros_py"
alias mrsw="bash $MINIROS_ROOT/scripts/mini_ros_wrapper.sh"

# miniROS completion (if available)
if command -v mini_ros >/dev/null 2>&1; then
    # Auto-load completions if supported
    if [ -n "\$BASH_VERSION" ]; then
        eval "\$(mini_ros completions bash 2>/dev/null || true)"
    elif [ -n "\$ZSH_VERSION" ]; then
        eval "\$(mini_ros completions zsh 2>/dev/null || true)"
    fi
fi
EOF
    
    echo -e "${GREEN}✅ Added miniROS CLI to $config_file${NC}"
}

# Function to add PATH export for fish
add_fish_path() {
    local config_file="$1"
    local miniros_path="$MINIROS_ROOT/target/debug"
    
    # Create fish config directory if it doesn't exist
    mkdir -p "$(dirname "$config_file")"
    
    # Create backup
    if [ -f "$config_file" ]; then
        cp "$config_file" "$config_file.backup.$(date +%Y%m%d_%H%M%S)"
        echo -e "${YELLOW}📄 Backed up $config_file${NC}"
    fi
    
    # Check if already added
    if grep -q "miniROS CLI PATH" "$config_file" 2>/dev/null; then
        echo -e "${YELLOW}⚠️  miniROS CLI PATH already configured in $config_file${NC}"
        return 0
    fi
    
    # Add PATH configuration for fish
    cat >> "$config_file" << EOF

# miniROS CLI PATH (auto-generated $(date))
if test -d "$miniros_path"
    set -gx PATH "$miniros_path" \$PATH
end

# miniROS conveniences
alias mrs="mini_ros"
alias mrspy="mini_ros_py"
alias mrsw="bash $MINIROS_ROOT/scripts/mini_ros_wrapper.sh"
EOF
    
    echo -e "${GREEN}✅ Added miniROS CLI to $config_file${NC}"
}

# Function to build Rust CLI
build_rust_cli() {
    echo -e "${BLUE}🔨 Building Rust CLI...${NC}"
    cd "$MINIROS_ROOT"
    
    if ! command -v cargo >/dev/null 2>&1; then
        echo -e "${RED}❌ Cargo not found. Please install Rust first: https://rustup.rs/${NC}"
        exit 1
    fi
    
    cargo build --bin mini_ros
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Rust CLI built successfully${NC}"
    else
        echo -e "${RED}❌ Failed to build Rust CLI${NC}"
        exit 1
    fi
}

# Function to install Python CLI
install_python_cli() {
    echo -e "${BLUE}🐍 Installing Python CLI...${NC}"
    cd "$MINIROS_ROOT/python"
    
    if ! command -v python3 >/dev/null 2>&1 && ! command -v python >/dev/null 2>&1; then
        echo -e "${RED}❌ Python not found. Please install Python first.${NC}"
        exit 1
    fi
    
    # Try to install with pip
    if command -v pip3 >/dev/null 2>&1; then
        pip3 install -e .
    elif command -v pip >/dev/null 2>&1; then
        pip install -e .
    else
        echo -e "${YELLOW}⚠️  pip not found, trying with python -m pip${NC}"
        python -m pip install -e .
    fi
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✅ Python CLI installed successfully${NC}"
    else
        echo -e "${YELLOW}⚠️  Python CLI installation had issues (might still work)${NC}"
    fi
    
    cd "$MINIROS_ROOT"
}

# Function to setup environment
setup_environment() {
    echo -e "${BLUE}🔧 Configuring shell environment...${NC}"
    
    # Detect shell
    SHELL_NAME=$(detect_shell)
    CONFIG_FILE=$(get_shell_config "$SHELL_NAME")
    
    echo -e "${BLUE}Shell: $SHELL_NAME${NC}"
    echo -e "${BLUE}Config: $CONFIG_FILE${NC}"
    
    # Configure shell
    case "$SHELL_NAME" in
        "fish")
            add_fish_path "$CONFIG_FILE"
            ;;
        *)
            add_bash_zsh_path "$CONFIG_FILE"
            ;;
    esac
}

# Function to test installation
test_installation() {
    echo -e "${BLUE}🧪 Testing installation...${NC}"
    
    # Test Rust CLI
    if [ -f "$MINIROS_ROOT/target/debug/mini_ros" ]; then
        echo -e "${GREEN}✅ Rust CLI binary found${NC}"
    else
        echo -e "${RED}❌ Rust CLI binary not found${NC}"
        return 1
    fi
    
    # Test Python CLI
    if command -v mini_ros_py >/dev/null 2>&1; then
        echo -e "${GREEN}✅ Python CLI command available${NC}"
    else
        echo -e "${YELLOW}⚠️  Python CLI command not found in PATH${NC}"
    fi
    
    # Test with actual commands
    echo -e "${BLUE}🔍 Testing CLI functionality...${NC}"
    
    if "$MINIROS_ROOT/target/debug/mini_ros" --version >/dev/null 2>&1; then
        echo -e "${GREEN}✅ Rust CLI functional${NC}"
    else
        echo -e "${RED}❌ Rust CLI not functional${NC}"
    fi
    
    if mini_ros_py --version >/dev/null 2>&1; then
        echo -e "${GREEN}✅ Python CLI functional${NC}"
    else
        echo -e "${YELLOW}⚠️  Python CLI not functional${NC}"
    fi
    
    return 0
}

# Function to clean build artifacts
clean_build() {
    echo -e "${BLUE}🧹 Cleaning build artifacts...${NC}"
    cd "$MINIROS_ROOT"
    
    # Clean Rust build
    if [ -d "target" ]; then
        cargo clean
        echo -e "${GREEN}✅ Cleaned Rust build artifacts${NC}"
    fi
    
    # Clean Python build
    if [ -d "python/build" ]; then
        rm -rf python/build
        echo -e "${GREEN}✅ Cleaned Python build artifacts${NC}"
    fi
    
    if [ -d "python/mini_ros_python.egg-info" ]; then
        rm -rf python/mini_ros_python.egg-info
        echo -e "${GREEN}✅ Cleaned Python egg-info${NC}"
    fi
    
    echo -e "${GREEN}🎉 Cleanup completed!${NC}"
}

# Function to show completion message
show_completion() {
    echo ""
    echo -e "${GREEN}🎉 Setup completed!${NC}"
    echo ""
    echo -e "${YELLOW}📋 Next steps:${NC}"
    echo -e "  1. Restart your terminal or run: ${BLUE}source $(get_shell_config $(detect_shell))${NC}"
    echo -e "  2. Test with: ${BLUE}mini_ros --help${NC}"
    echo -e "  3. Try: ${BLUE}mini_ros pkg list${NC}"
    echo -e "  4. Or: ${BLUE}mini_ros_py examples list${NC}"
    echo ""
    echo -e "${YELLOW}💡 Available commands:${NC}"
    echo -e "  • ${BLUE}mini_ros${NC} - Main CLI (Rust, high performance)"
    echo -e "  • ${BLUE}mini_ros_py${NC} - Python CLI (examples, testing)"
    echo -e "  • ${BLUE}mrs${NC} - Short alias for mini_ros"
    echo -e "  • ${BLUE}mrspy${NC} - Short alias for mini_ros_py"
    echo -e "  • ${BLUE}mrsw${NC} - Universal wrapper (auto-selects best CLI)"
    echo ""
    echo -e "${GREEN}Philosophy: Maximum robotics performance, minimum complexity 🤖⚡${NC}"
}

# Main execution logic
case "${1:-all}" in
    "help"|"-h"|"--help")
        show_help
        ;;
    "build")
        echo -e "${BLUE}🔨 Building CLIs only...${NC}"
        build_rust_cli
        install_python_cli
        echo -e "${GREEN}✅ Build completed!${NC}"
        ;;
    "env")
        echo -e "${BLUE}🔧 Setting up environment only...${NC}"
        setup_environment
        echo -e "${GREEN}✅ Environment setup completed!${NC}"
        ;;
    "test")
        echo -e "${BLUE}🧪 Testing installation...${NC}"
        test_installation
        ;;
    "clean")
        clean_build
        ;;
    "all"|"")
        echo -e "${BLUE}🚀 Running complete setup...${NC}"
        echo ""
        
        # Build CLIs
        build_rust_cli
        install_python_cli
        
        echo ""
        
        # Setup environment
        setup_environment
        
        echo ""
        
        # Test installation
        test_installation
        
        # Show completion message
        show_completion
        ;;
    *)
        echo -e "${RED}❌ Unknown command: $1${NC}"
        echo ""
        show_help
        exit 1
        ;;
esac 