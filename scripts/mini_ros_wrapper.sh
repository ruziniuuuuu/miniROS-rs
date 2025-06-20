#!/bin/bash
# miniROS Universal CLI Wrapper (Bash)
# Smart wrapper that automatically selects the best available miniROS CLI

# miniROS project root
MINIROS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Colors for output  
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🤖 miniROS Universal CLI${NC}"

# Function to check if Rust CLI is available
find_rust_cli() {
    # Check in current directory target/debug
    if [ -f "$MINIROS_ROOT/target/debug/mini_ros" ]; then
        echo "$MINIROS_ROOT/target/debug/mini_ros"
        return 0
    fi
    
    # Check in PATH
    if command -v mini_ros >/dev/null 2>&1; then
        echo "mini_ros"
        return 0
    fi
    
    return 1
}

# Function to check if Python CLI is available  
find_python_cli() {
    if command -v mini_ros_py >/dev/null 2>&1; then
        return 0
    fi
    return 1
}

# Function to determine if Python CLI should be used
should_use_python_cli() {
    local args=("$@")
    
    if [ ${#args[@]} -eq 0 ]; then
        return 1
    fi
    
    # Python-specific commands
    case "${args[0]}" in
        "examples"|"test"|"install")
            return 0
            ;;
        "run")
            # Check for Python-specific flags in run command
            if [ ${#args[@]} -gt 1 ] && [[ "${args[1]}" == "--list" || "${args[1]}" == "-l" ]]; then
                return 0
            fi
            ;;
    esac
    
    return 1
}

# Main logic
args=("$@")

# Determine which CLI to use
if rust_cli_path=$(find_rust_cli); then
    rust_available=true
else
    rust_available=false
fi

if find_python_cli; then
    python_available=true
else
    python_available=false
fi

if should_use_python_cli "${args[@]}" && [ "$python_available" = true ]; then
    echo -e "${GREEN}🐍 Using Python CLI for Python-specific operations${NC}"
    exec mini_ros_py "${args[@]}"
elif [ "$rust_available" = true ]; then
    echo -e "${GREEN}⚡ Using Rust CLI for optimal performance${NC}"
    exec "$rust_cli_path" "${args[@]}"
elif [ "$python_available" = true ]; then
    echo -e "${GREEN}🐍 Falling back to Python CLI${NC}"
    exec mini_ros_py "${args[@]}"
else
    echo -e "${RED}❌ No miniROS CLI found!${NC}"
    echo ""
    echo -e "${YELLOW}To install:${NC}"
    echo -e "  Rust CLI:   ${BLUE}bash scripts/setup.sh build${NC}"
    echo -e "  Python CLI: ${BLUE}bash scripts/setup.sh build${NC}"
    echo -e "  Complete:   ${BLUE}bash scripts/setup.sh${NC}"
    exit 1
fi 