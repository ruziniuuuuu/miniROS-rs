#!/bin/bash
# miniROS Python Build Script with uv
# Fast, simple, reliable

set -e

echo "🚀 Building miniROS Python bindings with uv..."

# Check if uv is installed
if ! command -v uv &> /dev/null; then
    echo "❌ uv not found. Installing..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.cargo/bin:$PATH"
fi

# Clean environment to avoid conflicts
unset VIRTUAL_ENV
unset CONDA_PREFIX

# Sync dependencies and build
echo "📦 Syncing dependencies..."
uv sync --dev

echo "🔨 Building Rust extension..."
uv run maturin develop --features python

echo "✅ Build complete! Testing import..."
uv run python -c "import mini_ros; print('miniROS ready!')"

echo "🎉 Ready to use! Try:"
echo "   uv run python python/examples/minimal_publisher.py" 