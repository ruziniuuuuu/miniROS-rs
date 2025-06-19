#!/bin/bash
set -e

VERSION=$1

if [ -z "$VERSION" ]; then
    echo "Error: Version not provided"
    exit 1
fi

echo "Publishing release version $VERSION"

# Run tests
echo "Running tests..."
cargo test --all-features

# Build release
echo "Building release..."
cargo build --release

# Publish to crates.io
if [ -n "$CRATES_IO_TOKEN" ]; then
    echo "Publishing to crates.io..."
    cargo publish --token "$CRATES_IO_TOKEN"
else
    echo "Skipping crates.io (no token)"
fi

# Build and publish Python package
if [ -f "pyproject.toml" ]; then
    echo "Building Python package..."
    pip install maturin twine
    maturin build --release --features python --out dist/
    
    if [ -n "$PYPI_TOKEN" ]; then
        echo "Publishing to PyPI..."
        twine upload dist/*.whl --username __token__ --password "$PYPI_TOKEN"
    else
        echo "Skipping PyPI (no token)"
    fi
fi

echo "Release published successfully" 