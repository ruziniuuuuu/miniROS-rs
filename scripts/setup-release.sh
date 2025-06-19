#!/bin/bash

echo "🚀 Setup miniROS-rs Release System"
echo "==================================="

# Check tools
echo "📋 Checking environment..."

if ! command -v git &> /dev/null; then
    echo "❌ Git not installed"
    exit 1
fi

if ! command -v cargo &> /dev/null; then
    echo "❌ Rust/Cargo not installed"
    exit 1
fi

echo "✅ Environment check passed"

# Check if in git repo
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo "❌ Please run this script in the Git repository root"
    exit 1
fi

echo "✅ Git repository check passed"

# Check required files
FILES_TO_CHECK=(".github/workflows/ci.yml" ".github/workflows/release.yml")
for file in "${FILES_TO_CHECK[@]}"; do
    if [ ! -f "$file" ]; then
        echo "❌ Missing file: $file"
        exit 1
    fi
done

echo "✅ CI/CD configuration files check passed"

# Get current version
CURRENT_VERSION=$(grep '^version = ' Cargo.toml | head -1 | sed 's/version = "\(.*\)"/\1/')
echo "📦 Current version: $CURRENT_VERSION"

echo ""
echo "🔧 Next steps:"
echo "1. Add GitHub Secrets:"
echo "   - CRATES_IO_TOKEN (for crates.io publishing)"
echo "   - PYPI_TOKEN (for PyPI publishing)"
echo ""
echo "2. Usage:"
echo "   • Push to main branch for CI testing"
echo "   • Create git tag 'v1.2.3' to trigger release"
echo "   • Or manually trigger release workflow in GitHub Actions"
echo ""
echo "🎉 Setup complete!" 