#!/bin/bash
set -e

# Version synchronization script for miniROS-rs
# Usage: ./scripts/sync-version.sh [version]
# If no version provided, extracts from Cargo.toml

VERSION=${1:-""}

# Function to get current version from Cargo.toml
get_current_version() {
    grep '^version = ' Cargo.toml | head -1 | sed 's/version = "\(.*\)"/\1/'
}

# Function to update version in a file
update_version_in_file() {
    local file=$1
    local version=$2
    local pattern=$3
    local replacement=$4
    
    if [ -f "$file" ]; then
        sed -i.bak "$pattern" "$file"
        rm -f "$file.bak"
        echo "✅ Updated $file"
    else
        echo "⚠️  File not found: $file"
    fi
}

# If no version provided, use current Cargo.toml version
if [ -z "$VERSION" ]; then
    VERSION=$(get_current_version)
    echo "📦 Using current version from Cargo.toml: $VERSION"
else
    echo "📦 Syncing to version: $VERSION"
fi

# Validate version format
if ! [[ "$VERSION" =~ ^[0-9]+\.[0-9]+\.[0-9]+(-[a-zA-Z0-9]+)?$ ]]; then
    echo "❌ Invalid version format: $VERSION"
    echo "Expected format: x.y.z or x.y.z-suffix"
    exit 1
fi

echo "🔄 Synchronizing version across all files..."

# Update Cargo.toml
update_version_in_file "Cargo.toml" "$VERSION" "s/^version = \".*\"/version = \"$VERSION\"/"

# Update pyproject.toml (handle both static and dynamic version)
if grep -q 'dynamic = \["version"\]' pyproject.toml 2>/dev/null; then
    # Replace dynamic version with static version
    update_version_in_file "pyproject.toml" "$VERSION" "s/dynamic = \\[\"version\"\\]/version = \"$VERSION\"/"
else
    # Update existing static version
    update_version_in_file "pyproject.toml" "$VERSION" "s/^version = \".*\"/version = \"$VERSION\"/"
fi

# Update Cargo.lock
echo "🔄 Updating Cargo.lock..."
cargo check --quiet

# Update Python package version if __init__.py exists
if [ -f "python/mini_ros/__init__.py" ]; then
    update_version_in_file "python/mini_ros/__init__.py" "$VERSION" "s/__version__ = \".*\"/__version__ = \"$VERSION\"/"
fi

echo ""
echo "🎉 Version synchronization complete!"
echo "📋 Summary:"
echo "   Version: $VERSION"
echo "   Files updated:"
echo "   - Cargo.toml"
echo "   - pyproject.toml"
echo "   - Cargo.lock"
if [ -f "python/mini_ros/__init__.py" ]; then
    echo "   - python/mini_ros/__init__.py"
fi

echo ""
echo "💡 Next steps:"
echo "   1. Review changes: git diff"
echo "   2. Test: cargo test && python -c 'import mini_ros; print(mini_ros.__version__)'"
echo "   3. Commit: git add -A && git commit -m 'chore: bump version to $VERSION'"
echo "   4. Tag: git tag v$VERSION"
echo "   5. Release: git push origin v$VERSION" 