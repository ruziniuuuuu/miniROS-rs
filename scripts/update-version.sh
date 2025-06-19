#!/bin/bash
set -e

VERSION=$1

if [ -z "$VERSION" ]; then
    echo "Error: Version not provided"
    exit 1
fi

echo "Updating version to $VERSION"

# Update Cargo.toml
sed -i.bak "s/^version = \".*\"/version = \"$VERSION\"/" Cargo.toml
rm -f Cargo.toml.bak

# Update Cargo.lock
cargo check --quiet

echo "Version updated successfully" 