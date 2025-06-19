# CI/CD Automation

miniROS-rs uses a simple and effective CI/CD pipeline to automate testing and releases.

## Overview

The system consists of two main workflows:

1. **CI Pipeline** - Runs tests on every push/PR
2. **Release Pipeline** - Publishes packages when tags are created

## Continuous Integration

The CI workflow (`.github/workflows/ci.yml`) runs:

- **Code Quality**: `cargo fmt` and `cargo clippy`
- **Rust Tests**: `cargo test --all-features`
- **Python Package**: Build and basic import test

### Triggers
- Push to `main` or `develop` branch
- Pull requests to `main` or `develop` branch

## Release Process

### Automatic Release

Create a git tag to trigger a release:

```bash
git tag v1.2.3
git push origin v1.2.3
```

This will:
1. Run tests
2. Build Rust crate and Python wheels
3. Publish to crates.io and PyPI (if tokens are set)
4. Create GitHub release with artifacts

### Manual Release

Alternatively, manually trigger the release workflow in GitHub Actions:

1. Go to **Actions** → **Release**
2. Click **Run workflow**
3. Enter the version number (e.g., `1.2.3`)

## Setup

### Required Secrets

Add these secrets in your GitHub repository settings:

- `CRATES_IO_TOKEN` - For publishing to crates.io
- `PYPI_TOKEN` - For publishing to PyPI

### Getting Tokens

**crates.io token:**
```bash
# Login to crates.io and go to Account Settings → API Tokens
# Create a new token with publish scope
```

**PyPI token:**
```bash
# Login to PyPI and go to Account Settings → API tokens
# Create a new token with upload scope
```

## Workflow Files

### CI Workflow
```yaml
name: CI
on:
  push:
    branches: [ main, develop ]
  pull_request:
    branches: [ main, develop ]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Install Rust
        uses: dtolnay/rust-toolchain@stable
      - name: Run tests
        run: cargo test --all-features
```

### Release Workflow
```yaml
name: Release
on:
  push:
    tags: ['v*']

jobs:
  release:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Publish packages
        run: |
          cargo publish --token $CRATES_IO_TOKEN
          maturin build --release --features python
          twine upload dist/*.whl
```

## Best Practices

1. **Test First**: Always ensure tests pass before releasing
2. **Semantic Versioning**: Use `v1.2.3` format for tags
3. **Small Releases**: Release frequently with small changes
4. **Documentation**: Update docs with API changes

## Troubleshooting

**Release fails?**
- Check that tokens are correctly set in repository secrets
- Ensure version number is unique (not already published)
- Verify tests pass locally first

**Python build fails?**
- Check that `pyproject.toml` is properly configured
- Ensure maturin can find Rust source code

**crates.io publish fails?**
- Verify package name is unique
- Check that all required metadata is in `Cargo.toml`
- Ensure no breaking changes without version bump 