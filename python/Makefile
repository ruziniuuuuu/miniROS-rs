# Makefile for miniROS Python Package
# This simplifies testing and development workflows

.PHONY: help install test test-fast test-integration test-performance clean dev-install coverage format lint

# Default target
help:
	@echo "miniROS Python Package - Development Commands"
	@echo ""
	@echo "Setup Commands:"
	@echo "  install       Install the package in development mode"
	@echo "  dev-install   Install with development dependencies"
	@echo ""
	@echo "Testing Commands:"
	@echo "  test          Run all tests"
	@echo "  test-fast     Run fast tests only (skip slow/integration)"
	@echo "  test-integration  Run integration tests only"
	@echo "  test-performance  Run performance tests only"
	@echo "  coverage      Run tests with coverage report"
	@echo ""
	@echo "Code Quality:"
	@echo "  format        Format code with black (if available)"
	@echo "  lint          Run linting checks (if available)"
	@echo ""
	@echo "Cleanup:"
	@echo "  clean         Clean up temporary files and cache"

# Installation targets
install:
	@echo "🔧 Installing miniROS Python package..."
	cd .. && maturin develop --features python
	pip install -e .

dev-install:
	@echo "🛠️ Installing with development dependencies..."
	cd .. && maturin develop --features python
	pip install -e ".[dev]"

# Testing targets
test: dev-install
	@echo "🧪 Running all tests..."
	pytest tests/ -v

test-fast: dev-install
	@echo "⚡ Running fast tests only..."
	pytest tests/ -v -m "not slow and not integration"

test-integration: dev-install
	@echo "🔗 Running integration tests..."
	pytest tests/ -v -m "integration"

test-performance: dev-install
	@echo "🚀 Running performance tests..."
	pytest tests/ -v -m "performance"

coverage: dev-install
	@echo "📊 Running tests with coverage..."
	pytest tests/ --cov=mini_ros --cov-report=html --cov-report=term

# Code quality targets
format:
	@echo "🎨 Formatting code..."
	@if command -v black >/dev/null 2>&1; then \
		black examples/ tests/; \
		echo "✅ Code formatted with black"; \
	else \
		echo "⚠️ black not installed, skipping formatting"; \
	fi

lint:
	@echo "🔍 Running linting checks..."
	@if command -v flake8 >/dev/null 2>&1; then \
		flake8 examples/ tests/; \
		echo "✅ Linting completed"; \
	else \
		echo "⚠️ flake8 not installed, skipping linting"; \
	fi

# Cleanup targets
clean:
	@echo "🧹 Cleaning up..."
	find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete 2>/dev/null || true
	find . -type f -name "*.pyo" -delete 2>/dev/null || true
	find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	rm -rf .pytest_cache/ 2>/dev/null || true
	rm -rf htmlcov/ 2>/dev/null || true
	rm -rf .coverage 2>/dev/null || true
	@echo "✅ Cleanup completed"

# Examples targets
run-examples:
	@echo "🚀 Running example scripts..."
	@echo "1. Running minimal_publisher..."
	python examples/minimal_publisher.py &
	@echo "2. Running simple_pubsub..."
	python examples/simple_pubsub.py
	@echo "✅ Examples completed"

# Quick development cycle
quick-test: install
	@echo "⚡ Quick test cycle..."
	pytest tests/test_basic_functionality.py -v -q

# Full development cycle
full-cycle: clean dev-install test
	@echo "🎯 Full development cycle completed successfully!" 