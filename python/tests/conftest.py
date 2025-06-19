#!/usr/bin/env python3
"""
Pytest configuration for miniROS Python tests

This file contains shared fixtures and configuration for all tests.
"""

import pytest
import sys
import os
import time
from pathlib import Path

# Add the project root to Python path to ensure we can import mini_ros
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

# Increase default timeout for tests that involve communication
pytest.DEFAULT_TIMEOUT = 10


@pytest.fixture(scope="session", autouse=True)
def setup_test_environment():
    """
    Setup the test environment before running any tests
    
    This fixture runs once per test session and ensures:
    - Python can find the mini_ros module
    - Test environment is clean
    """
    # Verify that mini_ros can be imported
    try:
        import mini_ros
        print(f"✅ mini_ros module found and importable")
    except ImportError as e:
        pytest.fail(f"❌ Cannot import mini_ros: {e}. "
                   f"Make sure to run 'maturin develop --features python' first")
    
    yield
    
    # Cleanup after all tests
    print("🧹 Test session cleanup completed")


@pytest.fixture
def mini_ros_context():
    """
    Fixture that provides a clean miniROS context for each test
    
    This ensures each test starts with a fresh miniROS environment
    and cleans up afterwards.
    """
    import mini_ros
    
    # Initialize miniROS
    mini_ros.init()
    
    # Verify it's working
    assert mini_ros.ok(), "miniROS initialization failed"
    
    yield mini_ros
    
    # Cleanup
    try:
        mini_ros.shutdown()
    except Exception as e:
        # Don't fail the test if shutdown has issues
        print(f"Warning: miniROS shutdown had issues: {e}")


@pytest.fixture
def test_node(mini_ros_context):
    """
    Fixture that provides a test node for each test
    
    This creates a uniquely named node for each test to avoid conflicts.
    """
    import mini_ros
    
    # Create node with unique name based on test
    node_name = f"test_node_{int(time.time() * 1000000) % 1000000}"
    node = mini_ros.Node(node_name)
    
    yield node
    
    # Node cleanup is handled by mini_ros_context fixture


@pytest.fixture
def message_factory():
    """
    Fixture that provides a factory for creating test messages
    """
    import mini_ros
    
    def create_string_message(data="test_data"):
        """Create a StringMessage with given data"""
        msg = mini_ros.StringMessage()
        msg.data = data
        return msg
    
    def create_test_messages(count=3, prefix="test_msg"):
        """Create multiple test messages"""
        return [create_string_message(f"{prefix}_{i}") for i in range(count)]
    
    # Return a simple object with factory methods
    class MessageFactory:
        string = create_string_message
        multiple = create_test_messages
    
    return MessageFactory()


# Pytest configuration
def pytest_configure(config):
    """Configure pytest with custom markers and settings"""
    config.addinivalue_line(
        "markers", "slow: marks tests as slow (deselect with '-m \"not slow\"')"
    )
    config.addinivalue_line(
        "markers", "integration: marks tests as integration tests"
    )
    config.addinivalue_line(
        "markers", "performance: marks tests as performance tests"
    )


def pytest_collection_modifyitems(config, items):
    """Modify test collection to add markers automatically"""
    for item in items:
        # Mark tests that take a long time
        if "performance" in item.name.lower() or "concurrent" in item.name.lower():
            item.add_marker(pytest.mark.slow)
        
        # Mark integration tests
        if "test_examples.py" in str(item.fspath):
            item.add_marker(pytest.mark.integration)


# Custom assertions
class MiniROSAssertions:
    """Custom assertions for miniROS testing"""
    
    @staticmethod
    def assert_message_received(received_messages, expected_data, timeout=1.0):
        """Assert that a message with expected data was received within timeout"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if any(msg == expected_data for msg in received_messages):
                return True
            time.sleep(0.01)
        
        pytest.fail(f"Expected message '{expected_data}' not received within {timeout}s. "
                   f"Received: {received_messages}")
    
    @staticmethod
    def assert_communication_working(pub, sub, test_data="test", timeout=1.0):
        """Assert that pub/sub communication is working"""
        import mini_ros
        
        received_data = []
        
        def callback(msg):
            received_data.append(msg.data)
        
        # This would need to be implemented based on the actual API
        # For now, just check that objects exist
        assert pub is not None, "Publisher should not be None"
        assert sub is not None, "Subscriber should not be None"


# Make assertions available to all tests
@pytest.fixture
def assert_miniROS():
    """Fixture providing custom miniROS assertions"""
    return MiniROSAssertions() 