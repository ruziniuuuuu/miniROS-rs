#!/usr/bin/env python3
"""
Test the example scripts to ensure they work correctly

This replaces manual testing of examples by running them programmatically
and verifying their output and behavior.
"""

import pytest
import subprocess
import sys
import os
import time
import signal
from pathlib import Path


class TestExamples:
    """Test that all example scripts work correctly"""
    
    @classmethod
    def setup_class(cls):
        """Setup for the test class"""
        # Get the examples directory
        cls.examples_dir = Path(__file__).parent.parent / "examples"
        assert cls.examples_dir.exists(), f"Examples directory not found: {cls.examples_dir}"
    
    def run_example_with_timeout(self, script_name, timeout=10):
        """
        Run an example script with timeout and capture output
        
        Args:
            script_name: Name of the script to run
            timeout: Maximum time to run in seconds
            
        Returns:
            (returncode, stdout, stderr)
        """
        script_path = self.examples_dir / script_name
        assert script_path.exists(), f"Script not found: {script_path}"
        
        try:
            # Run the script with timeout
            result = subprocess.run(
                [sys.executable, str(script_path)],
                timeout=timeout,
                capture_output=True,
                text=True,
                cwd=str(self.examples_dir)
            )
            return result.returncode, result.stdout, result.stderr
            
        except subprocess.TimeoutExpired:
            # This is expected for scripts that run indefinitely
            return 124, "", "Timeout (expected for spinning scripts)"
    
    def test_minimal_publisher_runs(self):
        """Test that minimal_publisher.py runs without errors"""
        returncode, stdout, stderr = self.run_example_with_timeout("minimal_publisher.py", 8)
        
        # Should exit normally (return code 0)
        assert returncode == 0, f"Script failed with return code {returncode}. Stderr: {stderr}"
        
        # Should contain expected output
        assert "Published:" in stdout, f"Expected 'Published:' in output. Got: {stdout}"
        assert "Hello World:" in stdout, f"Expected 'Hello World:' in output. Got: {stdout}"
        
        # Should publish multiple messages
        publish_count = stdout.count("Published:")
        assert publish_count >= 3, f"Expected at least 3 published messages, got {publish_count}"
    
    def test_minimal_subscriber_timeout(self):
        """Test that minimal_subscriber.py starts correctly (will timeout as expected)"""
        returncode, stdout, stderr = self.run_example_with_timeout("minimal_subscriber.py", 3)
        
        # Should timeout (return code 124) since it waits for messages
        assert returncode == 124, f"Expected timeout, got return code {returncode}"
        
        # For timeout cases, the output might be empty since the process was killed
        # Just verify it didn't crash with an error
        assert stderr == "Timeout (expected for spinning scripts)" or "Error" not in stderr, \
            f"Script crashed with errors: {stderr}"
    
    def test_simple_pubsub_runs(self):
        """Test that simple_pubsub.py runs and completes successfully"""
        returncode, stdout, stderr = self.run_example_with_timeout("simple_pubsub.py", 10)
        
        # Should exit normally
        assert returncode == 0, f"Script failed with return code {returncode}. Stderr: {stderr}"
        
        # Should contain expected output patterns
        assert "🚀 Starting miniROS pub/sub demo" in stdout, \
            f"Expected demo start message. Got: {stdout}"
        assert "✅ Demo completed!" in stdout, \
            f"Expected demo completion message. Got: {stdout}"
        
        # Should show both publishing and receiving
        assert "📤 Published:" in stdout, f"Expected publish messages. Got: {stdout}"
        assert "📨 Received:" in stdout, f"Expected receive messages. Got: {stdout}"
        
        # Should publish and receive the same number of messages
        published_count = stdout.count("📤 Published:")
        received_count = stdout.count("📨 Received:")
        assert published_count == received_count, \
            f"Mismatch: {published_count} published, {received_count} received"
        assert published_count >= 3, f"Expected at least 3 messages, got {published_count}"
    
    def test_simple_param_runs(self):
        """Test that simple_param.py runs correctly if it exists"""
        script_path = self.examples_dir / "simple_param.py"
        
        if not script_path.exists():
            pytest.skip("simple_param.py not found - skipping parameter test")
        
        returncode, stdout, stderr = self.run_example_with_timeout("simple_param.py", 8)
        
        # Should exit normally or timeout (depending on implementation)
        assert returncode in [0, 124], \
            f"Script failed with unexpected return code {returncode}. Stderr: {stderr}"
        
        # Should show some parameter-related output
        param_keywords = ["parameter", "param", "config", "setting"]
        has_param_output = any(keyword in stdout.lower() for keyword in param_keywords)
        assert has_param_output, f"Expected parameter-related output. Got: {stdout}"


class TestExampleScriptStructure:
    """Test the structure and quality of example scripts"""
    
    @classmethod
    def setup_class(cls):
        """Setup for the test class"""
        cls.examples_dir = Path(__file__).parent.parent / "examples"
    
    def test_all_examples_have_docstrings(self):
        """Test that all example scripts have proper docstrings"""
        python_files = list(self.examples_dir.glob("*.py"))
        assert len(python_files) > 0, "No Python example files found"
        
        for script_path in python_files:
            if script_path.name.startswith("__"):
                continue  # Skip __init__.py etc.
            
            with open(script_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Should have a shebang
            assert content.startswith("#!/usr/bin/env python3"), \
                f"{script_path.name} should start with shebang"
            
            # Should have a docstring
            assert '"""' in content, f"{script_path.name} should have a docstring"
    
    def test_examples_import_mini_ros(self):
        """Test that all examples properly import mini_ros"""
        python_files = list(self.examples_dir.glob("*.py"))
        
        for script_path in python_files:
            if script_path.name.startswith("__"):
                continue
            
            with open(script_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            assert "import mini_ros" in content, \
                f"{script_path.name} should import mini_ros"
    
    def test_examples_are_reasonably_short(self):
        """Test that examples are concise (mini philosophy)"""
        python_files = list(self.examples_dir.glob("*.py"))
        
        for script_path in python_files:
            if script_path.name.startswith("__"):
                continue
            
            with open(script_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # Remove empty lines and comments for counting
            code_lines = [line for line in lines 
                         if line.strip() and not line.strip().startswith('#')]
            
            # Examples should be concise (under 50 lines of actual code)
            assert len(code_lines) < 50, \
                f"{script_path.name} has {len(code_lines)} code lines - should be under 50 for 'mini' philosophy"


class TestExampleCompatibility:
    """Test ROS2 compatibility aspects of examples"""
    
    @classmethod
    def setup_class(cls):
        """Setup for the test class"""
        cls.examples_dir = Path(__file__).parent.parent / "examples"
    
    def test_examples_use_ros2_patterns(self):
        """Test that examples follow ROS2 rclpy patterns"""
        python_files = list(self.examples_dir.glob("*.py"))
        
        for script_path in python_files:
            if script_path.name.startswith("__"):
                continue
            
            with open(script_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            # Should use ROS2-like initialization pattern
            ros2_patterns = [
                "mini_ros.init()",
                "mini_ros.shutdown()",
                "Node(",
                "create_publisher(",
                "create_subscription("
            ]
            
            pattern_count = sum(1 for pattern in ros2_patterns if pattern in content)
            assert pattern_count >= 2, \
                f"{script_path.name} should use ROS2-like patterns. Found {pattern_count}/5"
    
    def test_minimal_examples_are_truly_minimal(self):
        """Test that minimal examples are actually minimal"""
        minimal_scripts = ["minimal_publisher.py", "minimal_subscriber.py"]
        
        for script_name in minimal_scripts:
            script_path = self.examples_dir / script_name
            if not script_path.exists():
                continue
            
            with open(script_path, 'r', encoding='utf-8') as f:
                lines = f.readlines()
            
            # Total lines should be under 45 (slightly relaxed for docstrings)
            assert len(lines) < 45, \
                f"{script_name} has {len(lines)} lines - should be under 45 for minimal example"
            
            # Should be focused on one primary function
            with open(script_path, 'r', encoding='utf-8') as f:
                content = f.read()
            
            function_count = content.count("def ")
            assert function_count <= 2, \
                f"{script_name} has {function_count} functions - minimal examples should have 1-2 functions"


if __name__ == '__main__':
    pytest.main([__file__, '-v']) 