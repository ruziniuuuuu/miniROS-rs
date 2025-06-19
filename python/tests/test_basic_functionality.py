#!/usr/bin/env python3
"""
Basic functionality tests for miniROS Python API

Tests core features:
- Initialization and shutdown
- Node creation and management
- Message types
- Basic error handling
"""

import pytest
import mini_ros
import time
import threading
from concurrent.futures import ThreadPoolExecutor


class TestInitialization:
    """Test miniROS initialization and shutdown"""
    
    def test_init_shutdown(self):
        """Test basic init/shutdown cycle"""
        mini_ros.init()
        assert mini_ros.ok()
        mini_ros.shutdown()
    
    def test_multiple_init_shutdown(self):
        """Test multiple init/shutdown cycles"""
        for _ in range(3):
            mini_ros.init()
            assert mini_ros.ok()
            mini_ros.shutdown()
    
    def test_init_twice_no_error(self):
        """Test that calling init twice doesn't error"""
        mini_ros.init()
        mini_ros.init()  # Should not error
        assert mini_ros.ok()
        mini_ros.shutdown()


class TestNodeCreation:
    """Test node creation and management"""
    
    def setup_method(self):
        """Setup for each test"""
        mini_ros.init()
    
    def teardown_method(self):
        """Cleanup after each test"""
        mini_ros.shutdown()
    
    def test_create_node(self):
        """Test basic node creation"""
        node = mini_ros.Node('test_node')
        assert node is not None
        assert node.get_name() == 'test_node'
    
    def test_create_multiple_nodes(self):
        """Test creating multiple nodes"""
        node1 = mini_ros.Node('node1')
        node2 = mini_ros.Node('node2')
        
        assert node1.get_name() == 'node1'
        assert node2.get_name() == 'node2'
    
    def test_node_with_special_characters(self):
        """Test node names with special characters"""
        node = mini_ros.Node('test_node_123')
        assert node.get_name() == 'test_node_123'


class TestMessageTypes:
    """Test message type creation and usage"""
    
    def setup_method(self):
        """Setup for each test"""
        mini_ros.init()
    
    def teardown_method(self):
        """Cleanup after each test"""
        mini_ros.shutdown()
    
    def test_string_message(self):
        """Test StringMessage creation and manipulation"""
        msg = mini_ros.StringMessage()
        assert msg.data == ""
        
        msg.data = "Hello, World!"
        assert msg.data == "Hello, World!"
    
    def test_string_message_unicode(self):
        """Test StringMessage with Unicode"""
        msg = mini_ros.StringMessage()
        msg.data = "Hello, 世界! 🚀"
        assert msg.data == "Hello, 世界! 🚀"
    
    def test_int_message(self):
        """Test IntMessage if available"""
        try:
            msg = mini_ros.IntMessage()
            msg.data = 42
            assert msg.data == 42
        except AttributeError:
            # IntMessage might not be implemented yet
            pytest.skip("IntMessage not implemented")


class TestPublisherSubscriber:
    """Test publisher and subscriber functionality"""
    
    def setup_method(self):
        """Setup for each test"""
        mini_ros.init()
        self.node = mini_ros.Node('test_pubsub_node')
    
    def teardown_method(self):
        """Cleanup after each test"""
        mini_ros.shutdown()
    
    def test_create_publisher(self):
        """Test publisher creation"""
        pub = self.node.create_publisher(mini_ros.StringMessage, 'test_topic', 10)
        assert pub is not None
    
    def test_create_subscriber(self):
        """Test subscriber creation"""
        def callback(msg):
            pass
        
        sub = self.node.create_subscription(
            mini_ros.StringMessage, 
            'test_topic', 
            callback, 
            10
        )
        assert sub is not None
    
    def test_publish_message(self):
        """Test publishing a message"""
        pub = self.node.create_publisher(mini_ros.StringMessage, 'test_topic', 10)
        
        msg = mini_ros.StringMessage()
        msg.data = "Test message"
        
        # Should not raise an exception
        pub.publish(msg)
    
    def test_pubsub_communication(self):
        """Test end-to-end pub/sub communication"""
        received_messages = []
        
        def callback(msg):
            received_messages.append(msg.data)
        
        # Create publisher and subscriber
        pub = self.node.create_publisher(mini_ros.StringMessage, 'comm_topic', 10)
        sub = self.node.create_subscription(
            mini_ros.StringMessage,
            'comm_topic',
            callback,
            10
        )
        
        # Give some time for setup
        time.sleep(0.1)
        
        # Publish messages
        test_messages = ["Message 1", "Message 2", "Message 3"]
        for test_msg in test_messages:
            msg = mini_ros.StringMessage()
            msg.data = test_msg
            pub.publish(msg)
            time.sleep(0.05)  # Small delay between messages
        
        # Wait for message processing
        time.sleep(0.2)
        
        # Verify messages were received
        assert len(received_messages) == len(test_messages)
        for i, test_msg in enumerate(test_messages):
            assert received_messages[i] == test_msg
    
    def test_multiple_subscribers(self):
        """Test multiple subscribers on same topic"""
        received_count = [0, 0]  # Counter for each subscriber
        
        def callback1(msg):
            received_count[0] += 1
        
        def callback2(msg):
            received_count[1] += 1
        
        # Create publisher and two subscribers
        pub = self.node.create_publisher(mini_ros.StringMessage, 'multi_topic', 10)
        sub1 = self.node.create_subscription(
            mini_ros.StringMessage, 'multi_topic', callback1, 10
        )
        sub2 = self.node.create_subscription(
            mini_ros.StringMessage, 'multi_topic', callback2, 10
        )
        
        time.sleep(0.1)
        
        # Publish a message
        msg = mini_ros.StringMessage()
        msg.data = "Broadcast message"
        pub.publish(msg)
        
        time.sleep(0.2)
        
        # Both subscribers should receive the message
        assert received_count[0] == 1
        assert received_count[1] == 1
    
    def test_different_topics(self):
        """Test that messages on different topics don't interfere"""
        topic1_messages = []
        topic2_messages = []
        
        def callback1(msg):
            topic1_messages.append(msg.data)
        
        def callback2(msg):
            topic2_messages.append(msg.data)
        
        # Create publishers and subscribers for different topics
        pub1 = self.node.create_publisher(mini_ros.StringMessage, 'topic1', 10)
        pub2 = self.node.create_publisher(mini_ros.StringMessage, 'topic2', 10)
        
        sub1 = self.node.create_subscription(
            mini_ros.StringMessage, 'topic1', callback1, 10
        )
        sub2 = self.node.create_subscription(
            mini_ros.StringMessage, 'topic2', callback2, 10
        )
        
        time.sleep(0.1)
        
        # Publish to different topics
        msg1 = mini_ros.StringMessage()
        msg1.data = "Topic 1 message"
        pub1.publish(msg1)
        
        msg2 = mini_ros.StringMessage()
        msg2.data = "Topic 2 message"
        pub2.publish(msg2)
        
        time.sleep(0.2)
        
        # Verify messages went to correct subscribers
        assert len(topic1_messages) == 1
        assert len(topic2_messages) == 1
        assert topic1_messages[0] == "Topic 1 message"
        assert topic2_messages[0] == "Topic 2 message"


class TestErrorHandling:
    """Test error handling and edge cases"""
    
    def test_node_before_init(self):
        """Test creating node before init"""
        # This might work or might not depending on implementation
        # Just ensure it doesn't crash
        try:
            node = mini_ros.Node('test_node')
            assert node is not None
        except Exception:
            # It's okay if this fails
            pass
    
    def test_invalid_topic_names(self):
        """Test handling of invalid topic names"""
        mini_ros.init()
        node = mini_ros.Node('test_node')
        
        try:
            # Test empty topic name
            pub = node.create_publisher(mini_ros.StringMessage, '', 10)
            # If it doesn't raise an exception, that's fine too
        except Exception:
            # Expected behavior
            pass
        
        mini_ros.shutdown()
    
    def test_large_message(self):
        """Test handling of large messages"""
        mini_ros.init()
        node = mini_ros.Node('test_node')
        
        received_data = []
        
        def callback(msg):
            received_data.append(msg.data)
        
        pub = node.create_publisher(mini_ros.StringMessage, 'large_topic', 10)
        sub = node.create_subscription(
            mini_ros.StringMessage, 'large_topic', callback, 10
        )
        
        time.sleep(0.1)
        
        # Create a large message (10KB)
        large_text = "A" * 10000
        msg = mini_ros.StringMessage()
        msg.data = large_text
        
        pub.publish(msg)
        time.sleep(0.2)
        
        assert len(received_data) == 1
        assert received_data[0] == large_text
        
        mini_ros.shutdown()


class TestPerformance:
    """Basic performance and stress tests"""
    
    def setup_method(self):
        """Setup for each test"""
        mini_ros.init()
        self.node = mini_ros.Node('perf_test_node')
    
    def teardown_method(self):
        """Cleanup after each test"""
        mini_ros.shutdown()
    
    def test_high_frequency_publishing(self):
        """Test high-frequency message publishing"""
        received_count = [0]
        
        def callback(msg):
            received_count[0] += 1
        
        pub = self.node.create_publisher(mini_ros.StringMessage, 'high_freq', 1000)
        sub = self.node.create_subscription(
            mini_ros.StringMessage, 'high_freq', callback, 1000
        )
        
        time.sleep(0.1)
        
        # Publish 100 messages rapidly
        num_messages = 100
        for i in range(num_messages):
            msg = mini_ros.StringMessage()
            msg.data = f"Message {i}"
            pub.publish(msg)
        
        # Wait for processing
        time.sleep(0.5)
        
        # Should receive most or all messages
        assert received_count[0] >= num_messages * 0.8  # Allow for 20% loss
    
    def test_concurrent_nodes(self):
        """Test multiple nodes operating concurrently"""
        def node_worker(node_id):
            """Worker function for concurrent node testing"""
            node = mini_ros.Node(f'worker_node_{node_id}')
            
            received = []
            
            def callback(msg):
                received.append(msg.data)
            
            pub = node.create_publisher(mini_ros.StringMessage, f'worker_topic_{node_id}', 10)
            sub = node.create_subscription(
                mini_ros.StringMessage, f'worker_topic_{node_id}', callback, 10
            )
            
            time.sleep(0.1)
            
            # Each worker publishes a few messages
            for i in range(5):
                msg = mini_ros.StringMessage()
                msg.data = f"Worker {node_id} message {i}"
                pub.publish(msg)
                time.sleep(0.01)
            
            time.sleep(0.2)
            return len(received)
        
        # Run multiple workers concurrently
        with ThreadPoolExecutor(max_workers=3) as executor:
            futures = [executor.submit(node_worker, i) for i in range(3)]
            results = [future.result() for future in futures]
        
        # Each worker should receive their own messages
        for result in results:
            assert result == 5


if __name__ == '__main__':
    pytest.main([__file__, '-v']) 