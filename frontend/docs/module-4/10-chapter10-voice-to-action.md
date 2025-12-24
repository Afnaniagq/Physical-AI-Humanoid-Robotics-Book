---
id: chapter10
title: "Chapter 10: Voice-to-Action (OpenAI Whisper)"
sidebar_position: 10
---

# Chapter 10: Voice-to-Action (OpenAI Whisper)

This chapter covers the integration of OpenAI's Whisper model into a ROS 2 ecosystem to create a voice-controlled robotics application.

## Introduction to Speech-to-Text in Robotics

- Why voice control is a powerful interface for robots.
- Overview of available speech-to-text technologies.

## Audio Stream Buffering for High-Latency Conditions

Handling audio streams in robotics, especially with services like Whisper over a network, often involves dealing with latency and intermittent connectivity. Buffering the audio stream is crucial for maintaining a robust and continuous transcription process.

### Challenges with Real-time Audio

- **Network Latency**: Delays in transmitting audio data to the Whisper service.
- **Jitter**: Variations in network delay, leading to uneven audio packet arrival.
- **Temporary Disconnections**: Brief loss of network connectivity.

### Buffering Strategy

- **Circular Buffer**: Implement a circular buffer to store incoming audio chunks.
- **Thresholds**: Define thresholds for minimum and maximum buffer sizes to trigger processing or pause input.
- **Re-synchronization**: Mechanisms to re-synchronize the audio stream after interruptions.

By buffering the audio, the system can smooth out network inconsistencies, ensuring that Whisper always receives a continuous flow of audio for more accurate and reliable transcriptions.

## Setting up OpenAI Whisper

- How to get access to the Whisper API or run it locally.
- Installation and dependencies.

## Creating a ROS 2 Node for Whisper

- We will create a Python-based ROS 2 node.
- The node will listen to an audio topic or use a microphone directly.
- It will publish the transcribed text to a string topic.

```python
# Placeholder for rclpy Python code for a ROS 2 Whisper node.
# This will be a simple node that subscribes to an audio topic,
# calls the Whisper service, and publishes the transcription.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray

class WhisperNode(Node):
    def __init__(self):
        super().__init__('whisper_node')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'audio',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(String, 'transcription', 10)

    def listener_callback(self, msg):
        # In a real implementation, you would convert the audio message
        # and send it to Whisper API.
        self.get_logger().info('I heard audio, pretending to transcribe...')
        transcribed_text = "this is a test"
        
        # Publish the transcribed text
        message = String()
        message.data = transcribed_text
        self.publisher_.publish(message)
        self.get_logger().info('Publishing: "%s"' % message.data)

def main(args=None):
    rclpy.init(args=args)
    whisper_node = WhisperNode()
    rclpy.spin(whisper_node)
    whisper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Advanced QoS Profile for Audio Subscribers

For audio streams, especially in real-time robotics, ensuring message delivery reliability and timeliness is critical. ROS 2 Quality of Service (QoS) policies allow fine-grained control over how messages are sent and received.

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Int16MultiArray, String

class AdvancedWhisperNode(Node):
    def __init__(self):
        super().__init__('advanced_whisper_node')

        # Define a custom QoS profile for the audio subscription
        # This profile prioritizes reliability over strict latency
        audio_qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=10,  # Keep up to 10 samples
            durability=DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
        )

        self.subscription = self.create_subscription(
            Int16MultiArray,
            'audio_in',
            self.audio_callback,
            qos_profile=audio_qos_profile) # Apply the custom QoS profile

        self.publisher_ = self.create_publisher(String, 'transcription_out', 10)
        self.get_logger().info('Advanced Whisper Node started with custom QoS.')

    def audio_callback(self, msg):
        self.get_logger().info('Received audio data with custom QoS settings.')
        # Simulate transcription
        transcribed_text = "advanced transcription example"
        output_msg = String()
        output_msg.data = transcribed_text
        self.publisher_.publish(output_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedWhisperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating with a Robot

- How to subscribe to the transcribed text topic from other nodes.
- Parsing the text to extract commands.
- Example of a simple robot that responds to voice commands.

## Common Failures and Troubleshooting

| Problem | Possible Cause(s) | Solution(s) |
|---|---|---|
| **Microphone not detected/accessible** | 1. Incorrect device permissions (especially in Docker containers).<br/>2. Microphone hardware failure or disconnection.<br/>3. ROS 2 audio driver issues. | 1. **Docker**: Ensure `docker run` includes `--device /dev/snd` or appropriate audio device mapping, and the user inside the container has audio group permissions. <br/>2. Check physical connections and system audio settings. <br/>3. Verify `ros2 run audio_common_msgs audio_capture` (or similar) is working. |
| **Whisper Model loading timeouts/errors** | 1. Insufficient RAM or CPU for the chosen model size.<br/>2. Slow internet connection for model download (if applicable).<br/>3. Incorrect model path or corrupted download. | 1. Use a smaller Whisper model (`tiny`, `base`). Increase container/VM resources. <br/>2. Pre-download the model. <br/>3. Verify model file integrity and path configuration. |
| **Garbled or inaccurate transcription** | 1. Poor audio quality (noise, echo).<br/>2. Microphone too far from speaker or gain set incorrectly.<br/>3. Language mismatch (Whisper transcribing in wrong language). | 1. Improve acoustic environment. Use noise suppression techniques. <br/>2. Adjust microphone placement and input gain. <br/>3. Explicitly set the language parameter in Whisper configuration if not auto-detecting correctly. |
| **ROS 2 topics not connecting** | 1. Incorrect topic names or message types.<br/>2. ROS 2 network issues (firewall, `ROS_DOMAIN_ID`).<br/>3. Nodes not running or crashing. | 1. Double-check `ros2 topic list -t` and `ros2 interface show <msg_type>`. <br/>2. Verify `ROS_DOMAIN_ID` matches across all nodes. Check firewall rules. <br/>3. Run `ros2 node info <node_name>` to debug. |

