---
id: chapter2
title: "ROS 2 Core Communication Fundamentals"
sidebar_label: "Chapter 2: Core Communication"
slug: /module1/chapter2
---

# Chapter 2: ROS 2 Core Communication Fundamentals

## Mastering the Communication Graph

In the previous chapter, we introduced ROS 2 as a decentralized, DDS-based middleware designed to facilitate robust communication within complex robotic systems. At the heart of any ROS 2 application is the "communication graph" – a dynamic network of interacting processes, or nodes, that exchange data to collectively achieve a robotic task. Understanding the fundamental communication mechanisms within this graph is crucial for developing effective Physical AI applications. This chapter will provide a hands-on guide to these primitives, focusing on practical implementation using `rclpy`, the Python client library for ROS 2.

ROS 2 provides several core communication primitives, each suited for different types of data exchange and interaction patterns:
*   **Nodes**: The fundamental, modular processing units that encapsulate specific functionalities.
*   **Topics**: For asynchronous, publish-subscribe data streaming, ideal for continuous data flows.
*   **Services**: For synchronous, request-reply interactions, suitable for discrete queries or commands requiring immediate responses.
*   **Actions**: For long-running, goal-oriented tasks with feedback and preemption capabilities (these will be covered in greater detail in a subsequent module, but it's important to recognize their role in more complex task execution).

In this chapter, we will delve into Nodes, Topics, and Services, providing practical Python `rclpy` examples to solidify your understanding and enable you to build basic ROS 2 applications.

## Nodes: The Executable Heart of ROS 2

A node in ROS 2 is an executable process that performs a specific, often modular, function within the robot's software system. Think of a node as a single, focused program responsible for a particular piece of functionality. For example, a robotic system might consist of:
*   A `camera_node` that acquires images from a camera.
*   A `object_detection_node` that processes those images to identify objects.
*   A `path_planning_node` that generates a path for the robot to navigate.
*   A `motor_controller_node` that sends commands to the robot's actuators.

Nodes are implemented using the ROS Client Libraries (`rclpy` for Python, `rclcpp` for C++). These libraries provide the necessary APIs to create node instances and enable them to interact with the ROS 2 communication graph (i.e., publish data to topics, subscribe to data from topics, offer services, or call services).

**Key characteristics of Nodes:**
*   **Modularity**: Each node typically performs a single, well-defined task. This promotes clear separation of concerns, simplifies development, and makes debugging more manageable. If one part of the system (e.g., object detection) needs an update, only that specific node might require modification and redeployment.
*   **Distribution**: Nodes are designed to run independently and can be distributed across various computational resources. They can execute on the same physical machine, on different machines in a local network, or even across a wider network connected to cloud services. DDS handles the underlying complexity of inter-process and inter-machine communication.
*   **Independence**: Nodes operate independently of each other. They communicate by sending and receiving messages via the ROS 2 communication mechanisms, without direct dependencies on the internal state or implementation details of other nodes. This fosters robust and fault-tolerant systems.

### Creating a Simple ROS 2 Node with `rclpy`

The fundamental steps to create a ROS 2 node in Python using `rclpy` involve initializing the `rclpy` library, creating an instance of `rclpy.node.Node`, and then "spinning" the node to keep it alive and allow it to process callbacks for incoming messages or service requests.

Below is a minimal example of a node that initializes `rclpy` and creates a node instance. While this node doesn't perform any specific communication yet, it serves as the basic boilerplate for any ROS 2 Python application.

```python
# minimal_node.py
import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args) # Initialize the ROS 2 Python client library

    node = Node('my_first_ros2_node') # Create a node with a unique name
    node.get_logger().info('My First ROS 2 Node has been started.') # Log a message

    try:
        rclpy.spin(node) # Keep the node alive and processing events until interrupted
    except KeyboardInterrupt:
        node.get_logger().info('Node gracefully shut down.')
    finally:
        node.destroy_node() # Destroy the node explicitly to free resources
        rclpy.shutdown() # Shut down the ROS 2 Python client library

if __name__ == '__main__':
    main()
```
**To run this node (after sourcing your ROS 2 environment):**
1.  Save the code as `minimal_node.py`.
2.  Ensure you have a ROS 2 workspace set up. If not, create one:
    ```bash
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    ros2 pkg create --build-type ament_python my_python_pkg
    ```
3.  Place `minimal_node.py` inside `~/ros2_ws/src/my_python_pkg/my_python_pkg/`.
4.  Add the entry point in `~/ros2_ws/src/my_python_pkg/setup.py` under `entry_points` (if it doesn't exist, create this dictionary within `setup()`):
    ```python
    entry_points={
        'console_scripts': [
            'my_first_ros2_node = my_python_pkg.minimal_node:main',
        ],
    },
    ```
5.  Build your workspace (from `~/ros2_ws/`):
    ```bash
    colcon build --packages-select my_python_pkg
    ```
6.  Source the setup files (from `~/ros2_ws/`):
    ```bash
    # For Bash/Zsh:
    . install/setup.bash
    # For PowerShell:
    . .\install\setup.ps1
    ```
7.  Run the node:
    ```bash
    ros2 run my_python_pkg my_first_ros2_node
    ```
    You should see the "My First ROS 2 Node has been started." message, and the node will remain active until you press `Ctrl+C`.

## Topics: Asynchronous Data Streams

Topics are the most fundamental and widely used communication mechanism in ROS 2. They implement a publish-subscribe (pub/sub) pattern, designed for streaming continuous or periodic data. When a node generates data (e.g., sensor readings, status updates), it "publishes" a message to a named topic. Any other node interested in that data can "subscribe" to the same topic and receive all messages published to it. This mechanism allows for a highly decoupled architecture.

**Key characteristics of Topics:**
*   **Decoupled Communication**: Publishers and subscribers do not need to know about each other's existence. They communicate anonymously via the topic. This allows for great flexibility; new publishers or subscribers can be added or removed dynamically without affecting existing parts of the system.
*   **Asynchronous**: Data flows unidirectionally. A publisher sends data without waiting for an acknowledgment, and subscribers process data as it arrives. This is crucial for high-frequency data streams where latency is critical.
*   **One-to-many / Many-to-many**: A single topic can have multiple publishers sending data to it, and multiple subscribers receiving data from it. This flexibility supports complex data distribution patterns.
*   **Quality of Service (QoS)**: ROS 2 significantly enhances communication reliability and behavior through fine-grained QoS settings. These settings can be configured for each publisher and subscriber to control aspects like:
    *   **Reliability**: `Reliable` (guaranteed delivery) vs. `Best Effort` (data loss possible, lower latency).
    *   **Durability**: `Transient Local` (new subscribers receive last message) vs. `Volatile` (subscribers only receive messages published while they are active).
    *   **History**: `Keep Last` (only store a certain number of messages) vs. `Keep All` (store all messages up to a limit).
    *   **Liveliness**: How publishers assert their presence.
    *   **Lease Duration**: How long messages are considered valid.
    These QoS profiles are essential for tailoring communication to specific application requirements, from high-fidelity sensor data to critical control commands.

### Implementing a Simple ROS 2 Publisher and Subscriber

Let's build a classic "talker" (publisher) and "listener" (subscriber) example to demonstrate topic communication. We'll use the `std_msgs/msg/String` message type, which is a simple string container.

**Publisher Node (`my_python_pkg/my_python_pkg/simple_publisher.py`)**
```python
# simple_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class SimplePublisher(Node):

    def __init__(self):
        # Initialize the node with the name 'simple_publisher'
        super().__init__('simple_publisher')
        # Create a publisher for messages of type String on the 'chatter' topic
        # The queue size (10) limits the number of outgoing messages in the buffer
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.i = 0 # Counter for messages
        timer_period = 0.5  # Message publishing rate in seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Create a timer to call timer_callback periodically
        self.get_logger().info('SimplePublisher Node has been started and is publishing to /chatter.')

    def timer_callback(self):
        """Callback function executed by the timer to publish messages."""
        msg = String() # Create a new String message object
        msg.data = f'Hello ROS 2 World! Count: {self.i}' # Populate message data
        self.publisher_.publish(msg) # Publish the message
        self.get_logger().info(f'Publishing: "{msg.data}"') # Log what was published
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    simple_publisher = SimplePublisher() # Create the publisher node
    rclpy.spin(simple_publisher) # Keep the node alive
    simple_publisher.destroy_node() # Clean up resources
    rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```

**Subscriber Node (`my_python_pkg/my_python_pkg/simple_subscriber.py`)**
```python
# simple_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Import the standard String message type

class SimpleSubscriber(Node):

    def __init__(self):
        # Initialize the node with the name 'simple_subscriber'
        super().__init__('simple_subscriber')
        # Create a subscriber to the 'chatter' topic, expecting String messages
        # When a message arrives, call listener_callback
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription # prevent unused variable warning by referencing it
        self.get_logger().info('SimpleSubscriber Node has been started and is listening to /chatter.')

    def listener_callback(self, msg):
        """Callback function executed when a message is received on the 'chatter' topic."""
        self.get_logger().info(f'I heard: "{msg.data}"') # Log the received message

def main(args=None):
    rclpy.init(args=args) # Initialize rclpy
    simple_subscriber = SimpleSubscriber() # Create the subscriber node
    rclpy.spin(simple_subscriber) # Keep the node alive
    simple_subscriber.destroy_node() # Clean up resources
    rclpy.shutdown() # Shut down rclpy

if __name__ == '__main__':
    main()
```
**To run the publisher and subscriber:**
1.  Save `simple_publisher.py` and `simple_subscriber.py` in your ROS 2 package directory, e.g., `~/ros2_ws/src/my_python_pkg/my_python_pkg/`.
2.  Update `~/ros2_ws/src/my_python_pkg/setup.py` entry points to include both scripts:
    ```python
    entry_points={
        'console_scripts': [
            'my_first_ros2_node = my_python_pkg.minimal_node:main', # Existing entry
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main',
        ],
    },
    ```
3.  Build your workspace (from `~/ros2_ws/`):
    ```bash
    colcon build --packages-select my_python_pkg
    ```
4.  Source the setup files (from `~/ros2_ws/`):
    ```bash
    . install/setup.bash # or your shell's equivalent
    ```
5.  Open two separate terminal windows. In *each* terminal, source the setup files.
6.  In **Terminal 1**: Run the publisher
    ```bash
    ros2 run my_python_pkg simple_publisher
    ```
7.  In **Terminal 2**: Run the subscriber
    ```bash
    ros2 run my_python_pkg simple_subscriber
    ```
    You should observe the publisher continuously logging messages, and the subscriber in the other terminal simultaneously logging that it "heard" those messages.

**[Image of ROS 2 Publisher and Subscriber Nodes Communication Diagram]**

You can further inspect the active topics using ROS 2 command-line tools:
*   List all active topics:
    ```bash
    ros2 topic list
    ```
    You should see `/chatter` among them.
*   Echo messages being published on a topic:
    ```bash
    ros2 topic echo /chatter
    ```
    This command will print all messages received on `/chatter` to your terminal, confirming that the communication is working as expected.

## Services: Synchronous Request/Reply

While topics are excellent for continuous data streams, sometimes you need a more direct, transactional interaction: a node sends a request, and another node processes it and sends back a response. This is precisely the role of Services in ROS 2. Services provide a synchronous request/reply communication pattern, analogous to a function call or a Remote Procedure Call (RPC) over the network.

**Key characteristics of Services:**
*   **Synchronous**: The client node sends a request and then blocks its execution, waiting for a response from the service server. This "blocking" nature makes them suitable for discrete operations where the client's next action depends on the server's immediate reply.
*   **Request-Reply**: Services define a clear interface consisting of a request message and a response message. The client populates the request, and the server populates the response.
*   **One-to-one**: Typically, a client initiates a request to a single service server. While multiple clients can call the same service, each call is an individual transaction between one client and one server.
*   **Defined Interfaces**: Service definitions are specified in `.srv` files. These files are crucial as they formally define the structure of both the data sent in the request and the data returned in the response, ensuring type safety and interoperability.

### Implementing a Simple ROS 2 Client/Server Service

Let's create a service that takes two 64-bit integers, adds them together, and returns their sum. This is a common example to illustrate service functionality.

**1. Define the Service Interface (`my_python_pkg/srv/AddTwoInts.srv`)**
First, you need to define the service interface. Create a `srv` directory inside your package and a file named `AddTwoInts.srv` with the following content:
```
int64 a
int64 b
---
int64 sum
```
The `---` is a mandatory separator between the request fields (above) and the response fields (below).

**2. Implement the Service Server (`my_python_pkg/my_python_pkg/add_two_ints_server.py`)**
The service server node will "advertise" the `add_two_ints` service and provide a callback function to handle incoming requests.

```python
# add_two_ints_server.py
import rclpy
from rclpy.node import Node
from my_python_pkg.srv import AddTwoInts # Import the custom service type

class AddTwoIntsService(Node):

    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create a service named 'add_two_ints' using the AddTwoInts service type
        # When a request comes in, call add_two_ints_callback
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('AddTwoInts Service has been started and is ready to add integers.')

    def add_two_ints_callback(self, request, response):
        """Callback function for the AddTwoInts service."""
        response.sum = request.a + request.b # Perform the addition
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}')
        self.get_logger().info(f'Sending response: sum={response.sum}')
        return response # Return the populated response object

def main(args=None):
    rclpy.init(args=args)
    service_node = AddTwoIntsService()
    rclpy.spin(service_node) # Keep the service node alive
    service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**3. Implement the Service Client (`my_python_pkg/my_python_pkg/add_two_ints_client.py`)**
The service client node will send a request to the `add_two_ints` service and process the response.

```python
# add_two_ints_client.py
import sys
import rclpy
from rclpy.node import Node
from my_python_pkg.srv import AddTwoInts # Import the custom service type

class AddTwoIntsClientAsync(Node):

    def __init__(self):
        super().__init__('add_two_ints_client_async')
        # Create a client for the 'add_two_ints' service
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait for the service to become available
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service "add_two_ints" not available, waiting again...')
        self.req = AddTwoInts.Request() # Create an instance of the request message
        self.get_logger().info('AddTwoInts Client has been started.')

    def send_request(self, a, b):
        """Send a request to the AddTwoInts service."""
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req) # Call the service asynchronously
        self.get_logger().info(f'Request sent: a={a}, b={b}')
        # Spin until the future is complete (i.e., response is received or error occurs)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            return self.future.result()
        else:
            self.get_logger().error('Service call failed %r' % (self.future.exception(),))
            return None

def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) != 3:
        node = Node('temp_error_node') # Create a temporary node for logging
        node.get_logger().error('Usage: ros2 run my_python_pkg add_two_ints_client <int_a> <int_b>')
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    client_node = AddTwoIntsClientAsync()
    response = client_node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    
    if response is not None:
        client_node.get_logger().info(f'Result of add_two_ints: {client_node.req.a} + {client_node.req.b} = {response.sum}')
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**4. Update `setup.py`**
To make the service and client executable and to ensure the `.srv` file is correctly installed, you need to update your `~/ros2_ws/src/my_python_pkg/setup.py` file.

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Install all files from the 'srv' directory
        (os.path.join('share', package_name, 'srv'), glob(os.path.join('srv', '*.srv'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Python package for ROS 2 examples',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_ros2_node = my_python_pkg.minimal_node:main',
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main',
            'add_two_ints_server = my_python_pkg.add_two_ints_server:main', # New server entry point
            'add_two_ints_client = my_python_pkg.add_two_ints_client:main',   # New client entry point
        ],
    },
)
```
**Important:** Also ensure your `package.xml` declares the `rosidl_default_generators` and `rosidl_runtime_py` dependencies:
```xml
<!-- In package.xml -->
<buildtool_depend>ament_python</buildtool_depend>
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>rosidl_default_generators</depend>
<exec_depend>rosidl_runtime_py</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**To run the service and client:**
1.  Ensure `AddTwoInts.srv` is in `~/ros2_ws/src/my_python_pkg/srv/`.
2.  Save `add_two_ints_server.py` and `add_two_ints_client.py` in `~/ros2_ws/src/my_python_pkg/my_python_pkg/`.
3.  Update `setup.py` and `package.xml` as shown above.
4.  Build your workspace (from `~/ros2_ws/`):
    ```bash
    colcon build --packages-select my_python_pkg
    ```
5.  Source the setup files (from `~/ros2_ws/`):
    ```bash
    . install/setup.bash # or your shell's equivalent
    ```
6.  Open two separate terminal windows. In *each* terminal, source the setup files.
7.  In **Terminal 1**: Run the service server
    ```bash
    ros2 run my_python_pkg add_two_ints_server
    ```
8.  In **Terminal 2**: Run the client with two integer arguments (e.g., 2 and 3)
    ```bash
    ros2 run my_python_pkg add_two_ints_client 2 3
    ```
    You should see the client terminal printing the sum (e.g., "Result of add_two_ints: 2 + 3 = 5"), and the service terminal logging the incoming request and the response sent.

**[Image of ROS 2 Service Client and Server Communication Diagram]**

You can also test the service directly from the command line without writing a client node:
*   List active services:
    ```bash
    ros2 service list
    ```
    You should see `/add_two_ints`.
*   Get the service type:
    ```bash
    ros2 service type /add_two_ints
    ```
    This should return `my_python_pkg/srv/AddTwoInts`.
*   Call the service with arguments from the command line:
    ```bash
    ros2 service call /add_two_ints my_python_pkg/srv/AddTwoInts "{a: 10, b: 20}"
    ```
    This will send a request with `a=10` and `b=20` and print the response directly to your terminal.

This chapter has provided a hands-on introduction to ROS 2's core communication primitives: Nodes, Topics, and Services. Mastering these concepts forms the bedrock for building sophisticated robotic applications that require robust and flexible data exchange. In the next chapter, we will explore how to describe robot morphology and bridge Python AI agents to control these physical systems, bringing us closer to truly embodied intelligence.