import os
import threading
import time
from typing import Dict, Any, Union

from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import Publisher, DataWriter
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.topic import Topic

from .messages import *


class AlexCommunication:
    """
    This class sets up communication for Alex using DDS.
    """
    def __init__(self, ip_address: str = "127.0.0.1", domain_id: int = 42,  frequency: float = 100.0):
        """
        Initialize AlexCommunication class

        :param ip_address: IP address to use for DDS communication
        :param domain_id: RTPS domain ID for communication
        :param frequency: Communication frequency in Hz
        """
        # Set up the communication environment to specifically select the correct IP address
        os.environ["CYCLONEDDS_URI"] = "<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"" + ip_address + "\"/></Interfaces></General></Domain></CycloneDDS>"
        self.alex_state = None
        self._missed_loops = 0
        self._avg_time = 0.0
        self._num_loops = 0

        # Set the loop dt
        self.frequency = frequency
        self.dt = 1.0/frequency

        # Initialize the DDS communication and topics
        qos = Qos(Policy.Reliability.Reliable(max_blocking_time=1000))
        qos.reliability = Policy.Reliability.Reliable
        domain_participant = DomainParticipant(domain_id, qos)
        alex_state_topic = Topic(domain_participant, "rt/alex_state", AlexState)
        alex_command_topic = Topic(domain_participant, "rt/alex_command", AlexCommand)
        left_hand_command_topic = Topic(domain_participant, "rt/ezgripper/left/command", EZGripperCommand)
        right_hand_command_topic = Topic(domain_participant, "rt/ezgripper/right/command", EZGripperCommand)
        left_hand_state_topic = Topic(domain_participant, "rt/ezgripper/left/state", EZGripperState)
        right_hand_state_topic = Topic(domain_participant, "rt/ezgripper/right/state", EZGripperState)
        alex_status_topic = Topic(domain_participant, "rt/hardware_status", HardwareStatus)

        # Set up listeners and subscribers
        self.state_listener = AlexStateListener()
        self.left_hand_state_listener = HandStateListener()
        self.right_hand_state_listener = HandStateListener()
        self.status_listener = HardwareStatusListener()
        subscriber = Subscriber(domain_participant)
        self._alex_state_reader = DataReader(subscriber, alex_state_topic, qos, listener=self.state_listener)
        self._left_hand_state_reader = DataReader(subscriber, left_hand_state_topic,
                                                   listener=self.left_hand_state_listener)
        self._right_hand_state_reader = DataReader(subscriber, right_hand_state_topic,
                                                   listener=self.right_hand_state_listener)
        self.hardware_status_reader = DataReader(subscriber, alex_status_topic, listener=self.status_listener)

        # Set up the publishers
        publisher = Publisher(domain_participant)
        self._alex_command_writer = DataWriter(publisher, alex_command_topic, qos)
        self._left_hand_command_writer = DataWriter(publisher, left_hand_command_topic, qos)
        self._right_hand_command_writer = DataWriter(publisher, right_hand_command_topic, qos)

    def run_communication(self, lock: Union[threading.Lock, None] = None, shared_data: Union[Dict[str, Any], None] = None):
        """
        Run the DDS communication loop continuously until the program ends
        :param lock: Threading lock to read and write data to shared memory
        :param shared_data: Dictionary of shared memory data
        """
        try:
            # Run the communication loop until the program is killed
            while True:
                self._num_loops += 1
                curr_time = time.perf_counter_ns()
                # If the threading lock and shared data are provided, read from and write to the shared data
                if lock is not None and shared_data is not None:
                    with lock:
                        # Check and make sure there is data to read from the subscribers
                        if self.state_listener.alex_state is not None:
                            shared_data["alex_state"] = self.state_listener.alex_state
                        if self.status_listener.hardware_status is not None:
                            shared_data["hardware_status"] = self.status_listener.hardware_status
                        if self.left_hand_state_listener.hand_state is not None:
                            shared_data["left_hand_state"] = self.left_hand_state_listener.hand_state
                        if self.right_hand_state_listener.hand_state is not None:
                            shared_data["right_hand_state"] = self.right_hand_state_listener.hand_state

                        # Publish commands to the robot and hands
                        self._alex_command_writer.write(shared_data["alex_command"])
                        self._left_hand_command_writer.write(shared_data["left_hand_command"])
                        self._right_hand_command_writer.write(shared_data["right_hand_command"])

                # Gather statistics. Uncomment if you want to use them
                # self._avg_time += (time.perf_counter_ns() - curr_time) * 1.0e-9
                # if self._num_loops > 10 * self.frequency:
                #     print(self._avg_time / self._num_loops)
                #     print(100.0 * self._missed_loops / self._num_loops)
                #     self._num_loops = 0
                #     self._missed_loops = 0
                #     self._avg_time = 0.0

                # To set loop frequency deterministically, get the elapsed time and have the loop sleep until the end
                elapsed_time = (time.perf_counter_ns() - curr_time) * 1.0e-9
                if elapsed_time < self.dt:
                    time.sleep(self.dt - elapsed_time)
                else:
                    self._missed_loops += 1
        except KeyboardInterrupt:
            print("\nStopping subscription.")




class AlexStateListener(Listener):
    """
    Listener that reads data from the AlexState topic
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.alex_state = None

    def on_data_available(self, reader: DataReader[AlexState]) -> None:
        self.alex_state = reader.read_next()

class HardwareStatusListener(Listener):
    """
    Listener that reads data from the hardware status topic
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.hardware_status = None

    def on_data_available(self, reader: DataReader[HardwareStatus]) -> None:
        self.hardware_status = reader.read_next()

class HandStateListener(Listener):
    """
    Listener that reads data from the hand state topic
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.hand_state = None

    def on_data_available(self, reader: DataReader[EZGripperState]) -> None:
        self.hand_state = reader.read_next()

def main():
    """
    Main function to test out communication
    """
    alex_communication = AlexCommunication()
    alex_communication.run_communication()

if __name__ == "__main__":
    main()