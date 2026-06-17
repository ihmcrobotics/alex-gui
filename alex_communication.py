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

from messages import *


class AlexCommunication:
    def __init__(self, ip_address: str = "10.43.3.6", domain_id: int = 42,  frequency: float = 100.0):
        os.environ["CYCLONEDDS_URI"] = "<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"" + ip_address + "\"/></Interfaces></General></Domain></CycloneDDS>"
        print(os.environ.get("CYCLONEDDS_URI"))
        self.alex_state = None

        self.frequency = frequency
        self.dt = 1.0/frequency
        qos = Qos(Policy.Reliability.Reliable(max_blocking_time=1000))
        qos.reliability = Policy.Reliability.Reliable
        domain_participant = DomainParticipant(domain_id, qos)
        alex_state_topic = Topic(domain_participant, "rt/alex_state", AlexState)
        alex_command_topic = Topic(domain_participant, "rt/alex_command", AlexCommand)
        left_hand_command_topic = Topic(domain_participant, "rt/ezgripper/left/command", EZGripperCommand)
        right_hand_command_topic = Topic(domain_participant, "rt/ezgripper/right/command", EZGripperCommand)
        left_hand_state_topic = Topic(domain_participant, "rt/ezgripper/left/state", EZGripperState)
        right_hand_state_topic = Topic(domain_participant, "rt/ezgripper/right/state", EZGripperState)
        hand_angle_topic = Topic(domain_participant, "rt/ihmc/alex/humanoid_control/output/hand_joint_angle", HandJointAnglePacket)
        alex_status_topic = Topic(domain_participant, "rt/hardware_status", HardwareStatus)
        self.state_listener = AlexStateListener()
        self.left_hand_state_listener = HandStateListener()
        self.right_hand_state_listener = HandStateListener()
        self.hand_angle_listener = HandJointAngleListener()
        self.status_listener = HardwareStatusListener()
        subscriber = Subscriber(domain_participant)
        self._alex_state_reader = DataReader(subscriber, alex_state_topic, qos, listener=self.state_listener)
        self._left_hand_state_reader = DataReader(subscriber, left_hand_state_topic,
                                                   listener=self.left_hand_state_listener)
        self._right_hand_state_reader = DataReader(subscriber, right_hand_state_topic,
                                                   listener=self.right_hand_state_listener)
        self._hand_angle_reader = DataReader(subscriber, hand_angle_topic, listener=self.hand_angle_listener)

        publisher = Publisher(domain_participant)
        self._alex_command_writer = DataWriter(publisher, alex_command_topic, qos)
        self._left_hand_command_writer = DataWriter(publisher, left_hand_command_topic, qos)
        self._right_hand_command_writer = DataWriter(publisher, right_hand_command_topic, qos)
        self.hardware_status_reader = DataReader(subscriber, alex_status_topic, listener=self.status_listener)

    def run_communication(self, lock: Union[threading.Lock, None] = None, shared_data: Union[Dict[str, Any], None] = None):
        try:
            while True:
                curr_time = time.perf_counter_ns()
                # print("reader guid:", self.alex_state_reader.guid)
                # print("matched:", self.alex_state_reader.get_matched_publications())
                alex_command = None
                left_hand_command = None
                right_hand_command = None
                if lock is not None and shared_data is not None:
                    with lock:
                        if self.state_listener.alex_state is not None:
                            shared_data["alex_state"] = self.state_listener.alex_state
                        if self.status_listener.hardware_status is not None:
                            shared_data["hardware_status"] = self.status_listener.hardware_status
                        if self.left_hand_state_listener.hand_state is not None:
                            shared_data["left_hand_state"] = self.left_hand_state_listener.hand_state
                        if self.right_hand_state_listener.hand_state is not None:
                            shared_data["right_hand_state"] = self.right_hand_state_listener.hand_state
                        if self.hand_angle_listener.hand_angles is not None:
                            shared_data["hand_angles"] = self.hand_angle_listener.hand_angles

                        alex_command = shared_data["alex_command"]
                        left_hand_command = shared_data["left_hand_command"]
                        right_hand_command = shared_data["right_hand_command"]

                if alex_command is not None:
                    self._alex_command_writer.write(alex_command)
                if left_hand_command is not None:
                    self._left_hand_command_writer.write(left_hand_command)
                if right_hand_command is not None:
                    self._right_hand_command_writer.write(right_hand_command)
                elapsed_time = (time.perf_counter_ns() - curr_time) * 1.0e-9
                if elapsed_time < self.dt:
                    time.sleep(self.dt - elapsed_time)
                else:
                    print("Missed comms loop")
        except KeyboardInterrupt:
            print("\nStopping subscription.")




class AlexStateListener(Listener):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # self.alex_state = alex_state
        self.alex_state = None

    def on_data_available(self, reader: DataReader[AlexState]) -> None:
        self.alex_state = reader.read_next()
        # print("state received")

class HardwareStatusListener(Listener):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # self.alex_state = alex_state
        self.hardware_status = None

    def on_data_available(self, reader: DataReader[HardwareStatus]) -> None:
        self.hardware_status = reader.read_next()
        # print("status received")

class HandStateListener(Listener):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.hand_state = None

    def on_data_available(self, reader: DataReader[EZGripperState]) -> None:
        self.hand_state = reader.read_next()
        # print("hand state received")

class HandJointAngleListener(Listener):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.hand_angles = None

    def on_data_available(self, reader: DataReader[HandJointAnglePacket]) -> None:
        self.hand_angles = reader.read_next()
        print("angle received")


def get_rtps_domain_id(config_path=os.path.expanduser("~/.ihmc/IHMCNetworkParameters.ini")):
    domain_id = 0  # default fallback
    if os.path.exists(config_path):
        with open(config_path, "r") as f:
            for line in f:
                if line.strip().startswith("RTPSDomainID"):
                    try:
                        domain_id = int(line.split("=")[1].strip())
                    except ValueError:
                        domain_id = 0
                    break
    return domain_id

def main(argv=None):
    alex_communication = AlexCommunication()
    alex_communication.run_communication()

    # If you want to test out the manual communication, use this code
    # os.environ["CYCLONEDDS_URI"] = "<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"10.43.3.6\"/></Interfaces></General></Domain></CycloneDDS>"
    # print(os.environ.get("CYCLONEDDS_URI"))
    # # print(os.environ.get("ROS_DOMAIN_ID"))
    # qos = Qos(Policy.Reliability.Reliable(max_blocking_time=1))
    # qos.reliability = Policy.Reliability.Reliable
    # # best_effort_qos = Qos(Policy.)
    # domain_participant = DomainParticipant(get_rtps_domain_id())
    # alex_status_topic = Topic(domain_participant, "rt/ihmc/alex/humanoid_control/output/hand_joint_angle", HandJointAnglePacket)
    # # alex_state_topic = Topic(domain_participant, "rt/alex_state", AlexState)
    # # state_listener = AlexStateListener()
    # subscriber = Subscriber(domain_participant)
    # alex_state_reader = DataReader(subscriber, alex_status_topic) #, listener=state_listener)
    #
    #
    #
    # try:
    #     while True:
    #         # Read arriving samples (non-blocking, returns a list)
    #         print("reader guid:", alex_state_reader.guid)
    #         print("matched:", alex_state_reader.get_matched_publications())
    #
    #         samples = alex_state_reader.read()
    #
    #         print("read returned", len(samples), "samples")
    #         print(samples)
    #
    #         for sample in samples:
    #             print("sample:", sample)
    #             # print("time: ", sample.time)
    #         #     print("joint_states: ", sample.joint_states)
    #         #     print(f"Received ID: {sample.id} | Message: {sample.message}")
    #
    #         # Prevent high CPU utilization in the loop
    #         time.sleep(0.5)
    #
    # except KeyboardInterrupt:
    #     print("\nStopping subscription.")

if __name__ == "__main__":
    main()