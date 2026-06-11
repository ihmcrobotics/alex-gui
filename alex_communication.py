import os
import threading
import time
from typing import Dict, Any, Union

from cyclonedds.core import Listener
from cyclonedds.domain import DomainParticipant
from cyclonedds.qos import Qos, Policy
from cyclonedds.sub import Subscriber, DataReader
from cyclonedds.topic import Topic

from messages import AlexState, IMUState, ForceTorqueState, HardwareStatus


class AlexCommunication:
    def __init__(self, ip_address: str = "10.43.3.4", domain_id: int = 42,  frequency: float = 100.0):
        os.environ["CYCLONEDDS_URI"] = ("<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"" +
                                        ip_address + "\"/></Interfaces></General></Domain></CycloneDDS>")
        print(os.environ.get("CYCLONEDDS_URI"))
        self.alex_state = None

        self.frequency = frequency
        self.dt = 1.0/frequency
        qos = Qos(Policy.Reliability.Reliable(max_blocking_time=1))
        qos.reliability = Policy.Reliability.Reliable
        domain_participant = DomainParticipant(domain_id, qos)
        alex_state_topic = Topic(domain_participant, "rt/alex_state", AlexState)
        # alex_status_topic = Topic(domain_participant, "rt/hardware_status", HardwareStatus)
        self.state_listener = AlexStateListener()
        # self.status_listener = HardwareStatusListener()
        subscriber = Subscriber(domain_participant)
        self.alex_state_reader = DataReader(subscriber, alex_state_topic, qos, listener=self.state_listener)
        # self.hardware_status_reader = DataReader(subscriber, alex_status_topic, qos, listener=self.status_listener)

    def run_communication(self, lock: Union[threading.Lock, None] = None, shared_data: Union[Dict[str, Any], None] = None):
        try:
            while True:
                curr_time = time.perf_counter_ns()
                # print("reader guid:", self.alex_state_reader.guid)
                # print("matched:", self.alex_state_reader.get_matched_publications())
                if lock is not None:
                    with lock:
                        if self.state_listener.alex_state is not None:
                            joint_states = self.state_listener.alex_state.joint_states
                            shared_data["alex_state"] = self.state_listener.alex_state
                            # append = len(shared_data["joint_states"]) < 1
                            # print(append)
                            for i in range(len(joint_states)):
                                shared_data["joint_states"][joint_states[i].joint_name] = joint_states[i]
                            # shared_data["hardware_status"] = self.status_listener.hardware_status
                            # print(self.status_listener.hardware_status)
                            # print("updated state")
                elapsed_time = (curr_time - time.perf_counter_ns()) * 1.0e-9
                if elapsed_time < self.dt:
                    time.sleep(self.dt - elapsed_time)
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
        # print(self.alex_state)

class HardwareStatusListener(Listener):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # self.alex_state = alex_state
        self.hardware_status = None

    def on_data_available(self, reader: DataReader[HardwareStatus]) -> None:
        self.hardware_status = reader.read_next()
        print("status received")
        # print(self.alex_state)


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
    # os.environ["CYCLONEDDS_URI"] = "<CycloneDDS><Domain><General><Interfaces><NetworkInterface address=\"10.43.3.4\"/></Interfaces></General></Domain></CycloneDDS>"
    # print(os.environ.get("CYCLONEDDS_URI"))
    # print(os.environ.get("ROS_DOMAIN_ID"))
    # qos = Qos(Policy.Reliability.Reliable(max_blocking_time=1))
    # qos.reliability = Policy.Reliability.Reliable
    # domain_participant = DomainParticipant(get_rtps_domain_id(), qos)
    # alex_state_topic = Topic(domain_participant, "rt/alex_state", AlexState)
    # state_listener = AlexStateListener()
    # subscriber = Subscriber(domain_participant)
    # alex_state_reader = DataReader(subscriber, alex_state_topic, qos, listener=state_listener)



    try:
        while True:
            # Read arriving samples (non-blocking, returns a list)
            # print("reader guid:", alex_state_reader.guid)
            # print("matched:", alex_state_reader.get_matched_publications())

            # samples = alex_state_reader.read()
            #
            # print("read returned", len(samples), "samples")
            # print(samples)

            # for sample in samples:
            #     print("sample:", sample)
            #     print("time: ", sample.time)
            #     print("joint_states: ", sample.joint_states)
            #     print(f"Received ID: {sample.id} | Message: {sample.message}")

            # Prevent high CPU utilization in the loop
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopping subscription.")

if __name__ == "__main__":
    main()