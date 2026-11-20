"""The ROS side of the monitor: topic discovery, subscriptions, and recording.

Runs an rclpy node in a background thread. Every received message is written
to the current Recorder (if any) and put on a queue for the GUI thread.
"""

import queue
import threading

from .clock import now_ns

DISCOVERY_PERIOD = 2.0


class TopicInfo:
    __slots__ = ('name', 'type', 'publishers', 'subscribed', 'error', 'excluded')

    def __init__(self, name, type_name):
        self.name = name
        self.type = type_name
        self.publishers = 0
        self.subscribed = False
        self.error = ''
        self.excluded = False


class RosBridge:
    def __init__(self, config, ros_args=None):
        self.config = config
        self.ros_args = ros_args
        self.queue = queue.SimpleQueue()
        self.topics = {}  # name -> TopicInfo (the ROS graph as last discovered)
        self._subs = {}
        self._last_kept = {}
        self._recorder = None
        self._rec_lock = threading.Lock()
        self._thread = None
        self._node = None
        self._executor = None
        self._owns_context = False

    # ------------------------------------------------------------ lifecycle
    def start(self):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        if not rclpy.ok():
            rclpy.init(args=self.ros_args)
            self._owns_context = True
        self._node = rclpy.create_node('maze_monitor')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._node.create_timer(DISCOVERY_PERIOD, self._discover)
        self._thread = threading.Thread(target=self._spin, name='ros', daemon=True)
        self._thread.start()
        self._executor.create_task(self._discover)

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # shutdown while spinning
            pass

    def stop(self):
        import rclpy
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._owns_context:
            rclpy.try_shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    # ------------------------------------------------------------ recording
    def set_recorder(self, recorder):
        """Swap the recorder (None stops recording). Returns the previous one."""
        with self._rec_lock:
            old, self._recorder = self._recorder, recorder
        return old

    # ------------------------------------------------------------ discovery
    def _discover(self):
        node = self._node
        if node is None:
            return
        try:
            names_types = node.get_topic_names_and_types()
        except Exception:
            return
        seen = set()
        for name, types in names_types:
            if not types:
                continue
            seen.add(name)
            info = self.topics.get(name)
            if info is None:
                info = self.topics[name] = TopicInfo(name, types[0])
            try:
                info.publishers = node.count_publishers(name)
            except Exception:
                info.publishers = 0
            info.excluded = self.config.is_excluded(name)
            if info.excluded or name in self._subs or info.error:
                continue
            if info.publishers == 0:
                continue  # subscribe once someone publishes, with matching QoS
            self._subscribe(info)
        for name, info in self.topics.items():
            if name not in seen:
                info.publishers = 0

    def _qos_for(self, name):
        from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                               ReliabilityPolicy)
        pubs = self._node.get_publishers_info_by_topic(name)
        reliable = bool(pubs) and all(
            p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in pubs)
        latched = bool(pubs) and all(
            p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in pubs)
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)

    def _subscribe(self, info):
        try:
            from rosidl_runtime_py.utilities import get_message
            cls = get_message(info.type)
        except Exception as exc:
            info.error = f'Message type {info.type} is not available: {exc}'
            return
        name, type_name = info.name, info.type
        min_period = 0.0
        limit = self.config.max_rate_hz.get(name)
        if limit:
            min_period = 1.0 / float(limit)

        large, large_period = self.config.large_message_bytes, 1.0 / self.config.large_message_max_hz

        def callback(data, name=name, type_name=type_name, min_period=min_period):
            t_ns = now_ns()
            if len(data) > large and large_period > min_period:
                # Very large messages (debug visualizations, point clouds) fill the disk
                # quickly and rarely help debugging; keep a few of them.
                min_period = large_period
            if min_period:
                last = self._last_kept.get(name)
                if last is not None and (t_ns - last) * 1e-9 < min_period:
                    return
                self._last_kept[name] = t_ns
            with self._rec_lock:
                rec = self._recorder
                if rec is not None:
                    try:
                        rec.write(name, type_name, data, t_ns)
                    except Exception as exc:
                        info.error = f'Recording failed: {exc}'
            self.queue.put((name, type_name, data, t_ns))

        try:
            self._subs[name] = self._node.create_subscription(
                cls, name, callback, self._qos_for(name), raw=True)
            info.subscribed = True
        except Exception as exc:
            info.error = f'Could not subscribe: {exc}'
