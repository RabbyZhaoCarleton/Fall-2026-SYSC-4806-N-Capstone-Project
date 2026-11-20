"""Writing and reading run recordings (rosbag2, SQLite 3 storage).

SQLite is used instead of the default MCAP because a SQLite recording stays
readable when the process is killed during recording; see docs/design.md.
"""

import glob
import os
import threading

STORAGE_ID = 'sqlite3'


def _rosbag2():
    import rosbag2_py
    return rosbag2_py


def _topic_metadata(rb, index, name, type_name):
    try:
        return rb.TopicMetadata(index, name, type_name, 'cdr')
    except TypeError:  # rosbag2 versions without the id argument
        return rb.TopicMetadata(name=name, type=type_name, serialization_format='cdr')


class Recorder:
    """Thread-safe writer for one run recording."""

    def __init__(self, bag_dir):
        rb = _rosbag2()
        self._rb = rb
        self.bag_dir = bag_dir
        self._writer = rb.SequentialWriter()
        try:
            options = rb.StorageOptions(uri=bag_dir, storage_id=STORAGE_ID,
                                        storage_preset_profile='resilient')
            self._writer.open(options, rb.ConverterOptions('cdr', 'cdr'))
        except RuntimeError:
            options = rb.StorageOptions(uri=bag_dir, storage_id=STORAGE_ID)
            self._writer.open(options, rb.ConverterOptions('cdr', 'cdr'))
        self._topics = {}
        self._lock = threading.Lock()
        self._closed = False
        self.messages = 0
        self.bytes = 0

    def write(self, topic, type_name, data, t_ns):
        with self._lock:
            if self._closed:
                return
            if topic not in self._topics:
                self._writer.create_topic(_topic_metadata(self._rb, len(self._topics), topic, type_name))
                self._topics[topic] = type_name
            self._writer.write(topic, data, t_ns)
            self.messages += 1
            self.bytes += len(data)

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            # Closing writes metadata.yaml. Without it, read_bag still opens the .db3 file.
            self._writer.close()
            del self._writer


def _open_reader(bag_dir):
    rb = _rosbag2()
    attempts = [bag_dir] + sorted(glob.glob(os.path.join(bag_dir, '*.db3'))) + \
        sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))
    errors = []
    for uri in attempts:
        storage = 'mcap' if uri.endswith('.mcap') else STORAGE_ID if uri.endswith('.db3') else ''
        reader = rb.SequentialReader()
        try:
            reader.open(rb.StorageOptions(uri=uri, storage_id=storage),
                        rb.ConverterOptions('cdr', 'cdr'))
            return reader
        except RuntimeError as exc:
            errors.append(f'{uri}: {exc}')
    raise RuntimeError('Could not open the recording:\n' + '\n'.join(errors))


def bag_message_count(bag_dir):
    """Number of messages, from metadata if present (fast), else None."""
    try:
        reader = _open_reader(bag_dir)
        return reader.get_metadata().message_count
    except Exception:
        return None


def read_bag(bag_dir):
    """Yield (topic, type name, serialized bytes, receive time in ns) in time order."""
    reader = _open_reader(bag_dir)
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        yield topic, types.get(topic, ''), data, t_ns
