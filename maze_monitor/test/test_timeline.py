from maze_monitor.timeline import Event, Series, Timeline


def test_series_value_at_returns_last_value_at_or_before():
    s = Series('k', 'k')
    for t, v in ((1.0, 10.0), (2.0, 20.0), (3.0, 30.0)):
        s.add(t, v)
    assert s.value_at(0.5) is None
    assert s.value_at(1.0) == 10.0
    assert s.value_at(2.9) == 20.0
    assert s.value_at(10.0) == 30.0
    assert s.value_at(10.0, max_age=1.0) is None


def test_series_min_interval_keeps_newest_value():
    s = Series('k', 'k', min_interval=1.0)
    s.add(0.0, 1.0)
    s.add(0.5, 2.0)
    s.add(1.2, 3.0)
    assert list(s.times) == [0.0, 1.2]
    assert s.value_at(0.9) == 2.0


def test_text_series_stores_changes_only():
    s = Series('k', 'k', numeric=False)
    for t, v in ((0, 'a'), (1, 'a'), (2, 'b')):
        s.add(t, v)
    assert list(s.values) == ['a', 'b']


def test_track_lookup_and_rate():
    tl = Timeline()
    for i in range(50):
        tl.add_message(i * 0.1, '/x', 'std_msgs/msg/String', b'abcd')
    tr = tl.tracks['/x']
    assert tr.index_at(-1) == -1
    assert tr.index_at(0.25) == 2
    assert abs(tr.rate_at(4.9, 2.0) - 10.0) < 0.6
    assert tl.duration >= 4.9


def test_condition_event_activity():
    e = Event(0, 5.0, 'warning', 'x', 'title', is_condition=True, t_end=7.0)
    assert not e.active_at(4.9)
    assert e.active_at(6.0)
    assert not e.active_at(7.1)
    instant = Event(1, 5.0, 'info', 'x', 'title')
    assert instant.active_at(9.0)
    assert not instant.active_at(11.0)


def test_next_and_previous_problem():
    tl = Timeline()
    tl.add_event(Event(0, 1.0, 'info', 'x', 'a'))
    tl.add_event(Event(0, 2.0, 'warning', 'x', 'b'))
    tl.add_event(Event(0, 3.0, 'error', 'x', 'c'))
    assert tl.next_event_time(0.0) == 2.0
    assert tl.next_event_time(2.0) == 3.0
    assert tl.prev_event_time(3.0) == 2.0
    assert tl.prev_event_time(2.0) is None
