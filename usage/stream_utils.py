import queue
import threading
from collections.abc import Generator
from typing import Generic, Literal, Tuple, TypeVar, Union, cast

from usage.utils import log_exception

T = TypeVar("T")

REFRESH_INTERVAL_S = 1.0
"""Minimum update interval for EventStream and SubscribableValue, even if 
there has been no new events / value updates.

This minimum update interval is needed so that the subscriber won't be stuck 
forever waiting for the next value, and
gets a chance to check whether it should exit.

When there is indeed a new event / value update, new data will be streamed 
immediately. Therefore this minimum interval
can be a slightly longer duration like 1 second.
"""


class _EventSubscriber(Generic[T]):
    """A multiple-producer, single-consumer channel for typed events.

    The producer threads may call `on_next` / `on_error` to generate events 
    or errors.

    The consumer thread may call `subscribe` to subscribe to the events or 
    raise the received error.

    There should be only one thread calling `subscribe`. Otherwise, only one of 
    the threads will receive any particular data.
    """

    def __init__(self) -> None:
        self._queue: "queue.Queue[Tuple[bool, Union[T, Exception]]]" = queue.Queue()

    def on_next(self, event: T) -> None:
        """Feed a new event to the subscriber thread."""
        self._queue.put((True, event))

    def on_error(self, error: Exception) -> None:
        """Feed an exception to be raised on the subscriber thread."""
        self._queue.put((False, error))

    def subscribe(self) -> Generator[tuple[Literal[False], None] | tuple[Literal[True], T], None, None]:
        """See `EventStream.subscribe`. The same semantics is shared."""
        while True:
            try:
                ok, data_or_error = self._queue.get(timeout=REFRESH_INTERVAL_S)
            except queue.Empty:
                # mypy is not smart enough to infer the type of literal boolean values (would infer `bool` instead).
                # same below for the literal `True` value.
                yield cast(Literal[False], False), None
                continue
            if ok:
                yield cast(Literal[True], True), cast(T, data_or_error)
            else:
                if not isinstance(data_or_error, BaseException):
                    raise ValueError(f"Invalid exception value attempted to be raised: {data_or_error}")
                raise data_or_error


class EventStream(Generic[T]):
    """A multiple-producer, multiple-consumer channel for typed events.

    Examples
    --------
    Construct a new event stream:
    >>> event_stream = EventStream()

    Subscribe to events:
    >>> for has_event, event in event_stream.subscribe():
    >>>     ...

    Feed a new event to all subscribers:
    >>> event_stream.on_next(event)

    Notes
    -----
    See `SubscribableValue` for some comparison between the two classes.
    """

    def __init__(self) -> None:
        self._queue: "queue.Queue[Tuple[bool, Union[T, Exception]]]" = queue.Queue()
        self._stop_subscribe = threading.Event()
        self._running = True
        self._subscribers: set[_EventSubscriber[T]] = set()
        self._publisher_loop_worker = threading.Thread(target=self._loop, daemon=True)
        self._publisher_loop_worker.start()

    def close(self) -> None:
        self._stop_subscribe.set()
        self._running = False
        self._publisher_loop_worker.join()
        self._subscribers.clear()

    def __del__(self) -> None:
        self.close()

    @log_exception
    def _loop(self) -> None:
        while self._running:
            try:
                ok, value = self._queue.get(timeout=REFRESH_INTERVAL_S)
                # Make a copy in case a new subscriber joined when iterating.
                subscribers = list(self._subscribers)
                if ok:
                    for subscriber in subscribers:
                        subscriber.on_next(cast(T, value))
                else:
                    for subscriber in subscribers:
                        subscriber.on_error(cast(Exception, value))
            except queue.Empty:
                continue

    def on_next(self, event: T) -> None:
        """Feed a new event to all subscribers."""
        self._queue.put((True, event))

    def on_error(self, error: Exception) -> None:
        """Feed the given error to all subscribers."""
        self._queue.put((False, error))

    def has_subscribers(self) -> bool:
        """Return whether there are currently any subscribers."""
        return len(self._subscribers) > 0

    def subscribe(self) -> Generator[tuple[Literal[False], None] | tuple[Literal[True], T], None, None]:
        """Subscribe to event updates, and raise an exception if received any.

        A tuple will be yielded at some minimum (approximately) guaranteed frequency. The tuple is of the form:
        * (True, new_event) if there is a new event; or
        * (False, None) if there are no new events.

        The minimum frequency is set so that the caller may terminate the loop without getting stuck. If there are no
        new events, the tuple (False, None) may be yielded repeatedly until terminated. If there are many events,
        all of them will be yielded at potentially a faster frequency than the minimum guaranteed frequency.
        """
        subscriber: _EventSubscriber[T] = _EventSubscriber()
        self._subscribers.add(subscriber)
        try:
            for value in subscriber.subscribe():
                if self._stop_subscribe.is_set():
                    break
                yield value
        finally:
            try:
                self._subscribers.remove(subscriber)
            except KeyError:
                # This may happen if close() is already called and self._subscribers is already cleared.
                pass


class SubscribableValue(Generic[T]):
    """A thread-safe container for a typed value whose updates may be subscribed from multiple threads.

    Examples
    --------
    Construct a subscribable value:
    >>> subscribable_value = SubscribableValue(initial_value=...)

    Set new value:
    >>> subscribable_value.set_value(new_value=...)

    Get the current value:
    >>> subscribable_value.get_value()

    Subscribe to value updates (see docstring of `subscribe` for details):
    >>> for value in subscribable_value.subscribe():
    >>>     ...

    Notes
    -----
    Comparison between `EventStream` and `SubscribableValue`:
    * When there is a new event fed to an `EventStream`, each subscriber will only receive it once; when a new value is
      set on a `SubscribableValue`, each subscriber will continue receiving this value until it's updated again.
    * When a new subscriber subscribes to an `EventStream`, it will not receive any events until a new event is fed;
      when a new subscriber subscribes to a `SubscribableValue`, it will continuously receive the latest value even if
      the value has not been updated.
    """

    def __init__(self, initial_value: T, name: str | None = None, should_log_changes: bool = False):
        self._name = name
        self._should_log_changes = should_log_changes
        self._event_stream: EventStream[T] = EventStream()
        self._value = initial_value
        self._queue: "queue.Queue[Tuple[bool, Union[T, Exception]]]" = queue.Queue()
        self._queue.put((True, initial_value))
        self._value_lock = threading.RLock()
        self._running = True
        self._publisher_loop_worker = threading.Thread(target=self._loop, daemon=True)
        self._publisher_loop_worker.start()

    def close(self) -> None:
        self._running = False
        self._publisher_loop_worker.join()
        self._event_stream.close()

    def __del__(self) -> None:
        self.close()

    @log_exception
    def _loop(self) -> None:
        # Get the very first value.
        ok, prev_value = self._queue.get(timeout=REFRESH_INTERVAL_S)
        assert ok
        self._event_stream.on_next(cast(T, prev_value))
        while self._running:
            try:
                ok, value = self._queue.get(timeout=REFRESH_INTERVAL_S)
                if ok:
                    self._event_stream.on_next(cast(T, value))
                    prev_value = value
                else:
                    self._event_stream.on_error(cast(Exception, value))
            except queue.Empty:
                self._event_stream.on_next(cast(T, prev_value))

    def set_value(self, new_value: T) -> None:
        """Set new value, and feed it to all subscriber threads."""
        with self._value_lock:
            if self._value != new_value:
                if self._should_log_changes:
                    print(f"SubscribableValue {self._name} changed from {self._value} to {new_value}")
                self._value = new_value
                self._queue.put((True, new_value))

    def set_error(self, error: Exception) -> None:
        """Feed the given error to all subscriber threads."""
        self._queue.put((False, error))

    def get_value(self) -> T:
        """Get the current value."""
        return self._value

    def has_subscribers(self) -> bool:
        """Return whether there is currently any subscribers."""
        return self._event_stream.has_subscribers()

    def subscribe(self) -> Generator[T, None, None]:
        """Subscribe to value updates.

        Values will be yielded at a minimum (approximately) guaranteed frequency, regardless of whether the value has
        been updated. If the value has not been updated, the current value may be yielded repeatedly at a fixed
        frequency. On the other hand, if the value is being updated at a high frequency, the updates will not be missed
        (i.e. they will be yielded at potentially a higher frequency).
        """
        # Yield the current value immediately.
        yield self.get_value()
        for has_event, value in self._event_stream.subscribe():
            if has_event:
                yield cast(T, value)
