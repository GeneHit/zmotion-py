import functools
import logging
import pytz
from collections.abc import Callable
from datetime import datetime
from typing import Any, TypeVar, cast

CallableT = TypeVar("CallableT", bound=Callable[..., Any])


def get_current_datetime(time_zone: str = "UTC") -> datetime:
    return datetime.now(tz=pytz.timezone(time_zone))


def log_exception(fn: CallableT) -> CallableT:
    """Log the exception before re-reraising.

    This is especially useful when `fn` is running in a background thread and
    the exception may be lost otherwise.
    """

    @functools.wraps(fn)
    def wrapped(*args: Any, **kwargs: Any) -> Any:
        try:
            return fn(*args, **kwargs)
        except Exception:
            logging.exception(f"Error while calling {fn}")
            raise

    return cast(CallableT, wrapped)
