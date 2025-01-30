# zmotion_py
Python bindings of the ZMotion control card library

Zmotion office: http://www.zmotionglobal.com/

## Information of the ZMotion library.

The folder `./lib/` has the include `zmotion.h` and library `libzmotion.so`, 
which are got from the ZMotion.
The current version we use is (v2.1, linux, c++, x64).

## Environment setup
Make sure `Cython` has been installed.

## Usage
To use the raw zmotion bindings `zmotion_py`:

```python
from runtime_compilation.zmotion_py import (
    Handle_Wrapper,
    zmotion_close,
    zmotion_direct,
    zmotion_execute,
    zmotion_init,
    zmotion_open_ethernet,
)
```
An example is:

```python
import threading
from runtime_compilation.zmotion_py import (
    Handle_Wrapper,
    zmotion_close,
    zmotion_direct,
    zmotion_execute,
    zmotion_init,
    zmotion_open_ethernet,
)

ip_address = "0.0.0.0"
zmotion_init()
controller_handle = Handle_Wrapper()
connect_success = zmotion_open_ethernet(ip_address, controller_handle)
if not connect_success == 0:
    raise ConnectionError(f"Failed to connect to motion controller at ip: {ip_address}.")
print(f"Connected to motion control card {ip_address}")
cmd_lock = threading.Lock()


def send_cmd(cmd: str, direct_or_execute: str = "direct") -> str:
    """Send a command to the motion command card practicality.

    Now, the duration of sending one command to the motion control card is about 0.15ms - 0.35ms.

    Parameters
    ----------
    cmd: str
        The sent command.
    direct_or_execute: str
        "direct": direct command interface, used for motion functions or some io operation.
        "execute": universal command execution interface.

    Returns
    -------
    response : str
        The response detail.
    """
    with cmd_lock:
        if direct_or_execute == "direct":
            # direct command interface, only supports motion functions, parameters and array variables configuration
            success, response = zmotion_direct(controller_handle, cmd)
        else:
            # Universal command execution interface, getting block when the controller is not buffered
            success, response = zmotion_execute(controller_handle, cmd)

    if not success == 0:
        raise ConnectionError(
            f"{'ZMC_DirectCommand' if direct_or_execute == 'direct' else 'ZMC_Execute'} failed to send {cmd} "
            f"with {success} status. Its response is {response}."
        )
    return response


send_cmd(cmd=f"?IN({11})")
zmotion_close(controller_handle)
```

## Add new interface

You can define a new interface as what the file `zmotion_py.pyx` does based on the include `zmotion.h`.
