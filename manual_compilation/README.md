# zmotion-py
Python bindings of the ZMotion control card library

## Prerequisite
Please remember to install these packages if not already installed before proceeding:
 `g++ 5 or higher`, `Cython 0.29.22`, `setuptools 51.3.3`

Cython could be installed by: `pip install Cython`

## Fast Install Proceeding
Clone the project file to somewhere, like:`/opt/zmotion-py`, you can install `zmotion_py` 
library as following terminal operation:

```bash
cd /opt/zmotion-py/
. zmotion_py_build.sh
```
When use this way, you can't delete the folder `/opt/zmotion-py/lib/`, which includes the 
c++ dynamic library `libzmotion.so`. Besides, you can self define this path by edit 
the `zmotion_py_build.sh` where must have the library file `libzmotion.so`.

## Self Proceeding
To build the bindings you need to run
```bash
cd /opt/zmotion-py/
python setup.py install
```

The `libzmotion.so` also needs to be added to the `LD_LIBRARY_PATH` by

 * ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ZMOTION_LIB_PATH$```

where `$ZMOTION_LIB_PATH$` is the absolute path for `libzmotion.so`. For example
 * ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/zmotion-py/lib```

## The architecture of this include shown as
```

cc_library(
    name = "zmotion",
    srcs = ["lib/libzmotion.so"],
    hdrs = ["lib/zmotion.h"],
    includes = ["lib"],
)

pyx_library(
    name = "zmotion_py",
    srcs = ["zmotion_py.pyx",],
    cplus = True,
    deps = [
        "zmotion",
        "@python//:numpy-include",
    ],
)

setup_library(
    name = "setup",
    srcs = ["setup.py",],
)
```
## Usage

To use the raw zmotion bindings `zmotion_py`:

```python
import zmotion_py
```

An example is:
```python
import zmotion_py

ip_address = "192.168.14.11"
zmotion_py.zmotion_init()
controller_handle = zmotion_py.Handle_Wrapper()
success = zmotion_py.zmotion_open_ethernet(ip_address, controller_handle)
if not success == 0:
    raise ConnectionError(f"Failed to connect to motion controller at ip: {ip_address}.")
print(f"Connected to motion control card {ip_address}")

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
            success, response = zmotion_py.zmotion_direct(controller_handle, cmd)
        else:
            # Universal command execution interface, getting block when the controller is not buffered
            success, response = zmotion_py.zmotion_execute(controller_handle, cmd)

    if not success == 0:
        # TODO get the detailed reason of the failure
        raise ConnectionError(
            f"{'ZMC_DirectCommand' if direct_or_execute == 'direct' else 'ZMC_Execute'} failed to send {cmd} "
            f"with {success} status. Its response is {response}."
        )
    return response

send_cmd(cmd=f"?IN({11})")
zmotion_py.zmotion_close(controller_handle)
```

