# zmotion-py
Python bindings of the ZMotion control card library

### Prerequisite
Please remember to install these packages if not already installed before proceeding:
 `g++ 5 or higher`, `Cython 0.29.22`, `setuptools 51.3.3`

Cython could be installed by: `pip install Cython`

### Fast Install Proceeding
Clone the project file to somewhere, like:`/opt/zmotion-py`, you can install `zmotion_py` 
library as following terminal operation:

```bash
cd /opt/zmotion-py/
. zmotion_py_build.sh
```
When use this way, you can't delete the folder `/opt/zmotion-py/lib/`, which includes the 
c++ dynamic library `libzmotion.so`. Besides, you can self define this path by edit 
the `zmotion_py_build.sh` where must have the library file `libzmotion.so`.

### Self Proceeding
To build the bindings you need to run
```bash
cd /opt/zmotion-py/
python setup.py install
```

The `libzmotion.so` also needs to be added to the `LD_LIBRARY_PATH` by

 * ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ZMOTION_LIB_PATH$```

where `$ZMOTION_LIB_PATH$` is the absolute path for `libzmotion.so`. For example
 * ```export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/zmotion-py/lib```

### The architecture of this include shown as
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
### Usage

To use the raw zmotion bindings `zmotion_py`:

```python
from zmotion_py import (
    Handle_Wrapper,
    zmotion_close,
    zmotion_direct,
    zmotion_execute,
    zmotion_init,
    zmotion_open_ethernet,
)
```

### Add new interface

You can define a new interface as what the file `zmotion_py.pyx` does based on the include `zmotion.h`.

