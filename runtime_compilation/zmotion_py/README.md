# zmotion_py
Python bindings of the ZMotion control card library

Zmotion office: http://www.zmotionglobal.com/

### Information of the ZMotion library.

The folder `./lib/` has the include `zmotion.h` and library `libzmotion.so`, 
which are got from the ZMotion.
The current version we use is (v2.1, linux, c++, x64).

### Environment setup
Make sure `Cython` has been installed.

### Usage
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
where you can change the path based on your file's position


### Add new interface

You can define a new interface as what the file `zmotion_py.pyx` does based on the include `zmotion.h`.
