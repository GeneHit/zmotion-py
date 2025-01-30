import numpy as np
import sys
from libcpp cimport bool
cimport numpy as np
from cython.operator cimport dereference as deref
from functools import partial
from cython.parallel import parallel, prange
from libc.stdlib cimport malloc, free
import copy


cdef class Handle_Wrapper:
    cdef void ** handle_zmc  # Hold a C++ instance which we're wrapping

    def __cinit__(self):
        self.handle_zmc = <void **> malloc(sizeof(void *))

    def __dealloc__(self):
        free(self.handle_zmc)

    cdef setHandle(self, void** h):
        free(self.handle_zmc)
        self.handle_zmc = h
        return self

cdef extern from "zmotion.h":
    ctypedef int int32
    ctypedef int uint16
    ctypedef int uint32
    ctypedef char uint8

    bool __stdcall ZMC_LinuxLibInit()
    int32 __stdcall ZMC_OpenEth(char *ipaddr, void **phandle)
    int32 __stdcall ZMC_Close(void *handle);
    int32 __stdcall ZMC_DirectCommand(void *handle, const char *pszCommand, char *psResponse, uint32 uiResponseLength);
    int32 __stdcall ZMC_Execute(void *handle, const char *pszCommand, uint32 uimswait, char *psResponse, uint32 uiResponseLength);


def zmotion_init() -> None:
    ZMC_LinuxLibInit()


def zmotion_open_ethernet(ipaddr, handle):
    cdef int ret_val
    cdef void** h_p
    ipaddr_in_bytes = str.encode(ipaddr)
    return _zmotion_open_ethernet(ipaddr_in_bytes, handle)

cdef _zmotion_open_ethernet(char *ipaddr, Handle_Wrapper handle):
    ret_val = ZMC_OpenEth(ipaddr, handle.handle_zmc)
    return ret_val


def zmotion_close(handle):
    ret_val = _zmotion_close(handle)
    return ret_val

cdef _zmotion_close(Handle_Wrapper handle):
    ret_val = ZMC_Close(deref(handle.handle_zmc))
    return ret_val


def zmotion_direct(handle, cmd):
    cmd_in_bytes = str.encode(cmd)
    ret_val, response = _zmotion_direct(handle, cmd_in_bytes)
    return ret_val, response

cdef _zmotion_direct(Handle_Wrapper handle, char*cmd):
    cdef char cmdbuff[2048]
    ret_val = ZMC_DirectCommand(deref(handle.handle_zmc), cmd, cmdbuff, 2048)
    return ret_val, cmdbuff.decode('iso-8859-1')


def zmotion_execute(handle, cmd):
    cmd_in_bytes = str.encode(cmd)
    ret_val, response = _zmotion_execute(handle, cmd_in_bytes)
    return ret_val, response

cdef _zmotion_execute(Handle_Wrapper handle, char*cmd):
    cdef char cmdbuff[2048]
    ret_val = ZMC_Execute(deref(handle.handle_zmc), cmd, 1000, cmdbuff, 2048)
    return ret_val, cmdbuff.decode('iso-8859-1')
