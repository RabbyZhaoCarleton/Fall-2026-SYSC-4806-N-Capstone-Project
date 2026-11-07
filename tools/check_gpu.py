#!/usr/bin/env python3
"""Print which renderer OpenGL uses, e.g. a real GPU through D3D12 or llvmpipe (CPU).

Run it in the same terminal you launch Gazebo and RViz from:
    python3 tools/check_gpu.py
"llvmpipe" means software rendering on the CPU, which makes Gazebo and RViz slow.
"""
import ctypes
import os
import sys

try:
    egl = ctypes.CDLL("libEGL.so.1")
    gl = ctypes.CDLL("libGL.so.1")
except OSError as e:
    sys.exit(f"OpenGL libraries not found: {e}")

egl.eglGetDisplay.restype = ctypes.c_void_p
egl.eglGetDisplay.argtypes = [ctypes.c_void_p]
egl.eglInitialize.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p]
egl.eglChooseConfig.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]
egl.eglCreateContext.restype = ctypes.c_void_p
egl.eglCreateContext.argtypes = [ctypes.c_void_p] * 4
egl.eglMakeCurrent.argtypes = [ctypes.c_void_p] * 4
gl.glGetString.restype = ctypes.c_char_p

display = egl.eglGetDisplay(None)
if not egl.eglInitialize(display, None, None):
    sys.exit("Could not initialise EGL (is a display available?)")
egl.eglBindAPI(0x30A2)  # EGL_OPENGL_API
attrs = (ctypes.c_int * 3)(0x3040, 0x0008, 0x3038)  # EGL_RENDERABLE_TYPE = EGL_OPENGL_BIT
config, count = ctypes.c_void_p(), ctypes.c_int()
egl.eglChooseConfig(display, attrs, ctypes.byref(config), 1, ctypes.byref(count))
context = egl.eglCreateContext(display, config, None, None)
egl.eglMakeCurrent(display, None, None, context)

renderer = (gl.glGetString(0x1F01) or b"unknown").decode()
version = (gl.glGetString(0x1F02) or b"unknown").decode()
print(f"Renderer: {renderer}")
print(f"OpenGL:   {version}")
print(f"GALLIUM_DRIVER={os.environ.get('GALLIUM_DRIVER', '(unset)')}  "
      f"MESA_D3D12_DEFAULT_ADAPTER_NAME={os.environ.get('MESA_D3D12_DEFAULT_ADAPTER_NAME', '(unset)')}")
if "llvmpipe" in renderer:
    print("Software rendering (CPU). Run ./setup.sh again, or see Troubleshooting in README.md.")
