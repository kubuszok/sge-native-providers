/*
 * Stub implementations for GLFW native window handle functions that are
 * unavailable on the current platform.
 *
 * Scala Native requires ALL @extern symbols to be resolvable at link time,
 * even if guarded by runtime platform checks. These stubs satisfy the linker
 * on platforms where the real implementations don't exist.
 *
 * The _GLFW_COCOA / _GLFW_X11 / _GLFW_WIN32 defines are set by build.rs
 * based on the cargo target OS, matching the GLFW build configuration.
 */

#include <stddef.h>

/* Export the stubs from the DLL on Windows, mirroring GLFW's own GLFWAPI
 * convention (__declspec(dllexport) when _GLFW_BUILD_DLL is defined). Without
 * this the stub is compiled into glfw3.dll but NOT placed in the export table,
 * so the import library glfw3.lib lacks it and a Scala Native @link("glfw3")
 * consumer fails with LNK2019 unresolved external (e.g. glfwGetCocoaWindow /
 * glfwGetX11Window on Windows). On ELF/Mach-O global symbols are exported by
 * default, so STUB_EXPORT is empty there. */
#if defined(_GLFW_BUILD_DLL)
  #define STUB_EXPORT __declspec(dllexport)
#else
  #define STUB_EXPORT
#endif

#if !defined(_GLFW_COCOA)
STUB_EXPORT void* glfwGetCocoaWindow(void* window) {
    (void)window;
    return NULL;
}
#endif

#if !defined(_GLFW_X11)
STUB_EXPORT unsigned long long glfwGetX11Window(void* window) {
    (void)window;
    return 0;
}
#endif

#if !defined(_GLFW_WIN32)
STUB_EXPORT void* glfwGetWin32Window(void* window) {
    (void)window;
    return NULL;
}
#endif
