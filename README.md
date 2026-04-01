# sge-native-components

Pre-built native libraries for [SGE (Scala Game Engine)](https://github.com/kubuszok/sge), packaged as fat JARs for automatic extraction by [sbt-multi-arch-release](https://github.com/kubuszok/sbt-multi-arch-release).

## Problem

SGE depends on several native libraries (Rust, C, and pre-built binaries) that must be available at link time (Scala Native) or runtime (JVM via Panama FFM). Building these from source requires Rust, zig, cargo-xwin, Android NDK, and ~15 minutes of cross-compilation. This repository builds them once and publishes platform-specific JARs so SGE consumers don't need any native toolchain installed.

## Provider artifacts

| Artifact | Contents | Platforms | Used by |
|----------|----------|-----------|---------|
| `scala-native-sge-ops-provider` | `sge_native_ops.a`, `sge_audio.a`, `glfw3.a`, `freetype.a` | 6 desktop | Scala Native linking |
| `panama-sge-ops-provider` | `sge_native_ops.{dylib,so,dll}`, `sge_audio.*`, `glfw.*` | 6 desktop | JVM Panama FFM runtime |
| `android-sge-ops-provider` | `sge_native_ops.so`, `sge_audio.so` | 3 Android ABIs | Android JNI |
| `scala-native-angle-provider` | `libEGL.{dylib,so,dll}`, `libGLESv2.*` | 6 desktop | Scala Native (OpenGL ES) |
| `panama-angle-provider` | Same ANGLE shared libs | 6 desktop | JVM Panama (OpenGL ES) |

Each JAR uses the fat JAR layout expected by `sbt-multi-arch-release`:

```
provider.jar
├── native-bundle.json          # manifest with linker flags
├── native/linux-x86_64/lib*.a
├── native/linux-aarch64/lib*.a
├── native/macos-x86_64/lib*.a
├── native/macos-aarch64/lib*.a
├── native/windows-x86_64/*.lib
└── native/windows-aarch64/*.lib
```

## Supported platforms

| Classifier | Target | Desktop | Android |
|------------|--------|---------|---------|
| `linux-x86_64` | `x86_64-unknown-linux-gnu` | ✓ | |
| `linux-aarch64` | `aarch64-unknown-linux-gnu` | ✓ | |
| `macos-x86_64` | `x86_64-apple-darwin` | ✓ | |
| `macos-aarch64` | `aarch64-apple-darwin` | ✓ | |
| `windows-x86_64` | `x86_64-pc-windows-msvc` | ✓ | |
| `windows-aarch64` | `aarch64-pc-windows-msvc` | ✓ | |
| `android-aarch64` | `aarch64-linux-android` | | ✓ |
| `android-armv7` | `armv7-linux-androideabi` | | ✓ |
| `android-x86_64` | `x86_64-linux-android` | | ✓ |

## What gets built

The Rust crate `sge-native-ops` produces:

- **sge_native_ops** — buffer operations, ETC1 codec, image decoding (PNG/JPEG/BMP), FreeType font bindings, Rapier2D physics engine
- **sge_audio** — miniaudio audio bridge (vendored C, public domain)
- **glfw3** — GLFW windowing library (vendored C source, zlib license)

Additionally:

- **freetype** — FreeType font rasterizer (built by `freetype-sys` crate; system library on macOS, vendored on cross-compiled targets)
- **ANGLE** — EGL + GLESv2 OpenGL ES implementation (pre-built, downloaded from [sge-angle-natives](https://github.com/kubuszok/sge-angle-natives))

## Usage in SGE

```scala
// In build.sbt (Scala Native axis)
libraryDependencies ++= Seq(
  "com.kubuszok" % "scala-native-sge-ops-provider" % version,
  "com.kubuszok" % "scala-native-angle-provider" % version,
  "com.kubuszok" % "scala-native-curl-provider" % curlVersion
)
```

The `NativeLibBundlePlugin` from `sbt-multi-arch-release` automatically discovers `native-bundle.json` manifests, extracts platform-specific `.a` files, and configures Scala Native's `nativeConfig` with the correct linker flags.

## Building locally

Prerequisites: Rust, zig, cargo-zigbuild, cargo-xwin, LLVM, Android NDK

```bash
# Build all 6 desktop targets
scripts/cross-all.sh

# Download ANGLE shared libraries
scripts/download-angle.sh

# Build 3 Android targets
ANDROID_NDK_HOME=/path/to/ndk scripts/cross-android.sh

# Package into provider JARs
sbt publishLocal
```

## License

Apache 2.0
