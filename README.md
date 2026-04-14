# sge-native-providers

Pre-built native libraries for [SGE (Scala Game Engine)](https://github.com/kubuszok/sge), packaged as fat JARs for automatic extraction by [multiarch-scala](https://github.com/kubuszok/multiarch-scala).

## Problem

SGE depends on several native libraries (Rust, C, and pre-built binaries) that must be available at link time (Scala Native) or runtime (JVM/Android via Panama FFM). Building these from source requires Rust, zig, cargo-xwin, Android NDK, and ~15 minutes of cross-compilation. This repository builds them once and publishes platform-specific JARs so SGE consumers don't need any native toolchain installed.

## Provider artifacts

| Artifact                            | Contents                                                 | Platforms     | Used by                  |
|-------------------------------------|----------------------------------------------------------|---------------|--------------------------|
| `pnm-provider-sge-desktop`          | `sge_native_ops.{dylib,so,dll}`, `sge_audio.*`, `glfw.*` | 6 desktop     | JVM Panama FFM runtime   |
| `pnm-provider-sge-android`          | `sge_native_ops.so`, `sge_audio.so`                      | 3 Android     | Android Panama FFM       |
| `sn-provider-sge`                   | `sge_native_ops.a`, `sge_audio.a`, `glfw3.a` + stubs     | 6 desktop     | Scala Native linking     |
| `pnm-provider-sge-freetype-desktop` | `sge_freetype.{dylib,so,dll}`                            | 6 desktop     | JVM Panama FFM runtime   |
| `pnm-provider-sge-freetype-android` | `sge_freetype.so`                                        | 3 Android     | Android Panama FFM       |
| `sn-provider-sge-freetype`          | `sge_freetype.a`, `freetype.a`                           | 6 desktop     | Scala Native linking     |
| `pnm-provider-sge-physics-desktop`  | `sge_physics.{dylib,so,dll}`                             | 6 desktop     | JVM Panama FFM runtime   |
| `pnm-provider-sge-physics-android`  | `sge_physics.so`                                         | 3 Android     | Android Panama FFM       |
| `sn-provider-sge-physics`           | `sge_physics.a`                                          | 6 desktop     | Scala Native linking     |
| `pnm-provider-sge-angle` (desktop)  | `libEGL.{dylib,so,dll}`, `libGLESv2.*`                   | 6 desktop     | JVM Panama (OpenGL ES)   |
| `sn-provider-sge-angle`             | Same ANGLE shared libs                                   | 6 desktop     | Scala Native (OpenGL ES) |

Each JAR uses the fat JAR layout expected by `multiarch-scala`:

```
provider.jar
├── sn-provider.json              # or pnm-provider.json
├── native/linux-x86_64/lib*.a    # (or .so / .dylib / .dll)
├── native/linux-aarch64/lib*.a
├── native/macos-x86_64/lib*.a
├── native/macos-aarch64/lib*.a
├── native/windows-x86_64/*.lib
└── native/windows-aarch64/*.lib
```

## Supported platforms

| Classifier         | Target                           | Desktop | Android |
|--------------------|----------------------------------|---------|---------|
| `linux-x86_64`     | `x86_64-unknown-linux-gnu`       | Yes     |         |
| `linux-aarch64`    | `aarch64-unknown-linux-gnu`      | Yes     |         |
| `macos-x86_64`     | `x86_64-apple-darwin`            | Yes     |         |
| `macos-aarch64`    | `aarch64-apple-darwin`           | Yes     |         |
| `windows-x86_64`   | `x86_64-pc-windows-msvc`         | Yes     |         |
| `windows-aarch64`  | `aarch64-pc-windows-msvc`        | Yes     |         |
| `android-aarch64`  | `aarch64-linux-android`          |         | Yes     |
| `android-armv7`    | `armv7-linux-androideabi`        |         | Yes     |
| `android-x86_64`   | `x86_64-linux-android`           |         | Yes     |

## What gets built

The Rust workspace in `native-components/` produces:

- **sge_native_ops** — buffer operations, ETC1 codec, image decoding (PNG/JPEG/BMP)
- **sge_audio** — miniaudio audio bridge (vendored C, public domain)
- **glfw3** — GLFW windowing library (vendored C source, zlib license)
- **sge_freetype** — FreeType font rasterization bindings
- **sge_physics** — Rapier2D physics engine wrapper
- **ANGLE** — EGL + GLESv2 OpenGL ES implementation (pre-built, downloaded from [sge-angle-natives](https://github.com/kubuszok/sge-angle-natives))

## Usage in SGE

```scala
// In build.sbt (Scala Native axis)
// sn-provider-sge transitively pulls sn-provider-sge-angle
libraryDependencies ++= Seq(
  "com.kubuszok" % "sn-provider-sge"          % version,
  "com.kubuszok" % "sn-provider-sge-freetype" % version,
  "com.kubuszok" % "sn-provider-sge-physics"  % version,
  "com.kubuszok" % "sn-provider-curl"         % curlVersion
)

// In build.sbt (JVM/Panama axis)
// pnm-provider-sge-desktop transitively pulls pnm-provider-sge-angle
libraryDependencies ++= Seq(
  "com.kubuszok" % "pnm-provider-sge-desktop"          % version,
  "com.kubuszok" % "pnm-provider-sge-freetype-desktop" % version,
  "com.kubuszok" % "pnm-provider-sge-physics-desktop"  % version
)

// In build.sbt (Android axis)
libraryDependencies ++= Seq(
  "com.kubuszok" % "pnm-provider-sge-android"          % version,
  "com.kubuszok" % "pnm-provider-sge-freetype-android" % version,
  "com.kubuszok" % "pnm-provider-sge-physics-android"  % version
)
```

The `NativeProviderPlugin` from `sbt-multiarch-scala` automatically discovers `sn-provider.json` manifests, extracts platform-specific `.a` files, and configures Scala Native's `nativeConfig` with the correct linker flags.

For JVM/Android, `NativeLibLoader` from `multiarch-core` discovers `pnm-provider.json` manifests and loads shared libraries at runtime.

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
