// SGE Native Ops — Core Rust native library
//
// Modules:
//   etc1       — ETC1 texture compression/decompression (port of etc1_utils.cpp)
//   buffer_ops — Memory copy, vertex transforms, vertex find/compare, memory management
//   audio      — Audio C ABI stubs (unconditional — no external dependency)
//   gdx2d      — Image decoding via `image` crate (behind "image_decode" feature flag)
//
// C ABI functions are exported for:
//   - Desktop JVM via Panama FFM (java.lang.foreign)
//   - Android via PanamaPort (com.v7878.foreign — backport of Panama FFM for ART)
//   - Scala Native via @extern
//
// FreeType and physics are separate crates in this workspace:
//   sge-native-freetype → libsge_freetype
//   sge-native-physics  → libsge_physics

pub mod buffer_ops;
pub mod etc1;

pub mod audio;

#[cfg(feature = "image_decode")]
pub mod gdx2d;

// Note: Controller/joystick FFI is handled directly by the sge-controllers
// extension via GLFW calls (Scala Native @link("glfw3") / JVM Panama).
// No Rust wrapper needed since GLFW is already linked by sge-core.
