lazy val isCI = sys.env.get("CI").contains("true")
ThisBuild / packageDoc / publishArtifact := false

// Version from git tags
git.useGitDescribe       := true
git.uncommittedSignifier := Some("SNAPSHOT")
git.gitUncommittedChanges := git.gitCurrentTags.value.isEmpty

// Used to publish snapshots to Maven Central.
val mavenCentralSnapshots = "Maven Central Snapshots" at "https://central.sonatype.com/repository/maven-snapshots"

val publishSettings = Seq(
  organization := "com.kubuszok",
  homepage := Some(url("https://github.com/kubuszok/sge-native-components")),
  organizationHomepage := Some(url("https://kubuszok.com")),
  licenses := Seq("Apache-2.0" -> url("https://www.apache.org/licenses/LICENSE-2.0")),
  scmInfo := Some(
    ScmInfo(
      url("https://github.com/kubuszok/sge-native-components/"),
      "scm:git:git@github.com:kubuszok/sge-native-components.git"
    )
  ),
  startYear := Some(2026),
  developers := List(
    Developer("MateuszKubuszok", "Mateusz Kubuszok", "", url("https://github.com/MateuszKubuszok"))
  ),
  pomExtra := (
    <issueManagement>
      <system>GitHub issues</system>
      <url>https://github.com/kubuszok/sge-native-components/issues</url>
    </issueManagement>
  ),
  publishTo := {
    if (isSnapshot.value) Some(mavenCentralSnapshots)
    else localStaging.value
  },
  publishMavenStyle := true,
  Test / publishArtifact := false,
  pomIncludeRepository := { _ =>
    false
  },
  versionScheme := Some("early-semver")
)

val noPublishSettings =
  Seq(publish / skip := true, publishArtifact := false)

// ── Platform definitions ──────────────────────────────────────────────

// Desktop platform classifiers (must match sbt-multi-arch-release Platform.desktop)
val desktopPlatforms = Seq(
  "linux-x86_64", "linux-aarch64",
  "macos-x86_64", "macos-aarch64",
  "windows-x86_64", "windows-aarch64"
)

// Android ABI classifiers
val androidAbis = Seq(
  ("aarch64-linux-android", "android-aarch64"),
  ("armv7-linux-androideabi", "android-armv7"),
  ("x86_64-linux-android", "android-x86_64")
)

// ── Shared helpers ────────────────────────────────────────────────────

// Rust cross-compilation output root
val crossDir = settingKey[File]("Root directory containing cross-compiled native artifacts")
ThisBuild / crossDir := (ThisBuild / baseDirectory).value / "native-components" / "target" / "cross"

/** Create fat JAR mappings: native/<platform-classifier>/<file> for matching files. */
def fatJarMappings(crossRoot: File, platforms: Seq[String], fileFilter: String => Boolean): Seq[(File, String)] =
  platforms.flatMap { plat =>
    val dir = crossRoot / plat
    if (dir.exists())
      sbt.IO.listFiles(dir).filter(f => f.isFile && fileFilter(f.getName)).map(f => f -> s"native/$plat/${f.getName}").toSeq
    else Seq.empty
  }

// Shared library extensions per OS
def isSharedLib(name: String): Boolean =
  name.endsWith(".dylib") || name.endsWith(".so") || name.endsWith(".dll")

def isStaticLib(name: String): Boolean =
  name.endsWith(".a") || name.endsWith(".lib")

// ── Root project ──────────────────────────────────────────────────────

lazy val root = project
  .in(file("."))
  .enablePlugins(GitVersioning, GitBranchPrompt)
  .settings(publishSettings *)
  .settings(noPublishSettings *)
  .aggregate(
    `scala-native-sge-ops-provider`,
    `panama-sge-ops-provider`,
    `android-sge-ops-provider`,
    `scala-native-angle-provider`,
    `panama-angle-provider`
  )
  .settings(
    name := "sge-native-components-root",
    commands += Command.command("ci-release") { state =>
      val extracted = Project.extract(state)
      val tags      = extracted.get(git.gitCurrentTags)
      if (tags.nonEmpty) "publishSigned" :: "sonaRelease" :: state
      else "publishSigned" :: state
    }
  )

// ── SGE core native ops (sge_native_ops + sge_audio + glfw3) ─────────

// Scala Native: static archives (.a) for linking
lazy val `scala-native-sge-ops-provider` = project
  .in(file("providers/scala-native-sge-ops"))
  .settings(publishSettings *)
  .settings(
    name             := "scala-native-sge-ops-provider",
    autoScalaLibrary := false,
    crossPaths       := false,
    Compile / packageDoc / publishArtifact := false,
    Compile / packageSrc / publishArtifact := false,
    // Fat JAR: native/<platform>/lib*.a for all 6 desktop platforms
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val sgeOpsLibs = Set(
        "libsge_native_ops.a", "sge_native_ops.lib",
        "libsge_audio.a",
        "libglfw3.a",
        "libfreetype.a"  // built by freetype-sys: system lib on macOS, vendored on cross-compiled targets
      )
      fatJarMappings(cross, desktopPlatforms, sgeOpsLibs.contains)
    }
  )

// JVM (Panama): shared libraries (.dylib/.so/.dll) for runtime loading
lazy val `panama-sge-ops-provider` = project
  .in(file("providers/panama-sge-ops"))
  .settings(publishSettings *)
  .settings(
    name             := "panama-sge-ops-provider",
    autoScalaLibrary := false,
    crossPaths       := false,
    Compile / packageDoc / publishArtifact := false,
    Compile / packageSrc / publishArtifact := false,
    // Fat JAR: native/<platform>/*.{dylib,so,dll} for all 6 desktop platforms
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val sgeOpsLibs = Set(
        "libsge_native_ops.dylib", "libsge_native_ops.so", "sge_native_ops.dll",
        "sge_native_ops.dll.lib",
        "libsge_audio.dylib", "libsge_audio.so", "sge_audio.dll",
        "libglfw.dylib", "libglfw.so", "glfw3.dll"
      )
      fatJarMappings(cross, desktopPlatforms, sgeOpsLibs.contains)
    }
  )

// Android: JNI shared libraries (.so) for 3 ABIs
lazy val `android-sge-ops-provider` = project
  .in(file("providers/android-sge-ops"))
  .settings(publishSettings *)
  .settings(
    name             := "android-sge-ops-provider",
    autoScalaLibrary := false,
    crossPaths       := false,
    Compile / packageDoc / publishArtifact := false,
    Compile / packageSrc / publishArtifact := false,
    // Fat JAR: native/<android-classifier>/*.so for 3 Android ABIs
    Compile / packageBin / mappings ++= {
      val base = (ThisBuild / baseDirectory).value / "native-components" / "target"
      androidAbis.flatMap { case (rustTarget, classifier) =>
        val dir = base / rustTarget / "release"
        if (dir.exists()) {
          val sgeOpsLibs = Set("libsge_native_ops.so", "libsge_audio.so")
          sbt.IO.listFiles(dir).filter(f => f.isFile && sgeOpsLibs.contains(f.getName))
            .map(f => f -> s"native/$classifier/${f.getName}").toSeq
        } else Seq.empty
      }
    }
  )

// ── ANGLE (EGL + GLESv2) ──────────────────────────────────────────────

// Scala Native: ANGLE static/shared libs for linking
lazy val `scala-native-angle-provider` = project
  .in(file("providers/scala-native-angle"))
  .settings(publishSettings *)
  .settings(
    name             := "scala-native-angle-provider",
    autoScalaLibrary := false,
    crossPaths       := false,
    Compile / packageDoc / publishArtifact := false,
    Compile / packageSrc / publishArtifact := false,
    // ANGLE libs in cross dir (downloaded by angle scripts)
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val angleLibs = Set(
        "libEGL.dylib", "libEGL.so", "libEGL.dll",
        "libGLESv2.dylib", "libGLESv2.so", "GLESv2.dll"
      )
      fatJarMappings(cross, desktopPlatforms, angleLibs.contains)
    }
  )

// JVM (Panama): same ANGLE shared libs for runtime loading
lazy val `panama-angle-provider` = project
  .in(file("providers/panama-angle"))
  .settings(publishSettings *)
  .settings(
    name             := "panama-angle-provider",
    autoScalaLibrary := false,
    crossPaths       := false,
    Compile / packageDoc / publishArtifact := false,
    Compile / packageSrc / publishArtifact := false,
    // Same ANGLE shared libs as scala-native, used at JVM runtime
    Compile / packageBin / mappings ++= {
      val cross = crossDir.value
      val angleLibs = Set(
        "libEGL.dylib", "libEGL.so", "libEGL.dll",
        "libGLESv2.dylib", "libGLESv2.so", "GLESv2.dll"
      )
      fatJarMappings(cross, desktopPlatforms, angleLibs.contains)
    }
  )
