// kubuszok plugin (bundles: sbt-git, sbt-pgp, and more)
// TODO: switch to sbt-kubuszok 0.2.5 after its release (resolved from the Maven Central Snapshots resolver below)
addSbtPlugin("com.kubuszok" % "sbt-kubuszok" % "0.2.4-4-g52226ed-SNAPSHOT")

ThisBuild / libraryDependencySchemes += "org.scala-lang.modules" %% "scala-xml" % VersionScheme.Always

// multiarch-core Platform model (used in build.sbt for platform definitions)
resolvers += "Maven Central Snapshots" at "https://central.sonatype.com/repository/maven-snapshots/"
libraryDependencies += "com.kubuszok" %% "multiarch-core" % "0.3.0"
