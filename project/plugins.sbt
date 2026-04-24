// git
addSbtPlugin("com.github.sbt" % "sbt-git" % "2.1.0")
// publishing
addSbtPlugin("com.github.sbt" % "sbt-pgp" % "2.3.1")

ThisBuild / libraryDependencySchemes += "org.scala-lang.modules" %% "scala-xml" % VersionScheme.Always

// multiarch-core Platform model (used in build.sbt for platform definitions)
resolvers += "Maven Central Snapshots" at "https://central.sonatype.com/repository/maven-snapshots/"
libraryDependencies += "com.kubuszok" %% "multiarch-core" % "0.1.2"
