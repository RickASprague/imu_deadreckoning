import com.google.protobuf.gradle.id
import com.google.protobuf.gradle.proto

plugins {
    id("com.android.application")
    id("com.google.protobuf") version "0.9.4"
}

android {
    namespace = "com.rastar.sensorlogger"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.rastar.sensorlogger"
        minSdk = 24
        targetSdk = 35
        versionCode = 1
        versionName = "1.0"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    // Generate Java straight from the shared, single-source-of-truth protos in
    // ../msgs — no hand-written Java copy of the schema.
    sourceSets {
        getByName("main") {
            proto {
                srcDir("${rootProject.projectDir}/../msgs")
            }
        }
    }
}

// protobuf-gradle-plugin: run protoc and emit the *lite* Java runtime
// (protobuf-javalite) — smaller APK, no reflection, right choice for Android.
protobuf {
    protoc {
        artifact = "com.google.protobuf:protoc:4.28.2"
    }
    generateProtoTasks {
        all().forEach { task ->
            task.builtins {
                id("java") {
                    option("lite")
                }
            }
        }
    }
}

dependencies {
    implementation("androidx.appcompat:appcompat:1.7.0")

    // protobuf lite runtime — must match the protoc version above.
    // (Transport is raw UDP broadcast via java.net — no ZeroMQ needed.)
    implementation("com.google.protobuf:protobuf-javalite:4.28.2")

    // appcompat drags in kotlin-stdlib 1.8.22 while older transitive deps pull
    // kotlin-stdlib-jdk7/jdk8 1.6.21, whose classes were folded into the main
    // stdlib in 1.8 -> duplicate-class build failure. Align the jdk7/jdk8
    // artifacts to 1.8.22, where they are empty stubs delegating to stdlib.
    constraints {
        implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk7:1.8.22")
        implementation("org.jetbrains.kotlin:kotlin-stdlib-jdk8:1.8.22")
    }
}
