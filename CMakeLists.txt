# ============================================================================
#  imu_deadreckoning — top-level build orchestrator
# ============================================================================
#  There is no C++ in this repo (LANGUAGES NONE — no compiler is probed).
#  CMake here is a thin, deterministic driver over two heterogeneous builds
#  that share one source of truth, the protobuf schema in msgs/:
#
#     sensorlogger   ->  Android (Java) APK, via the Gradle wrapper.
#                        Gradle's protobuf plugin regenerates the Java
#                        bindings from ../msgs itself.
#
#     julia_proto    ->  Julia protobuf bindings under
#                        SensorLoggerViewer/proto_gen/, via ProtoBuf.jl
#                        (gen_proto.jl). These are the bindings the
#                        SensorLoggerViewer decodes ImuBatch with.
#
#  Both are wired to their real inputs (the .proto files and, for Android,
#  the Java sources) so `cmake --build` re-runs only when those change.
#
#  Configure + build:
#     cmake -S . -B build -G Ninja
#     cmake --build build                 # everything enabled below
#     cmake --build build --target julia_proto
#     cmake --build build --target sensorlogger
#     cmake --build build --target install-apk   # adb install -r (opt-in)
# ============================================================================

cmake_minimum_required(VERSION 3.20)

project(imu_deadreckoning
        VERSION      0.1.0
        DESCRIPTION  "Android IMU -> UDP -> Julia 3-D viewer; strapdown dead reckoning"
        LANGUAGES    NONE)

# ---- What to build ---------------------------------------------------------
option(IMU_BUILD_JULIA_PROTO "Generate the Julia protobuf bindings (ProtoBuf.jl)" ON)
option(IMU_BUILD_ANDROID     "Build the SensorLogger Android APK (Gradle)"        ON)

set(IMU_ANDROID_VARIANT "Debug" CACHE STRING "Android build variant: Debug or Release")
set_property(CACHE IMU_ANDROID_VARIANT PROPERTY STRINGS Debug Release)

# juliaup channel selector: passed as `julia +<channel>`. Empty -> default julia.
set(IMU_JULIA_CHANNEL "1.11" CACHE STRING "juliaup channel for the viewer (empty = default julia)")

# ---- Shared paths ----------------------------------------------------------
set(MSGS_DIR       "${CMAKE_CURRENT_SOURCE_DIR}/msgs")
set(VIEWER_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/SensorLoggerViewer")
set(ANDROID_DIR    "${CMAKE_CURRENT_SOURCE_DIR}/SensorLogger")

# The protobuf schema — the single source of truth both builds consume. Glob
# with CONFIGURE_DEPENDS so adding/removing a .proto forces a reconfigure.
file(GLOB_RECURSE PROTO_FILES CONFIGURE_DEPENDS "${MSGS_DIR}/*.proto")

# ============================================================================
#  julia_proto — ProtoBuf.jl bindings for the SensorLoggerViewer
# ============================================================================
if(IMU_BUILD_JULIA_PROTO)
    find_program(JULIA_EXECUTABLE NAMES julia)
    if(NOT JULIA_EXECUTABLE)
        message(FATAL_ERROR
            "IMU_BUILD_JULIA_PROTO=ON but 'julia' was not found on PATH. "
            "Install juliaup, or set -DIMU_BUILD_JULIA_PROTO=OFF.")
    endif()

    # `julia +<channel>` only when a juliaup channel is requested.
    set(_julia_cmd "${JULIA_EXECUTABLE}")
    if(IMU_JULIA_CHANNEL)
        list(APPEND _julia_cmd "+${IMU_JULIA_CHANNEL}")
    endif()

    # A stamp decouples "did it run" from ProtoBuf.jl's exact output filenames
    # (which follow the proto package layout and may change with the schema).
    set(_julia_proto_stamp "${CMAKE_CURRENT_BINARY_DIR}/julia_proto.stamp")

    add_custom_command(
        OUTPUT  "${_julia_proto_stamp}"
        COMMAND ${_julia_cmd} --project=. gen_proto.jl
        COMMAND ${CMAKE_COMMAND} -E touch "${_julia_proto_stamp}"
        WORKING_DIRECTORY "${VIEWER_DIR}"
        DEPENDS "${VIEWER_DIR}/gen_proto.jl"
                "${VIEWER_DIR}/Project.toml"
                ${PROTO_FILES}
        BYPRODUCTS "${VIEWER_DIR}/proto_gen"
        COMMENT "Generating Julia protobuf bindings -> SensorLoggerViewer/proto_gen/"
        VERBATIM)

    add_custom_target(julia_proto ALL DEPENDS "${_julia_proto_stamp}")
endif()

# ============================================================================
#  sensorlogger — Android APK via the Gradle wrapper
# ============================================================================
if(IMU_BUILD_ANDROID)
    if(CMAKE_HOST_WIN32)
        set(GRADLEW "${ANDROID_DIR}/gradlew.bat")
    else()
        set(GRADLEW "${ANDROID_DIR}/gradlew")
    endif()

    # Map the variant to its Gradle task and its emitted APK path.
    if(IMU_ANDROID_VARIANT STREQUAL "Release")
        set(_assemble_task ":app:assembleRelease")
        set(_apk "${ANDROID_DIR}/app/build/outputs/apk/release/app-release-unsigned.apk")
    else()
        set(_assemble_task ":app:assembleDebug")
        set(_apk "${ANDROID_DIR}/app/build/outputs/apk/debug/app-debug.apk")
    endif()

    file(GLOB_RECURSE ANDROID_SOURCES CONFIGURE_DEPENDS
        "${ANDROID_DIR}/app/src/*.java"
        "${ANDROID_DIR}/app/src/main/AndroidManifest.xml"
        "${ANDROID_DIR}/app/src/main/res/*")

    # Gradle's protobuf plugin reads ../msgs directly, so the schema is an
    # input to the APK too — re-assemble when it changes.
    add_custom_command(
        OUTPUT  "${_apk}"
        COMMAND "${GRADLEW}" ${_assemble_task}
        WORKING_DIRECTORY "${ANDROID_DIR}"
        DEPENDS ${ANDROID_SOURCES}
                ${PROTO_FILES}
                "${ANDROID_DIR}/app/build.gradle.kts"
                "${ANDROID_DIR}/build.gradle.kts"
                "${ANDROID_DIR}/settings.gradle.kts"
        COMMENT "Assembling SensorLogger ${IMU_ANDROID_VARIANT} APK (Gradle)"
        VERBATIM USES_TERMINAL)

    add_custom_target(sensorlogger ALL DEPENDS "${_apk}")

    # ---- install-apk: flash the built APK to an attached device via adb -----
    # adb usually isn't on PATH — it lives in the SDK's platform-tools/. Derive
    # the SDK root from local.properties (sdk.dir=...) and search there too.
    set(_sdk_hints "")
    if(DEFINED ENV{ANDROID_HOME})
        list(APPEND _sdk_hints "$ENV{ANDROID_HOME}/platform-tools")
    endif()
    if(DEFINED ENV{ANDROID_SDK_ROOT})
        list(APPEND _sdk_hints "$ENV{ANDROID_SDK_ROOT}/platform-tools")
    endif()
    if(EXISTS "${ANDROID_DIR}/local.properties")
        file(STRINGS "${ANDROID_DIR}/local.properties" _lp REGEX "^sdk\\.dir=")
        if(_lp)
            string(REGEX REPLACE "^sdk\\.dir=" "" _sdk_dir "${_lp}")
            # local.properties escapes ':' and '=' with backslashes on Windows.
            string(REPLACE "\\" "" _sdk_dir "${_sdk_dir}")
            list(APPEND _sdk_hints "${_sdk_dir}/platform-tools")
        endif()
    endif()

    find_program(ADB_EXECUTABLE NAMES adb HINTS ${_sdk_hints})

    # Always define the target. If adb still wasn't found, fail at *build* time
    # with a clear message instead of silently not existing.
    if(ADB_EXECUTABLE)
        add_custom_target(install-apk
            COMMAND "${ADB_EXECUTABLE}" install -r "${_apk}"
            DEPENDS "${_apk}"
            COMMENT "adb install -r ${_apk}"
            VERBATIM USES_TERMINAL)
    else()
        add_custom_target(install-apk
            COMMAND ${CMAKE_COMMAND} -E false
            COMMENT "install-apk: 'adb' not found. Add <sdk>/platform-tools to PATH "
                    "or set ANDROID_HOME, then reconfigure."
            VERBATIM)
    endif()
endif()

# ---- Configure-time summary ------------------------------------------------
message(STATUS "")
message(STATUS "imu_deadreckoning targets:")
if(IMU_BUILD_JULIA_PROTO)
    set(_jchan "")
    if(IMU_JULIA_CHANNEL)
        set(_jchan " +${IMU_JULIA_CHANNEL}")
    endif()
    message(STATUS "  julia_proto   ON   (${JULIA_EXECUTABLE}${_jchan})")
else()
    message(STATUS "  julia_proto   OFF")
endif()
if(IMU_BUILD_ANDROID)
    message(STATUS "  sensorlogger  ON   (${IMU_ANDROID_VARIANT} via ${GRADLEW})")
else()
    message(STATUS "  sensorlogger  OFF")
endif()
message(STATUS "")
