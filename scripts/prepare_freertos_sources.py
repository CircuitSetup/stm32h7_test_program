from pathlib import Path

Import("env")


framework_dir = Path(env.PioPlatform().get_package_dir("framework-stm32cubeh7"))
freertos_root = framework_dir / "Middlewares" / "Third_Party" / "FreeRTOS" / "Source"

include_paths = [
    freertos_root / "include",
    freertos_root / "CMSIS_RTOS_V2",
    freertos_root / "portable" / "GCC" / "ARM_CM7" / "r0p1",
]

env.Append(
    CPPPATH=[str(path) for path in include_paths],
    CPPDEFINES=[
        ('CMSIS_device_header', '\\"stm32h743xx.h\\"'),
        "USE_FreeRTOS_HEAP_4",
        ("USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION", 1),
    ],
    ASFLAGS=[
        "-mfpu=fpv5-d16",
        "-mfloat-abi=hard",
    ],
    LINKFLAGS=[
        "-mfpu=fpv5-d16",
        "-mfloat-abi=hard",
    ],
)

build_dir = Path(env["PROJECT_BUILD_DIR"]) / env["PIOENV"] / "freertos_kernel"

env.BuildSources(
    str(build_dir / "kernel"),
    str(freertos_root),
    src_filter=[
        "+<tasks.c>",
        "+<queue.c>",
        "+<list.c>",
        "+<timers.c>",
        "+<event_groups.c>",
        "+<stream_buffer.c>",
    ],
)

env.BuildSources(
    str(build_dir / "portable"),
    str(freertos_root / "portable"),
    src_filter=[
        "+<GCC/ARM_CM7/r0p1/port.c>",
        "+<MemMang/heap_4.c>",
    ],
)

env.BuildSources(
    str(build_dir / "cmsis_os2"),
    str(freertos_root / "CMSIS_RTOS_V2"),
    src_filter=["+<cmsis_os2.c>"],
)
