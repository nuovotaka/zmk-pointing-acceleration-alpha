# CMakeLists.txt - ZMK Input Processor Acceleration Middleware
# Universal build configuration for embedded systems
# Compatible with: ARM Cortex-M, RISC-V, x86, Xtensa, and other Zephyr-supported architectures

zephyr_library()

# Core source files (always included)
zephyr_library_sources(
  src/input_processor_accel_main.c
  src/input_processor_accel_utils.c
  src/input_processor_accel_calc_common.c
  src/input_processor_accel_calc_level1.c
  src/input_processor_accel_calc_level2.c
  src/config/accel_config.c
  src/config/accel_config_adapter.c
  src/config/accel_device_init.c
  src/validation/accel_validation.c
  src/presets/accel_presets.c
)

# Include directories
# zephyr_library_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


zephyr_include_directories(${APPLICATION_SOURCE_DIR}/include)

# Universal compiler optimizations for embedded systems (platform-agnostic)
zephyr_compile_options(
  -Os                    # Optimize for size (universal for embedded systems)
  -ffunction-sections    # Place functions in separate sections (enables dead code elimination)
  -fdata-sections        # Place data in separate sections (enables dead data elimination)
  -fno-common           # Avoid common blocks (saves RAM, improves linking)
  -fomit-frame-pointer  # Remove frame pointer (saves code size and registers)
  -finline-functions    # Inline small functions (improves performance)
  -fno-strict-aliasing  # Safe for embedded systems with type punning
  -fshort-enums         # Use smallest possible enum size (saves memory)
)

# Performance optimizations (compiler-agnostic)
zephyr_compile_options(
  -ffast-math           # Enable fast math optimizations (safe for integer-heavy code)
  -fno-math-errno       # Don't set errno for math functions (performance gain)
  -fno-trapping-math    # Assume no floating point exceptions (performance gain)
)

# Code generation optimizations
zephyr_compile_options(
  -fmerge-all-constants # Merge identical constants (saves flash)
  -fno-ident            # Don't emit compiler identification (saves space)
  -fno-asynchronous-unwind-tables # Remove unwind tables (saves flash)
)

# Universal linker optimizations for embedded systems
zephyr_link_libraries(
  -Wl,--gc-sections              # Remove unused sections (dead code elimination)
  -Wl,--sort-common              # Sort common symbols for better packing
  -Wl,--sort-section=alignment   # Align sections for optimal memory usage
  -Wl,--build-id=none            # Remove build ID to save flash space
  -Wl,--hash-style=gnu           # Use GNU hash style (faster dynamic linking)
)

# Advanced linker optimizations (when supported)
if(CMAKE_C_COMPILER_ID STREQUAL "GNU")
  zephyr_link_libraries(
    -Wl,--relax                  # Enable linker relaxation (size optimization)
    -Wl,--compress-debug-sections=zlib # Compress debug sections
  )
endif()

# Optional: Memory usage reporting for development
if(CONFIG_INPUT_PROCESSOR_ACCEL_DEBUG_LOG)
  zephyr_link_libraries(
    -Wl,--print-memory-usage     # Show memory usage after linking
    -Wl,--cref                   # Generate cross-reference table
  )
endif()

# Optional: Link-time optimization (LTO) for maximum performance
# Note: Can increase build time significantly
if(CONFIG_LTO)
  zephyr_compile_options(-flto)
  zephyr_link_libraries(-flto)
endif()
