I have created a Repo from [Template](https://github.com/oleksandrmaslov/zmk-pointing-acceleration).
**Please create a template based on the original source.**
**If necessary, please feel free to create a repository from `Use this Template`.**

**Modifications by NUOVOTAKA** - Enhanced code quality, improved error handling, and better maintainability.

---

# ZMK MULTI-LEVEL POINTING ACCELERATION

This repository contains a **2-level** pointer acceleration implementation for pointing devices in ZMK, designed to accommodate users from beginners to advanced enthusiasts.

The acceleration makes fine cursor control more precise at slow speeds while allowing faster cursor movement when moving quickly. It supports customizable acceleration curves and can be configured for different input devices.

## 🎯 **Choose Your Level**

### **Level 1: Simple** (Recommended for most users)

- **3 basic settings** + **preset configurations**
- Perfect for getting started quickly
- Includes presets: Office, Gaming, High Sensitivity

### **Level 2: Standard** (Advanced features)

- **7 configurable settings**
- Speed-based acceleration with Y-axis boost
- Advanced exponential curves (1-5)
- Perfect for users who want full control

**Device Compatibility Note:** This module has been tested with trackballs using the PAW3222 mouse sensor. It should work with other pointing devices (trackpads, trackpoints, etc.). There may be risks you face due to differences in sensors and other factors.

**Before you start, you should make sure that you have a working
input device by following this: https://zmk.dev/docs/features/pointing**

## ✨ Features by Level

### **Level 1: Simple Features**

- ✅ **Preset configurations** (Office, Gaming, High Sensitivity)
- ✅ **Basic sensitivity** adjustment
- ✅ **3 curve types** (Linear, Mild, Strong)
- ✅ **Plug-and-play** setup

### **Level 2: Standard Features**

- ✅ All Level 1 features **including presets**
- ✅ **Speed-based acceleration** with configurable thresholds
- ✅ **Y-axis boost** for widescreen displays
- ✅ **Advanced exponential curves** with mathematical precision:
  - **Linear curve (1)**: `f(t) = t` - Constant acceleration rate
  - **Exponential curves (2-5)**: Natural, smooth acceleration feel
    - Mild exponential (2): `f(t) = e^(2t) - 1` - Balanced for general use
    - Moderate exponential (3): `f(t) = e^(3t) - 1` - More responsive
    - Strong exponential (4): `f(t) = e^(4t) - 1` - Aggressive acceleration
    - Aggressive exponential (5): `f(t) = e^(5t) - 1` - Maximum responsiveness
- ✅ **7 configurable parameters** for complete customization
- ✅ Compatible with any relative input device (mouse, trackball, touchpad)

## Installation & Usage

To use pointer acceleration, there are several steps necessary:

- adjust the `west.yml` to make the acceleration module available
- import the dependencies into your configuration files
- configure the acceleration parameters
- add the acceleration processor to your input chain

We'll go through these steps one by one.

### Adjust west.yml

Add the acceleration module to your `west.yml`:

```yaml
manifest:
  remotes:
    - name: zmkfirmware
      url-base: https://github.com/zmkfirmware
    - name: nuovotaka
      url-base: https://github.com/nuovotaka
  projects:
    - name: zmk-pointing-acceleration-alpha
      remote: nuovotaka
      revision: main
    - name: zmk
      remote: zmkfirmware
      revision: main
      import: app/west.yml
```

> [!WARNING]  
> Run `west update` if you're building on your local machine (not github actions).

### Import the dependencies

Add the necessary includes to your device overlay file (e.g. `yourkeyboard_left.overlay`):

```C
#include <input/processors.dtsi>
#include <behaviors/input_gestures_accel.dtsi>
```

### Configure Your Level

Choose your configuration level in your `prj.conf` file:

```ini
# Choose your level (1 or 2)
CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_SIMPLE=y      # Level 1: Simple
# CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD=y  # Level 2: Standard
```

### Configure Acceleration

Add the acceleration configuration to your device overlay. This configuration should go BEFORE your _input-listener_.

#### **Level 1: Simple Configuration**

**Option A: Use a Preset (Recommended)**

Available presets work for both Level 1 and Level 2:

- `CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE=y` - Conservative settings for office work
- `CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING=y` - Balanced settings for gaming
- `CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS=y` - High sensitivity for fast movements

```ini
# In your prj.conf, choose a preset:
CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING=y       # Gaming preset
# CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE=y     # Office preset
# CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS=y  # High sensitivity preset
```

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>;
    // Preset values are automatically applied!
};
```

**Option B: Custom Simple Settings**

```ini
# In your prj.conf:
CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_CUSTOM=y
```

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>;
    sensitivity = <1300>;     // 1.3x base sensitivity
    max-factor = <2800>;      // 2.8x maximum acceleration
    curve-type = <1>;         // 0=Linear, 1=Mild, 2=Strong
    sensor-dpi = <800>;       // 800 DPI sensor (optional, defaults to 800)
};
```

#### **Level 2: Standard Configuration**

**Option A: Use a Preset (Recommended)**

```ini
# In your prj.conf:
CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD=y
CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_GAMING=y       # Gaming preset
# CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_OFFICE=y     # Office preset
# CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_HIGH_SENS=y  # High sensitivity preset
```

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>;
    track-remainders;             // Enable precision tracking
    // Preset values are automatically applied for all Level 2 settings!
    // Including: sensitivity, max-factor, curve-type, y-boost,
    // speed-threshold, speed-max, min-factor
};
```

**Option B: Custom Standard Settings**

```ini
# In your prj.conf:
CONFIG_INPUT_PROCESSOR_ACCEL_LEVEL_STANDARD=y
CONFIG_INPUT_PROCESSOR_ACCEL_PRESET_CUSTOM=y
```

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>;
    sensitivity = <1200>;         // 1.2x base sensitivity
    max-factor = <3000>;          // 3.0x maximum acceleration
    curve-type = <1>;             // Basic curve (0=Linear, 1=Mild, 2=Strong)
    y-boost = <1300>;             // 1.3x Y-axis boost for widescreen
    speed-threshold = <600>;      // Start acceleration at 600 counts/sec
    speed-max = <3500>;           // Max acceleration at 3500 counts/sec
    min-factor = <1000>;          // 1.0x minimum (no deceleration)
    acceleration-exponent = <2>;  // Advanced exponential curve (1-5)
    sensor-dpi = <800>;           // 800 DPI sensor (optional, defaults to 800)
};
```

### Add to Input Chain

Add the acceleration processor to your input device's processor chain:

```devicetree
/ {
    tpad0: tpad0 {
        compatible = "zmk,input-listener";
        status = "okay";
        device = <&glidepoint>;
        input-processors = <
            &pointer_accel      // Acceleration processor
            &zip_xy_transform
        >;
    };
};
```

## Configuration Options

The acceleration processor provides several settings to customize how your pointing device behaves. Here's a detailed explanation of each option:

### Basic Settings

- `sensitivity`: (Default: 1000)

  - Controls the base sensitivity multiplier for all pointer movements
  - Values are in thousandths (e.g., 1200 = 1.2x speed, 800 = 0.8x speed)
  - This affects the baseline speed before any acceleration is applied
  - **Lower values (e.g., 800)**: Slower, more precise movements - good for detailed work
  - **Higher values (e.g., 1500)**: Faster movements - good for large displays or quick navigation
  - Example: `sensitivity = <1200>` makes all movements 20% faster than default

- `max-factor`: (Default: 3500)
  - Controls maximum acceleration at high speeds
  - Values are in thousandths (e.g., 3500 = 3.5x speed)
  - **Lower values (e.g., 2000)**: Less aggressive acceleration - more predictable movement
  - **Higher values (e.g., 5000)**: More aggressive acceleration - faster for long distances
  - Example: `max-factor = <3000>` means fast movements are up to 3x faster

### Speed Settings

- `speed-threshold`: (Default: 1000)

  - Speed at which acceleration starts
  - Measured in counts per second
  - Below this speed, base sensitivity is applied
  - Above this speed, acceleration begins
  - Example: `speed-threshold = <1200>` means acceleration starts at moderate speeds

- `speed-max`: (Default: 6000)
  - Speed at which maximum acceleration is reached
  - Measured in counts per second
  - At this speed and above, max-factor is applied
  - Example: `speed-max = <6000>` means you reach max acceleration at high speeds

### Acceleration Behavior

- `min-factor`: (Default: 1000) **[Level 2 Standard only]**

  - Controls how slow movements are handled
  - Values below 1000 will make slow movements even slower for precision
  - Values are in thousandths (e.g., 800 = 0.8x speed)
  - Example: `min-factor = <800>` makes slow movements 20% slower

- `acceleration-exponent`: (Default: 2) **[Level 2 Standard only]**
  - Controls the advanced mathematical curve type for acceleration
  - **Available curves:**
    - 1 = Linear `t` (constant acceleration rate)
    - 2 = Mild exponential `e^(2t) - 1` (default, balanced)
    - 3 = Moderate exponential `e^(3t) - 1` (more responsive)
    - 4 = Strong exponential `e^(4t) - 1` (aggressive)
    - 5 = Aggressive exponential `e^(5t) - 1` (maximum responsiveness)
  - Example: `acceleration-exponent = <3>` for moderate exponential acceleration

### Hardware Settings

- `sensor-dpi`: (Default: 800)
  - DPI/CPI setting of your mouse sensor
  - Used for automatic sensitivity scaling
  - Common values: 400, 800, 1200, 1600, 3200 DPI
  - Higher DPI sensors will have reduced sensitivity to maintain consistent feel
  - Example: `sensor-dpi = <1600>` for a 1600 DPI sensor

### Advanced Options

- `track-remainders`: (Default: disabled) **[Level 2 Standard only]**
  - Enables tracking of fractional movements for higher precision
  - Accumulates small movements that would otherwise be lost due to rounding
  - Improves precision by carrying over fractional parts to subsequent movements
  - Enable with `track-remainders;` in your config (boolean property)
  - Particularly useful for low DPI sensors or high acceleration factors

### Visual Examples

Here's how different configurations affect pointer movement:

```
Slow Speed │  Medium Speed  │  High Speed
───────────┼────────────────┼────────────
0.8x      →│      1.0x     →│     3.0x     (Balanced)
0.9x      →│      1.0x     →│     2.0x     (Light)
0.7x      →│      1.0x     →│     4.0x     (Heavy)
0.5x      →│      1.0x     →│     1.5x     (Precision)
```

## Share Your Settings

### App for easy configuration visualisation: https://pointing.streamlit.app/

The configurations under are just starting points - every person's perfect pointer settings are as unique as they are) I'd love to see what works best for you.

### Why Share?

- Help others find their ideal setup
- Contribute to the community knowledge
- Get feedback and suggestions
- Inspire new configuration ideas

### How to Share

- Drop your config in a GitHub issue
- Share on Discord ZMK or my DM (with a quick note about your use case)
- Comment on what worked/didn't work for you

> **Remember**: These examples were primarily tested with trackballs using the PAW3222 mouse sensor. If you're using other pointing devices (like trackpads or trackpoints), your mileage may vary - and that's why sharing your experience is so valuable

### General Use (Level 2 Standard):

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    track-remainders;          // Enable precision tracking
    sensitivity = <1200>;      // 1.2x base sensitivity
    max-factor = <3000>;       // Good acceleration for large movements
    curve-type = <1>;          // Basic mild curve
    y-boost = <1200>;          // 1.2x Y-axis boost
    speed-threshold = <1200>;  // Balanced acceleration point
    speed-max = <6000>;        // Max acceleration at 6000 counts/sec
    min-factor = <800>;        // 0.8x minimum for precision
    acceleration-exponent = <2>; // Mild exponential curve
    sensor-dpi = <800>;        // 800 DPI sensor
};
```

### Light Acceleration (Level 2 Standard):

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    sensitivity = <1100>;      // 1.1x base sensitivity
    max-factor = <2000>;       // 2.0x maximum
    curve-type = <0>;          // Linear basic curve
    y-boost = <1100>;          // 1.1x Y-axis boost
    speed-threshold = <1500>;  // Start accelerating later
    speed-max = <5000>;        // 5000 counts/sec for max accel
    acceleration-exponent = <1>; // Linear acceleration
    sensor-dpi = <800>;        // 800 DPI sensor
};
```

### Heavy Acceleration (Level 2 Standard):

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    track-remainders;          // Enable precision tracking
    sensitivity = <1000>;      // 1.0x base sensitivity
    max-factor = <4000>;       // 4.0x maximum
    curve-type = <2>;          // Strong basic curve
    y-boost = <1000>;          // 1.0x Y-axis boost
    speed-threshold = <1000>;  // Start accelerating earlier
    speed-max = <6000>;        // 6000 counts/sec for max accel
    min-factor = <700>;        // 0.7x minimum for precision
    acceleration-exponent = <4>; // Strong exponential curve
    sensor-dpi = <800>;        // 800 DPI sensor
};
```

### Optimized Widescreen (Level 2 Standard) - Tested Configuration:

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    track-remainders;          // Enable precision tracking
    sensitivity = <1300>;      // 1.3x base sensitivity
    max-factor = <5000>;       // 5.0x maximum acceleration
    curve-type = <2>;          // Strong curve for responsive feel
    y-boost = <2500>;          // 2.5x Y-axis boost for widescreen
    speed-threshold = <200>;   // Early acceleration start
    speed-max = <4000>;        // Lower speed for max acceleration
    min-factor = <800>;        // 0.8x minimum for precision
    acceleration-exponent = <4>; // Strong exponential curve
    sensor-dpi = <800>;        // 800 DPI sensor
};
```

### Precision Mode (Level 1 Simple):

```devicetree
&pointer_accel {
    input-type = <INPUT_EV_REL>;
    codes = <INPUT_REL_X INPUT_REL_Y>; // X and Y axis events
    sensitivity = <800>;       // 0.8x for fine control
    max-factor = <1500>;       // 1.5x maximum
    curve-type = <0>;          // Linear curve
    sensor-dpi = <800>;        // 800 DPI sensor
};
```
