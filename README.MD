# Watara Supervision LCD Grabber

A Raspberry Pi Pico-based solution for capturing and converting the LCD output from Watara Supervision handheld consoles.

## Overview

This project allows you to:
- Replace the original LCD screen with a modern IPS display
- Convert your Watara Supervision from a handheld device to a TV console
- Capture and process the raw LCD signals for display on external screens

The LCD grabber intercepts the digital signals from the Watara Supervision's video controller and converts them for use with modern displays.

## Hardware Requirements

- Raspberry Pi Pico
- Watara Supervision console
- Jumper wires for connecting to LCD pins
- Optional: IPS display for replacement
- Optional: Video output components for TV connection

## Pin Configuration

| Pico GPIO | Watara Signal      | Description                     |
|-----------|--------------------|---------------------------------|
| 16        | DATA0_PIN          | LCD data line 0                 |
| 17        | DATA1_PIN          | LCD data line 1                 |
| 18        | DATA2_PIN          | LCD data line 2                 |
| 19        | DATA3_PIN          | LCD data line 3                 |
| 20        | PIXEL_CLOCK_PIN    | Clock signal for pixel data     |
| 21        | FRAME_POLARITY_PIN | Frame sync/polarity indicator   |

## Installation

1. Clone this repository:
   ```
   git clone https://github.com/your-username/watara-supervision-lcd-grabber.git
   cd watara-supervision-lcd-grabber
   ```

2. Set up the Raspberry Pi Pico SDK according to the [official documentation](https://github.com/raspberrypi/pico-sdk).

3. Build the project:
   ```
   mkdir build
   cd build
   cmake ..
   make
   ```

4. Flash the resulting `.uf2` file to your Raspberry Pi Pico.

## How It Works

The code captures the Watara Supervision's LCD signals using the following approach:

1. GPIO pins are configured to read the LCD data and control signals
2. Core 1 handles the display/rendering functions
3. The main core captures LCD data on rising clock edges
4. Frame polarity changes indicate new frames
5. Data is stored in the frame buffer (160x160 resolution)
6. Display is handled with a 4-color palette matching the Watara's original greenscale

The system can refresh at full speed while maintaining accurate representation of the original display data.

## Display Modes

The code supports multiple display modes through the graphics library:
- Default mode that matches the original display's appearance
- Optional color palette customizations
- Flash mode for special visual effects

## Customization

You can modify the display appearance by adjusting:
- Color palette values in the `graphics_set_palette()` calls
- Buffer dimensions for different scaling options
- Timing parameters for different Watara Supervision models

## Debugging

To enable debugging, uncomment the `#define DEBUG` line to activate LED blinking on each new frame. This helps verify that frames are being captured correctly.

## Contributions

Contributions to improve the project are welcome! Please feel free to submit issues or pull requests.

## License

[Your preferred license here]

## Acknowledgments

- Thanks to the Raspberry Pi Pico community
- Watara Supervision hardware documentation resources
- [Any other acknowledgments]
