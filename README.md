# STM32H743 Project

This project is a bare-metal application for the STM32H743 microcontroller, built using CMake and the ARM GCC toolchain.

## Prerequisites

Before you begin, ensure you have the following software installed on your system:

*   **STM32CubeMX**: A graphical tool to configure STM32 microcontrollers. This project was generated using STM32CubeMX, and the `test.ioc` file can be used to regenerate the configuration. You can download it from the [ST website](https://www.st.com/en/development-tools/stm32cubemx.html).
*   **ARM GCC Toolchain**: The `arm-none-eabi-gcc` compiler is required. You can download it from the [ARM developer website](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads). Make sure to add the toolchain's `bin` directory to your system's PATH.
*   **CMake**: A cross-platform build system generator. You can download it from the [official CMake website](https://cmake.org/download/).
*   **Ninja**: A small, fast build system. It is recommended for use with CMake. You can find installation instructions on the [Ninja website](https://ninja-build.org/).
*   **STM32CubeProgrammer**: A tool from STMicroelectronics for programming STM32 microcontrollers. You can download it from the [ST website](https://www.st.com/en/development-tools/stm32cubeprog.html).
*   **Visual Studio Code**: The recommended code editor for this project, as it is pre-configured with build and flash tasks.

## Setup

1.  **Clone the repository:**
    ```bash
    git clone <repository-url>
    cd <repository-name>
    ```

2.  **Configure the project with CMake:**
    This project uses `CMakePresets.json` for easy configuration. From within Visual Studio Code, you can select the "Debug" or "Release" configure preset.

    Alternatively, you can configure from the command line:
    ```bash
    cmake --preset=Debug
    ```
    This will generate the build files in the `build/Debug` directory.

## Building the Project

Once the project is configured, you can build it.

*   **Using Visual Studio Code:**
    You can run the default build task in VS Code (usually `Ctrl+Shift+B` or `Cmd+Shift+B`), which will invoke CMake.

*   **From the command line:**
    ```bash
    cmake --build --preset=Debug
    ```
    This will compile the project and create the final binary file at `build/debug/test.bin`.

## Flashing the Firmware

The project includes VS Code tasks for flashing the compiled binary to the STM32 microcontroller.

There are two methods available:

1.  **Flashing via ST-Link (SWD):**
    *   Connect your ST-Link programmer to the SWD port of your STM32 board.
    *   In VS Code, open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`).
    *   Select "Tasks: Run Task".
    *   Choose the task **1. Flash via ST-Link (SWD)**. This will flash the `build/debug/test.bin` file to the device.
    *   There is also a task **2. Erase via ST-Link (SWD)** to erase the flash memory.

2.  **Flashing via USB Type-C (DFU Mode):**
    *   Put your STM32 board into DFU (Device Firmware Update) mode. This usually involves holding the BOOT0 pin high and resetting the board.
    *   Connect the board to your computer via the USB Type-C port.
    *   In VS Code, open the Command Palette (`Ctrl+Shift+P` or `Cmd+Shift+P`).
    *   Select "Tasks: Run Task".
    *   Choose the task **3. Flash via USB Type-C (DFU Mode)**.
    *   There is also a task **4. Erase via USB Type-C (DFU Mode)** to erase the flash memory in DFU mode.

## Pinout

The following table lists the pins used in this project, as configured in the `test.ioc` file:

| Pin                   | Label           | Function        |
| -------------------   | --------------- | --------------- |
| `PB3 (JTDO/TRACESWO)` | `S34_12_DIR`    | GPIO Output     |
| `PB4 (NJTRST)`        | `S34_85_PUL`    | GPIO Output     |
| `PB5`                 | `S34_85_DIR`    | GPIO Output     |
| `PB6`                 | `S23_PUL`       | GPIO Output     |
| `PB7`                 | `S23_DIR`       | GPIO Output     |
| `PB8`                 | `S17_12_PUL`    | GPIO Output     |
| `PB9`                 | `S17_12_DIR`    | GPIO Output     |
| `PD5`                 | `S34_12_PUL_1`  | GPIO Output     |
| `PD6`                 | `S34_12_DIR_1`  | GPIO Output     |
| `PD7`                 | `S34_12_PUL`    | GPIO Output     |
| `PE0`                 | `S17_07_PUL`    | GPIO Output     |
| `PE1`                 | `S17_07_DIR`    | GPIO Output     |
