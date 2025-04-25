## Instructions

These assume a vanilla ARM64 Mac running Sonoma, with Xcode or at least its command line tools, *without* Homebrew.

### Obtain prerequisites

- Build and install xz from source [https://tukaani.org/xz/#_stable] (macOS doesn't come with it, but macOS tar WILL use it to extract .tar.xz if present)
- Build and install cmake from source
- Download a prebuilt gcc-arm-none-eabi from arm (also satisfies newlib dep) and extract it in ~/Downloads/

    curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/14.2.rel1/binrel/arm-gnu-toolchain-14.2.rel1-aarch64-arm-none-eabi.tar.xz
    
    tar Jxf arm-gnu-toolchain-14.2.rel1-aarch64-arm-none-eabi.tar.xz

- git clone pico-sdk, also in ~/Downloads/

### Build this code

- `mkdir -p build && cd build && cmake .. -DPICO_BOARD=pico2 && cd ..`
- `make -C build -j4`

### Upload and run this code

- Hold down BOOTSEL while plugging into USB
- `cp build/picoadc.uf2 /Volumes/RP2350`
