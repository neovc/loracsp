# Usage

program to test [ebyte E77-400M22S board](https://www.ebyte.com/product-view-news.html?id=1890)

# Install arm gcc cross compiler
  [Download gcc ver 8-2019-q3-update for linux!](https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2)

```
  download arm gcc toolchain from https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
  recommend gcc-8
  put arm-none-eabi-gcc in PATH
```

# Install stlink-utils 1.8

```
git clone https://github.com/stlink-org/stlink
cd stlink
sudo apt install cmake libusb-1.0-0-dev
mkdir build
cd build
cmake ..
make && make install
```

# Build

```
./waf clean
./waf configure build
```

# Flash

Use stlink-tools to flash E77-400M22S board.

```
~/loracsp$ ./waf flash
[1/1] Processing build/loracsp.240619.d0e9f896.bin
st-flash 1.8.0-32-g32ce4bf
file loracsp.240619.d0e9f896.bin md5 checksum: 7b09b22afc41fa7654c96406d77569, stlink checksum: 0x002e3b05
-> Flash page at 0x8000000 erased (size: 0x800)
-> Flash page at 0x8000800 erased (size: 0x800)
-> Flash page at 0x8001000 erased (size: 0x800)
-> Flash page at 0x8001800 erased (size: 0x800)
-> Flash page at 0x8002000 erased (size: 0x800)
-> Flash page at 0x8002800 erased (size: 0x800)
-> Flash page at 0x8003000 erased (size: 0x800)
-> Flash page at 0x8003800 erased (size: 0x800)
-> Flash page at 0x8004000 erased (size: 0x800)
-> Flash page at 0x8004800 erased (size: 0x800)
-> Flash page at 0x8005000 erased (size: 0x800)
-> Flash page at 0x8005800 erased (size: 0x800)
-> Flash page at 0x8006000 erased (size: 0x800)
-> Flash page at 0x8006800 erased (size: 0x800)
-> Flash page at 0x8007000 erased (size: 0x800)

  1/14  pages written
  2/14  pages written
  3/14  pages written
  4/14  pages written
  5/14  pages written
  6/14  pages written
  7/14  pages written
  8/14  pages written
  9/14  pages written
 10/14  pages written
 11/14  pages written
 12/14  pages written
 13/14  pages written
 14/14  pages written


2024-06-19T10:09:18 INFO common.c: STM32WLEx: 64 KiB SRAM, 256 KiB flash in at least 2 KiB pages.
2024-06-19T10:09:18 INFO common_flash.c: Attempting to write 30200 (0x75f8) bytes to stm32 address: 134217728 (0x8000000)
2024-06-19T10:09:19 INFO flash_loader.c: Starting Flash write for WB/G0/G4/L5/U5/H5/C0
2024-06-19T10:09:22 INFO common_flash.c: Starting verification of write complete
2024-06-19T10:09:22 INFO common_flash.c: Flash written and verified! jolly good!

'flash' finished successfully (4.056s)

````

# TEST

Use tio utility to connect to USART2 of board

```
~/loracsp$ tio /dev/ttyUSB0
[tio 10:10:57] tio v1.32
[tio 10:10:57] Press ctrl-t q to quit
[tio 10:10:57] Connected

LORA> help
avail cmds: reset uptime flash param help
LORA> uptime
rev d0e9f896 built at Jun 19 2024 10:09:04, up 72 secs (0.00 days), FreeRTOS V10.5.1
LORA> param list
  0x000 * csp_node       U8   5
  0x001   boot_cause     U8   1
  0x004 * eccc_cnt       U32  0
  0x008 * eccd_cnt       U32  0
  0x0FB   eof            I8   0
LORA> flash
flash [read|write] addr [len|data]
LORA> param
param [list|set|save] [name] [value]
LORA> param set csp_node 6
set csp_node -> 6
LORA> param list
  0x000 * csp_node       U8   6
  0x001   boot_cause     U8   1
  0x004 * eccc_cnt       U32  0
  0x008 * eccd_cnt       U32  0
  0x0FB   eof            I8   0
LORA> param save
save parameter -> OK
LORA> reset▒▒READ PARAMS FROM FLASH -> OK
CSP NODE -> #6
HSI16 36MHZ -> OK
INIT -> OK
LORA> param list
  0x000 * csp_node       U8   6
  0x001   boot_cause     U8   2
  0x004 * eccc_cnt       U32  0
  0x008 * eccd_cnt       U32  0
  0x0FB   eof            I8   0
LORA>
```
