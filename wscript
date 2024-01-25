#!/usr/bin/env python
# encoding: utf-8

import os
import time
from datetime import datetime

from waflib.Build import BuildContext

APPNAME = 'loracsp'

top = '.'
out = 'build'
cwd = os.getcwd()
now = datetime.now()
bin_date = now.strftime("%y%m%d")
if os.path.exists('.git'):
    revision = os.popen('git log --pretty=format:"%h" --abbrev=8 | head -1').read().strip()
    revision_date = os.popen('git log --pretty=format:"%ad" --date=format:"%Y%m%d" 2>/dev/null | head -1 | cut -f 1 -d\' \'').read().strip()
else:
    revision = "deaddead"
    revision_date = bin_date

bin_target = APPNAME + "." + bin_date + "." + revision + ".bin"
elf_target = APPNAME + ".elf"
map_target = APPNAME + ".map"
main_file = APPNAME + ".c"

def update_libopencm3():
    os.system('cd libopencm3 && make TARGETS=stm32/wl')

def options(ctx):
    ctx.load('gcc')

    if not os.path.exists('libopencm3/lib/libopencm3_stm32wl.a'):
        update_libopencm3()

    ctx.add_option('--arch', action='store', default='cortex-m4', help='MCU arch')
    ctx.add_option('--toolchain', action='store', default='arm-none-eabi-', help='Set toolchain prefix')
    ctx.add_option('--release', action='store_true', help='Build release image')

def configure(ctx):
    ctx.env.CC = ctx.options.toolchain + "gcc"
    ctx.env.AR = ctx.options.toolchain + "ar"
    ctx.load('gcc')

    # Locate programs
    ctx.find_program('st-flash', var='STFLASH')
    ctx.find_program(ctx.options.toolchain + 'size', var='SIZE')
    ctx.find_program(ctx.options.toolchain + 'objcopy', var='OBJCOPY')

    # Generate build arguments
    ctx.env.append_unique('CFLAGS', ['-Wall', '-DSTM32WL', '-fno-common', '-Os', '-mthumb', '-mcpu=cortex-m4', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16', '-fno-exceptions', '-ffunction-sections', '-fdata-sections', '-Wempty-body', '-Wtype-limits', '-Wmissing-parameter-type', '-Wuninitialized', '-fno-strict-aliasing', '-Wno-unused-function', '-Wno-stringop-truncation', '-fsingle-precision-constant'])

    ctx.env.append_unique('LINKFLAGS', ['--static', '-nostartfiles', '-Wl,--gc-sections', '-mthumb', '-mcpu=cortex-m4', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16'])

    ctx.env.append_unique('LDFLAGS', ['--specs=nano.specs', '-Wl,--start-group', '-lc', '-lgcc', '-lnosys', '-Wl,--end-group', '-lm'])

    # FreeRTOS
    ctx.env.append_unique('FILES', ['rtos/*.c'])

    # MAIN FILE
    ctx.env.append_unique('FILES', ['src/' + main_file + ''])

    # OTHER SOURCE FILES
    ctx.env.append_unique('FILES', ['src/printf.c'])

    ctx.env.append_unique('LINKFLAGS', ['-T' + cwd + '/flash.ld', '-Wl,-Map=' + map_target + ''])

    git_revision = int(revision, 16)
    git_date = int(revision_date, 16)

    ctx.define('CONFIG_REVISION', revision)
    ctx.define('GIT_REVISION', git_revision, False)
    ctx.define('GIT_DATE', git_date)

    if ctx.options.release == True:
        ctx.define('BUILD_RELEASE', 1)

    # Save config to header file
    ctx.write_config_header('include/conf_loracsp.h', top=True)

def build(ctx):
    ctx(export_includes=['include', '.', 'rtos/include', 'libopencm3/include', 'src'], name='include')
    # Linker script
    use=['include']
    use.extend(ctx.env.LIBCLIENTS_USE)

    ctx.program(
        source=ctx.path.ant_glob(ctx.env.FILES),
        target=elf_target,
        use=use,
        stlib=['opencm3_stm32wl'],
        stlibpath=[cwd + '/libopencm3/lib']
    )
    ctx(rule='${OBJCOPY} -O binary ${SRC} ${TGT}', source=elf_target, target=bin_target, name='objcopy', always=True)
    ctx(name="size", rule='${SIZE} ${SRC}', source=elf_target, always=True)

def flash(ctx):
    ctx(name='flash', rule='${STFLASH} write ${SRC} 0x8000000', source=bin_target, always=True)
#    ctx(name='flash', rule='${STFLASH} erase && ${STFLASH} write ${SRC} 0x8000000', source=bin_target, always=True)

class Program(BuildContext):
    cmd = 'flash'
    fun = 'flash'
