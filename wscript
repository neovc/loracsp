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

if os.path.exists('libopencm3/.git'):
    libopencm3_revision = os.popen('cd libopencm3 && git log --pretty=format:"%h" --abbrev=8 2> /dev/null | head -1 || echo {0}'.format("1.")).read().strip()
    libopencm3_revision_date = os.popen('cd libopencm3 && git log --pretty=format:"%ad" --date=format:"%Y%m%d" 2>/dev/null | head -1 | cut -f 1 -d\' \'').read().strip()
else:
    libopencm3_revision = "deaddead"
    libopencm3_revision_date = bin_date

# Scan modules in local lib dir
modules = ['libcsp']

def down_libopencm3():
    if not os.path.exists('libopencm3'):
        print('begin downloading libopencm3 library...\n')
        os.system('git clone git@github.com:neovc/stm32wl libopencm3')

def update_libopencm3():
    os.system('cd libopencm3 && git pull && make TARGETS=stm32/wl')

def options(ctx):
    ctx.load('gcc')

    down_libopencm3()

    ctx.recurse(modules, once=True)

    ctx.add_option('--arch', action='store', default='cortex-m4', help='MCU arch')
    ctx.add_option('--toolchain', action='store', default='arm-none-eabi-', help='Set toolchain prefix')
    ctx.add_option('--release', action='store_true', help='Build release image')
    ctx.add_option('--update', action='store_true', help='Update libopencm3 source')
    ctx.add_option('--hostname', action='store', default='uv', help='Specify CSP hostname')
    ctx.add_option('--model', action='store', default='loracsp', help='Specify board version')

def configure(ctx):
    ctx.env.CC = ctx.options.toolchain + "gcc"
    ctx.env.AR = ctx.options.toolchain + "ar"
    ctx.load('gcc')

    # Locate programs
    ctx.find_program('st-flash', var='STFLASH')
    ctx.find_program(ctx.options.toolchain + 'size', var='SIZE')
    ctx.find_program(ctx.options.toolchain + 'objcopy', var='OBJCOPY')

    # Generate build arguments
    ctx.env.append_unique('CFLAGS', ['-Wall', '-DSTM32WL', '-fno-common', '-Os', '-mthumb', '-mcpu=cortex-m4', '-fno-exceptions', '-ffunction-sections', '-fdata-sections', '-Wempty-body', '-Wtype-limits', '-Wmissing-parameter-type', '-Wuninitialized', '-fno-strict-aliasing', '-Wno-unused-function', '-Wno-stringop-truncation', '-fsingle-precision-constant'])

    ctx.env.append_unique('LINKFLAGS', ['--static', '-nostartfiles', '-Wl,--gc-sections', '-mthumb', '-mcpu=cortex-m4'])

    ctx.env.append_unique('LDFLAGS', ['--specs=nano.specs', '-Wl,--start-group', '-lc', '-lgcc', '-lnosys', '-Wl,--end-group', '-lm'])

    # FreeRTOS
    ctx.env.append_unique('FILES', ['rtos/*.c'])

    # PROJECT FILES
    ctx.env.append_unique('FILES', ['src/*.c'])

    ctx.env.append_unique('LINKFLAGS', ['-T' + cwd + '/flash.ld', '-Wl,-Map=' + map_target + ''])

    git_revision = int(revision, 16)
    git_date = int(revision_date, 16)

    libopencm3_git_revision = int(libopencm3_revision, 16)
    libopencm3_git_date = int(libopencm3_revision_date, 16)

    ctx.define('CONFIG_REVISION', revision)
    ctx.define('GIT_REVISION', git_revision, False)
    ctx.define('GIT_DATE', git_date)
    ctx.define('LIBOPENCM3_GIT_REVISION', libopencm3_git_revision, False)
    ctx.define('LIBOPENCM3_GIT_DATE', libopencm3_git_date)

    if ctx.options.release == True:
        ctx.define('BUILD_RELEASE', 1)

    # FOR LIBCSP
    ctx.env.append_unique('INCLUDES_CSP', ['include', '../rtos/include', '../src'])

    # Options for libCSP
    ctx.options.with_os = 'freertos'
    ctx.options.with_rtable = 'cidr'
    ctx.options.with_max_connections = 20
    ctx.options.with_conn_queue_length = 50
    ctx.options.with_router_queue_length = 20
    ctx.options.with_max_bind_port = 25
    ctx.options.enable_crc32 = True
    ctx.options.enable_if_i2c = False
    ctx.options.enable_if_kiss = True
    ctx.options.enable_if_can = False
    ctx.options.enable_if_stdcan = False
    ctx.options.enable_if_extcan = False
    ctx.options.enable_rdp = True
    ctx.options.enable_qos = True
    ctx.options.enable_hmac = True
    ctx.options.enable_dedup = True

    # Save config to header file
    ctx.write_config_header('include/conf_loracsp.h', top=True)
    ctx.recurse(modules)

    if ctx.options.update == True or not os.path.exists('libopencm3/lib/libopencm3_stm32wl.a'):
        update_libopencm3()

def build(ctx):
    ctx(export_includes=['include', '.', 'rtos/include', 'libopencm3/include', 'src'], name='include')
    ctx.recurse(modules)
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
