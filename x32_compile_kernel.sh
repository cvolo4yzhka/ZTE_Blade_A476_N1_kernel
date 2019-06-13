#!/bin/bash
#set architectute
export ARCH=arm
#set cross compiller path
#export CROSS_COMPILE=~/Kernel/toolchain/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-
export CROSS_COMPILE=~/Kernel/toolchain/arm-eabi-4.8/bin/arm-eabi-
#set directory for build output
export  KBUILD_OUTPUT=out32
#set defconfig
make zte_blade_a476_n1_x32_defconfig
#start compile
make zImage-dtb -j16 2>&1 | tee out32/build.log
