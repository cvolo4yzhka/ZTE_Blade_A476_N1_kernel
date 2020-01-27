#!/bin/bash
#set architectute
export ARCH=arm
#set cross compiller path
#export CROSS_COMPILE=~/Kernel/toolchain/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-
#export CROSS_COMPILE=~/Kernel/toolchain/arm-linux-androideabi-4.9/bin/arm-eabi-
export CROSS_COMPILE=~/toolchain/arm-linux-androideabi-4.9/bin/arm-linux-androideabi-
#set directory for build output
export  KBUILD_OUTPUT=out32
#set defconfig
make zte_blade_a476_n1_x32_defconfig
#start compile
make zImage-dtb -j16 2>&1 | tee out32/build.log
