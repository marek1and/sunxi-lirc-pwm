KDIR ?= /home/marek/Embedded/Cubietruck/kernel/source
KBUILD_EXTRA_SYMBOLS=$(KDIR)/Module.symvers
CROSS_COMPILE = /opt/toolchains/linaro/4.8/bin/arm-linux-gnueabihf-
ARCH = arm

default:
	$(MAKE) -C $(KDIR) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) M=$$PWD/build modules
	mv src/*.{ko,o,mod.c} build/
	mv src/.*.cmd build/

clean:
	$(MAKE) -C $(KDIR) CROSS_COMPILE=$(CROSS_COMPILE) ARCH=$(ARCH) M=$$PWD/build clean


