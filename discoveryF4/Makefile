all: demos

libstm_build:
	$(MAKE) -C libstm/STM32F4xx_StdPeriph_Driver/build

demos:  libstm_build
	$(MAKE) -C discovery_demo
	$(MAKE) -C serialUSB

clean:
	$(MAKE) -C discovery_demo clean
	$(MAKE) -C serialUSB clean
	$(MAKE) -C libstm/STM32F4xx_StdPeriph_Driver/build clean
