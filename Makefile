# This file was automagically generated by mbed.org. For more information, 
# see http://mbed.org/handbook/Exporting-to-GCC-ARM-Embedded

###############################################################################
# Boiler-plate

# cross-platform directory manipulation
ifeq ($(shell echo $$OS),$$OS)
    MAKEDIR = if not exist "$(1)" mkdir "$(1)"
    RM = rmdir /S /Q "$(1)"
else
    MAKEDIR = '$(SHELL)' -c "mkdir -p \"$(1)\""
    RM = '$(SHELL)' -c "rm -rf \"$(1)\""
endif

OBJDIR := BUILD
# Move to the build directory
ifeq (,$(filter $(OBJDIR),$(notdir $(CURDIR))))
.SUFFIXES:
mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKETARGET = '$(MAKE)' --no-print-directory -C $(OBJDIR) -f '$(mkfile_path)' \
		'SRCDIR=$(CURDIR)' $(MAKECMDGOALS)
.PHONY: $(OBJDIR) clean
all:
	+@$(call MAKEDIR,$(OBJDIR))
	+@$(MAKETARGET)
$(OBJDIR): all
Makefile : ;
% :: $(OBJDIR) ; :
clean :
	$(call RM,$(OBJDIR))

else

# trick rules into thinking we are in the root, when we are in the bulid dir
VPATH = ..

# Boiler-plate
###############################################################################
# Project settings

PROJECT := FTHR_Bluetooth


# Project settings
###############################################################################
# Objects and Paths

OBJECTS += USBDevice/USBAudio/USBAudio.o
OBJECTS += USBDevice/USBDevice/USBDevice.o
OBJECTS += USBDevice/USBDevice/USBHAL_EFM32.o
OBJECTS += USBDevice/USBDevice/USBHAL_KL25Z.o
OBJECTS += USBDevice/USBDevice/USBHAL_LPC11U.o
OBJECTS += USBDevice/USBDevice/USBHAL_LPC17.o
OBJECTS += USBDevice/USBDevice/USBHAL_LPC40.o
OBJECTS += USBDevice/USBDevice/USBHAL_Maxim.o
OBJECTS += USBDevice/USBDevice/USBHAL_RZ_A1H.o
OBJECTS += USBDevice/USBDevice/USBHAL_STM32F4.o
OBJECTS += USBDevice/USBHID/USBHID.o
OBJECTS += USBDevice/USBHID/USBKeyboard.o
OBJECTS += USBDevice/USBHID/USBMouse.o
OBJECTS += USBDevice/USBHID/USBMouseKeyboard.o
OBJECTS += USBDevice/USBMIDI/USBMIDI.o
OBJECTS += USBDevice/USBMSD/USBMSD.o
OBJECTS += USBDevice/USBSerial/USBCDC.o
OBJECTS += USBDevice/USBSerial/USBSerial.o
OBJECTS += main.o
OBJECTS += max32630fthr/MAX14690/MAX14690.o
OBJECTS += max32630fthr/max32630fthr.o
OBJECTS += mbed-os/drivers/AnalogIn.o
OBJECTS += mbed-os/drivers/BusIn.o
OBJECTS += mbed-os/drivers/BusInOut.o
OBJECTS += mbed-os/drivers/BusOut.o
OBJECTS += mbed-os/drivers/CAN.o
OBJECTS += mbed-os/drivers/Ethernet.o
OBJECTS += mbed-os/drivers/FlashIAP.o
OBJECTS += mbed-os/drivers/I2C.o
OBJECTS += mbed-os/drivers/I2CSlave.o
OBJECTS += mbed-os/drivers/InterruptIn.o
OBJECTS += mbed-os/drivers/InterruptManager.o
OBJECTS += mbed-os/drivers/RawSerial.o
OBJECTS += mbed-os/drivers/SPI.o
OBJECTS += mbed-os/drivers/SPISlave.o
OBJECTS += mbed-os/drivers/Serial.o
OBJECTS += mbed-os/drivers/SerialBase.o
OBJECTS += mbed-os/drivers/Ticker.o
OBJECTS += mbed-os/drivers/Timeout.o
OBJECTS += mbed-os/drivers/Timer.o
OBJECTS += mbed-os/drivers/TimerEvent.o
OBJECTS += mbed-os/drivers/UARTSerial.o
OBJECTS += mbed-os/events/EventQueue.o
OBJECTS += mbed-os/events/equeue/equeue.o
OBJECTS += mbed-os/events/equeue/equeue_mbed.o
OBJECTS += mbed-os/events/equeue/equeue_posix.o
OBJECTS += mbed-os/events/mbed_shared_queues.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/BLE.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/BLEInstanceBase.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/DiscoveredCharacteristic.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/GapScanningParams.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/services/DFUService.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/services/UARTService.o
OBJECTS += mbed-os/features/FEATURE_BLE/source/services/URIBeaconConfigService.o
OBJECTS += mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/MaximBLE.o
OBJECTS += mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/MaximGap.o
OBJECTS += mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/MaximGattServer.o
OBJECTS += mbed-os/features/filesystem/Dir.o
OBJECTS += mbed-os/features/filesystem/File.o
OBJECTS += mbed-os/features/filesystem/FileSystem.o
OBJECTS += mbed-os/features/filesystem/bd/ChainingBlockDevice.o
OBJECTS += mbed-os/features/filesystem/bd/HeapBlockDevice.o
OBJECTS += mbed-os/features/filesystem/bd/MBRBlockDevice.o
OBJECTS += mbed-os/features/filesystem/bd/ProfilingBlockDevice.o
OBJECTS += mbed-os/features/filesystem/bd/SlicingBlockDevice.o
OBJECTS += mbed-os/features/filesystem/fat/ChaN/ccsbcs.o
OBJECTS += mbed-os/features/filesystem/fat/ChaN/ff.o
OBJECTS += mbed-os/features/filesystem/fat/FATFileSystem.o
OBJECTS += mbed-os/features/frameworks/greentea-client/source/greentea_metrics.o
OBJECTS += mbed-os/features/frameworks/greentea-client/source/greentea_serial.o
OBJECTS += mbed-os/features/frameworks/greentea-client/source/greentea_test_env.o
OBJECTS += mbed-os/features/frameworks/unity/source/unity.o
OBJECTS += mbed-os/features/frameworks/utest/mbed-utest-shim.o
OBJECTS += mbed-os/features/frameworks/utest/source/unity_handler.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_case.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_default_handlers.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_greentea_handlers.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_harness.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_shim.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_stack_trace.o
OBJECTS += mbed-os/features/frameworks/utest/source/utest_types.o
OBJECTS += mbed-os/features/mbedtls/platform/src/mbed_trng.o
OBJECTS += mbed-os/features/mbedtls/src/aes.o
OBJECTS += mbed-os/features/mbedtls/src/aesni.o
OBJECTS += mbed-os/features/mbedtls/src/arc4.o
OBJECTS += mbed-os/features/mbedtls/src/asn1parse.o
OBJECTS += mbed-os/features/mbedtls/src/asn1write.o
OBJECTS += mbed-os/features/mbedtls/src/base64.o
OBJECTS += mbed-os/features/mbedtls/src/bignum.o
OBJECTS += mbed-os/features/mbedtls/src/blowfish.o
OBJECTS += mbed-os/features/mbedtls/src/camellia.o
OBJECTS += mbed-os/features/mbedtls/src/ccm.o
OBJECTS += mbed-os/features/mbedtls/src/certs.o
OBJECTS += mbed-os/features/mbedtls/src/cipher.o
OBJECTS += mbed-os/features/mbedtls/src/cipher_wrap.o
OBJECTS += mbed-os/features/mbedtls/src/cmac.o
OBJECTS += mbed-os/features/mbedtls/src/ctr_drbg.o
OBJECTS += mbed-os/features/mbedtls/src/debug.o
OBJECTS += mbed-os/features/mbedtls/src/des.o
OBJECTS += mbed-os/features/mbedtls/src/dhm.o
OBJECTS += mbed-os/features/mbedtls/src/ecdh.o
OBJECTS += mbed-os/features/mbedtls/src/ecdsa.o
OBJECTS += mbed-os/features/mbedtls/src/ecjpake.o
OBJECTS += mbed-os/features/mbedtls/src/ecp.o
OBJECTS += mbed-os/features/mbedtls/src/ecp_curves.o
OBJECTS += mbed-os/features/mbedtls/src/entropy.o
OBJECTS += mbed-os/features/mbedtls/src/entropy_poll.o
OBJECTS += mbed-os/features/mbedtls/src/error.o
OBJECTS += mbed-os/features/mbedtls/src/gcm.o
OBJECTS += mbed-os/features/mbedtls/src/havege.o
OBJECTS += mbed-os/features/mbedtls/src/hmac_drbg.o
OBJECTS += mbed-os/features/mbedtls/src/md.o
OBJECTS += mbed-os/features/mbedtls/src/md2.o
OBJECTS += mbed-os/features/mbedtls/src/md4.o
OBJECTS += mbed-os/features/mbedtls/src/md5.o
OBJECTS += mbed-os/features/mbedtls/src/md_wrap.o
OBJECTS += mbed-os/features/mbedtls/src/memory_buffer_alloc.o
OBJECTS += mbed-os/features/mbedtls/src/net_sockets.o
OBJECTS += mbed-os/features/mbedtls/src/oid.o
OBJECTS += mbed-os/features/mbedtls/src/padlock.o
OBJECTS += mbed-os/features/mbedtls/src/pem.o
OBJECTS += mbed-os/features/mbedtls/src/pk.o
OBJECTS += mbed-os/features/mbedtls/src/pk_wrap.o
OBJECTS += mbed-os/features/mbedtls/src/pkcs11.o
OBJECTS += mbed-os/features/mbedtls/src/pkcs12.o
OBJECTS += mbed-os/features/mbedtls/src/pkcs5.o
OBJECTS += mbed-os/features/mbedtls/src/pkparse.o
OBJECTS += mbed-os/features/mbedtls/src/pkwrite.o
OBJECTS += mbed-os/features/mbedtls/src/platform.o
OBJECTS += mbed-os/features/mbedtls/src/ripemd160.o
OBJECTS += mbed-os/features/mbedtls/src/rsa.o
OBJECTS += mbed-os/features/mbedtls/src/sha1.o
OBJECTS += mbed-os/features/mbedtls/src/sha256.o
OBJECTS += mbed-os/features/mbedtls/src/sha512.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_cache.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_ciphersuites.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_cli.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_cookie.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_srv.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_ticket.o
OBJECTS += mbed-os/features/mbedtls/src/ssl_tls.o
OBJECTS += mbed-os/features/mbedtls/src/threading.o
OBJECTS += mbed-os/features/mbedtls/src/timing.o
OBJECTS += mbed-os/features/mbedtls/src/version.o
OBJECTS += mbed-os/features/mbedtls/src/version_features.o
OBJECTS += mbed-os/features/mbedtls/src/x509.o
OBJECTS += mbed-os/features/mbedtls/src/x509_create.o
OBJECTS += mbed-os/features/mbedtls/src/x509_crl.o
OBJECTS += mbed-os/features/mbedtls/src/x509_crt.o
OBJECTS += mbed-os/features/mbedtls/src/x509_csr.o
OBJECTS += mbed-os/features/mbedtls/src/x509write_crt.o
OBJECTS += mbed-os/features/mbedtls/src/x509write_csr.o
OBJECTS += mbed-os/features/mbedtls/src/xtea.o
OBJECTS += mbed-os/features/netsocket/NetworkInterface.o
OBJECTS += mbed-os/features/netsocket/NetworkStack.o
OBJECTS += mbed-os/features/netsocket/Socket.o
OBJECTS += mbed-os/features/netsocket/SocketAddress.o
OBJECTS += mbed-os/features/netsocket/TCPServer.o
OBJECTS += mbed-os/features/netsocket/TCPSocket.o
OBJECTS += mbed-os/features/netsocket/UDPSocket.o
OBJECTS += mbed-os/features/netsocket/WiFiAccessPoint.o
OBJECTS += mbed-os/features/netsocket/cellular/generic_modem_driver/OnboardCellularInterface.o
OBJECTS += mbed-os/features/netsocket/cellular/generic_modem_driver/PPPCellularInterface.o
OBJECTS += mbed-os/features/netsocket/cellular/generic_modem_driver/UARTCellularInterface.o
OBJECTS += mbed-os/features/netsocket/nsapi_dns.o
OBJECTS += mbed-os/hal/mbed_flash_api.o
OBJECTS += mbed-os/hal/mbed_gpio.o
OBJECTS += mbed-os/hal/mbed_lp_ticker_api.o
OBJECTS += mbed-os/hal/mbed_pinmap_common.o
OBJECTS += mbed-os/hal/mbed_sleep_manager.o
OBJECTS += mbed-os/hal/mbed_ticker_api.o
OBJECTS += mbed-os/hal/mbed_us_ticker_api.o
OBJECTS += mbed-os/platform/ATCmdParser.o
OBJECTS += mbed-os/platform/CallChain.o
OBJECTS += mbed-os/platform/FileBase.o
OBJECTS += mbed-os/platform/FileHandle.o
OBJECTS += mbed-os/platform/FilePath.o
OBJECTS += mbed-os/platform/FileSystemHandle.o
OBJECTS += mbed-os/platform/LocalFileSystem.o
OBJECTS += mbed-os/platform/Stream.o
OBJECTS += mbed-os/platform/mbed_alloc_wrappers.o
OBJECTS += mbed-os/platform/mbed_application.o
OBJECTS += mbed-os/platform/mbed_assert.o
OBJECTS += mbed-os/platform/mbed_board.o
OBJECTS += mbed-os/platform/mbed_critical.o
OBJECTS += mbed-os/platform/mbed_error.o
OBJECTS += mbed-os/platform/mbed_interface.o
OBJECTS += mbed-os/platform/mbed_mem_trace.o
OBJECTS += mbed-os/platform/mbed_mktime.o
OBJECTS += mbed-os/platform/mbed_poll.o
OBJECTS += mbed-os/platform/mbed_retarget.o
OBJECTS += mbed-os/platform/mbed_rtc_time.o
OBJECTS += mbed-os/platform/mbed_sdk_boot.o
OBJECTS += mbed-os/platform/mbed_semihost_api.o
OBJECTS += mbed-os/platform/mbed_stats.o
OBJECTS += mbed-os/platform/mbed_wait_api_no_rtos.o
OBJECTS += mbed-os/platform/mbed_wait_api_rtos.o
OBJECTS += mbed-os/rtos/EventFlags.o
OBJECTS += mbed-os/rtos/Mutex.o
OBJECTS += mbed-os/rtos/RtosTimer.o
OBJECTS += mbed-os/rtos/Semaphore.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/mbed_boot.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/mbed_rtx_handlers.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/mbed_rtx_idle.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx4/cmsis_os1.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/RTX_Config.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/TARGET_RTOS_M4_M7/TOOLCHAIN_GCC/irq_cm4f.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rt_OsEventObserver.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_delay.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_evflags.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_evr.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_kernel.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_lib.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_memory.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_mempool.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_msgqueue.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_mutex.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_semaphore.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_system.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_thread.o
OBJECTS += mbed-os/rtos/TARGET_CORTEX/rtx5/rtx_timer.o
OBJECTS += mbed-os/rtos/Thread.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/PeripheralPins.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/TARGET_MAX32630FTHR/low_level_init.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/analogin_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/device/TOOLCHAIN_GCC_ARM/startup_max3263x.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/device/device_nvic.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/device/system_max3263x.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/gpio_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/gpio_irq_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/i2c_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/adc.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/aes.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/clkman.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/crc.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/flc.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/gpio.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/i2cm.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/i2cs.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/icc.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/ioman.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/lp.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/maa.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/mxc_assert.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/mxc_sys.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/owm.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/pmu.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/prng.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/pt.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/rtc.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/spim.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/spix.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/tmr.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/tmr_utils.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/uart.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/wdt.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc/wdt2.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/pinmap.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/port_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/pwmout_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/rtc_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/serial_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/sleep.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/spi_api.o
OBJECTS += mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/us_ticker.o


INCLUDE_PATHS += -I../
INCLUDE_PATHS += -I../.
INCLUDE_PATHS += -I..//usr/src/mbed-sdk
INCLUDE_PATHS += -I../USBDevice
INCLUDE_PATHS += -I../USBDevice/USBAudio
INCLUDE_PATHS += -I../USBDevice/USBDevice
INCLUDE_PATHS += -I../USBDevice/USBHID
INCLUDE_PATHS += -I../USBDevice/USBMIDI
INCLUDE_PATHS += -I../USBDevice/USBMSD
INCLUDE_PATHS += -I../USBDevice/USBSerial
INCLUDE_PATHS += -I../max32630fthr
INCLUDE_PATHS += -I../max32630fthr/MAX14690
INCLUDE_PATHS += -I../mbed-os
INCLUDE_PATHS += -I../mbed-os/cmsis
INCLUDE_PATHS += -I../mbed-os/cmsis/TARGET_CORTEX_M
INCLUDE_PATHS += -I../mbed-os/cmsis/TARGET_CORTEX_M/TOOLCHAIN_GCC
INCLUDE_PATHS += -I../mbed-os/drivers
INCLUDE_PATHS += -I../mbed-os/events
INCLUDE_PATHS += -I../mbed-os/events/equeue
INCLUDE_PATHS += -I../mbed-os/features
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/ble
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/ble/services
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/hci
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/stack
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/stack/cfg
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/stack/include
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/util
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/wsf
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/wsf/generic
INCLUDE_PATHS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/wsf/include
INCLUDE_PATHS += -I../mbed-os/features/filesystem
INCLUDE_PATHS += -I../mbed-os/features/filesystem/bd
INCLUDE_PATHS += -I../mbed-os/features/filesystem/fat
INCLUDE_PATHS += -I../mbed-os/features/filesystem/fat/ChaN
INCLUDE_PATHS += -I../mbed-os/features/frameworks
INCLUDE_PATHS += -I../mbed-os/features/frameworks/greentea-client
INCLUDE_PATHS += -I../mbed-os/features/frameworks/greentea-client/greentea-client
INCLUDE_PATHS += -I../mbed-os/features/frameworks/unity
INCLUDE_PATHS += -I../mbed-os/features/frameworks/unity/unity
INCLUDE_PATHS += -I../mbed-os/features/frameworks/utest
INCLUDE_PATHS += -I../mbed-os/features/frameworks/utest/utest
INCLUDE_PATHS += -I../mbed-os/features/mbedtls
INCLUDE_PATHS += -I../mbed-os/features/mbedtls/inc
INCLUDE_PATHS += -I../mbed-os/features/mbedtls/inc/mbedtls
INCLUDE_PATHS += -I../mbed-os/features/mbedtls/platform
INCLUDE_PATHS += -I../mbed-os/features/mbedtls/platform/inc
INCLUDE_PATHS += -I../mbed-os/features/netsocket
INCLUDE_PATHS += -I../mbed-os/features/netsocket/cellular
INCLUDE_PATHS += -I../mbed-os/features/netsocket/cellular/generic_modem_driver
INCLUDE_PATHS += -I../mbed-os/features/netsocket/cellular/utils
INCLUDE_PATHS += -I../mbed-os/hal
INCLUDE_PATHS += -I../mbed-os/hal/storage_abstraction
INCLUDE_PATHS += -I../mbed-os/platform
INCLUDE_PATHS += -I../mbed-os/rtos
INCLUDE_PATHS += -I../mbed-os/rtos/TARGET_CORTEX
INCLUDE_PATHS += -I../mbed-os/rtos/TARGET_CORTEX/rtx4
INCLUDE_PATHS += -I../mbed-os/rtos/TARGET_CORTEX/rtx5
INCLUDE_PATHS += -I../mbed-os/targets/TARGET_Maxim
INCLUDE_PATHS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630
INCLUDE_PATHS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/TARGET_MAX32630FTHR
INCLUDE_PATHS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/device
INCLUDE_PATHS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc

LIBRARY_PATHS := -L../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/TOOLCHAIN_GCC_ARM 
LIBRARIES := -lexactLE 
LINKER_SCRIPT ?= ../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/device/TOOLCHAIN_GCC_ARM/max3263x.ld

# Objects and Paths
###############################################################################
# Tools and Flags

AS      = arm-none-eabi-gcc
CC      = arm-none-eabi-gcc
CPP     = arm-none-eabi-g++
LD      = arm-none-eabi-gcc
ELF2BIN = arm-none-eabi-objcopy
PREPROC = arm-none-eabi-cpp -E -P -Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp


C_FLAGS += -std=gnu11
C_FLAGS += -include
C_FLAGS += mbed_config.h
C_FLAGS += -DMBED_BUILD_TIMESTAMP=1580327371.0565317
C_FLAGS += -DDEVICE_SLEEP=1
C_FLAGS += -DDEVICE_PORTIN=1
C_FLAGS += -DOPEN_DRAIN_LEDS
C_FLAGS += -DDEVICE_SPI=1
C_FLAGS += -DTARGET_M4
C_FLAGS += -D__MBED_CMSIS_RTOS_CM
C_FLAGS += -DDEVICE_INTERRUPTIN=1
C_FLAGS += -DTARGET_Maxim
C_FLAGS += -D__CMSIS_RTOS
C_FLAGS += -DARM_MATH_CM4
C_FLAGS += -DTARGET_LIKE_MBED
C_FLAGS += -DDEVICE_ANALOGIN=1
C_FLAGS += -D__FPU_PRESENT=1
C_FLAGS += -DDEVICE_I2C=1
C_FLAGS += -DDEVICE_SERIAL_FC=1
C_FLAGS += -DDEVICE_RTC=1
C_FLAGS += -DTARGET_NAME=MAX32630FTHR
C_FLAGS += -DTOOLCHAIN_GCC
C_FLAGS += -DDEVICE_SERIAL=1
C_FLAGS += -DDEVICE_PORTINOUT=1
C_FLAGS += -DFEATURE_BLE=1
C_FLAGS += -DTARGET_RELEASE
C_FLAGS += -DTARGET_REV=0x4132
C_FLAGS += -D__CORTEX_M4
C_FLAGS += -DDEVICE_STDIO_MESSAGES=1
C_FLAGS += -DTARGET_LIKE_CORTEX_M4
C_FLAGS += -D__MBED__=1
C_FLAGS += -DTOOLCHAIN_GCC_ARM
C_FLAGS += -DTARGET_MAX32630
C_FLAGS += -DDEVICE_PWMOUT=1
C_FLAGS += -DTARGET_CORTEX
C_FLAGS += -DTARGET_RTOS_M4_M7
C_FLAGS += -DDEVICE_LOWPOWERTIMER=1
C_FLAGS += -D__SYSTEM_HFX=96000000
C_FLAGS += -DDEVICE_PORTOUT=1
C_FLAGS += -DBLE_HCI_UART
C_FLAGS += -DTARGET_MAX32630FTHR
C_FLAGS += -DTARGET=MAX32630
C_FLAGS += -DTARGET_CORTEX_M
C_FLAGS += -include
C_FLAGS += mbed_config.h
C_FLAGS += -std=gnu11
C_FLAGS += -c
C_FLAGS += -Wall
C_FLAGS += -Wextra
C_FLAGS += -Wno-unused-parameter
C_FLAGS += -Wno-missing-field-initializers
C_FLAGS += -fmessage-length=0
C_FLAGS += -fno-exceptions
C_FLAGS += -ffunction-sections
C_FLAGS += -fdata-sections
C_FLAGS += -funsigned-char
C_FLAGS += -MMD
C_FLAGS += -fno-delete-null-pointer-checks
C_FLAGS += -fomit-frame-pointer
C_FLAGS += -Os
C_FLAGS += -g
C_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
C_FLAGS += -mcpu=cortex-m4
C_FLAGS += -mthumb
C_FLAGS += -mfpu=fpv4-sp-d16
C_FLAGS += -mfloat-abi=softfp

CXX_FLAGS += -std=gnu++14
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h
CXX_FLAGS += -DMBED_BUILD_TIMESTAMP=1580327371.0565317
CXX_FLAGS += -DDEVICE_SLEEP=1
CXX_FLAGS += -DDEVICE_PORTIN=1
CXX_FLAGS += -DOPEN_DRAIN_LEDS
CXX_FLAGS += -DDEVICE_SPI=1
CXX_FLAGS += -DTARGET_M4
CXX_FLAGS += -D__MBED_CMSIS_RTOS_CM
CXX_FLAGS += -DDEVICE_INTERRUPTIN=1
CXX_FLAGS += -DTARGET_Maxim
CXX_FLAGS += -D__CMSIS_RTOS
CXX_FLAGS += -DARM_MATH_CM4
CXX_FLAGS += -DTARGET_LIKE_MBED
CXX_FLAGS += -DDEVICE_ANALOGIN=1
CXX_FLAGS += -D__FPU_PRESENT=1
CXX_FLAGS += -DDEVICE_I2C=1
CXX_FLAGS += -DDEVICE_SERIAL_FC=1
CXX_FLAGS += -DDEVICE_RTC=1
CXX_FLAGS += -DTARGET_NAME=MAX32630FTHR
CXX_FLAGS += -DTOOLCHAIN_GCC
CXX_FLAGS += -DDEVICE_SERIAL=1
CXX_FLAGS += -DDEVICE_PORTINOUT=1
CXX_FLAGS += -DFEATURE_BLE=1
CXX_FLAGS += -DTARGET_RELEASE
CXX_FLAGS += -DTARGET_REV=0x4132
CXX_FLAGS += -D__CORTEX_M4
CXX_FLAGS += -DDEVICE_STDIO_MESSAGES=1
CXX_FLAGS += -DTARGET_LIKE_CORTEX_M4
CXX_FLAGS += -D__MBED__=1
CXX_FLAGS += -DTOOLCHAIN_GCC_ARM
CXX_FLAGS += -DTARGET_MAX32630
CXX_FLAGS += -DDEVICE_PWMOUT=1
CXX_FLAGS += -DTARGET_CORTEX
CXX_FLAGS += -DTARGET_RTOS_M4_M7
CXX_FLAGS += -DDEVICE_LOWPOWERTIMER=1
CXX_FLAGS += -D__SYSTEM_HFX=96000000
CXX_FLAGS += -DDEVICE_PORTOUT=1
CXX_FLAGS += -DBLE_HCI_UART
CXX_FLAGS += -DTARGET_MAX32630FTHR
CXX_FLAGS += -DTARGET=MAX32630
CXX_FLAGS += -DTARGET_CORTEX_M
CXX_FLAGS += -include
CXX_FLAGS += mbed_config.h
CXX_FLAGS += -std=gnu++14
CXX_FLAGS += -fno-rtti
CXX_FLAGS += -Wvla
CXX_FLAGS += -c
CXX_FLAGS += -Wall
CXX_FLAGS += -Wextra
CXX_FLAGS += -Wno-unused-parameter
CXX_FLAGS += -Wno-missing-field-initializers
CXX_FLAGS += -fmessage-length=0
CXX_FLAGS += -fno-exceptions
CXX_FLAGS += -ffunction-sections
CXX_FLAGS += -fdata-sections
CXX_FLAGS += -funsigned-char
CXX_FLAGS += -MMD
CXX_FLAGS += -fno-delete-null-pointer-checks
CXX_FLAGS += -fomit-frame-pointer
CXX_FLAGS += -Os
CXX_FLAGS += -g
CXX_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
CXX_FLAGS += -mcpu=cortex-m4
CXX_FLAGS += -mthumb
CXX_FLAGS += -mfpu=fpv4-sp-d16
CXX_FLAGS += -mfloat-abi=softfp

ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -DTARGET_REV=0x4132
ASM_FLAGS += -DBLE_HCI_UART
ASM_FLAGS += -D__SYSTEM_HFX=96000000
ASM_FLAGS += -D__MBED_CMSIS_RTOS_CM
ASM_FLAGS += -D__CORTEX_M4
ASM_FLAGS += -D__CMSIS_RTOS
ASM_FLAGS += -DARM_MATH_CM4
ASM_FLAGS += -D__FPU_PRESENT=1
ASM_FLAGS += -DTARGET=MAX32630
ASM_FLAGS += -DOPEN_DRAIN_LEDS
ASM_FLAGS += -I/usr/src/mbed-sdk
ASM_FLAGS += -I../USBDevice
ASM_FLAGS += -I../USBDevice/USBAudio
ASM_FLAGS += -I../USBDevice/USBDevice
ASM_FLAGS += -I../USBDevice/USBHID
ASM_FLAGS += -I../USBDevice/USBMIDI
ASM_FLAGS += -I../USBDevice/USBMSD
ASM_FLAGS += -I../USBDevice/USBSerial
ASM_FLAGS += -I../max32630fthr
ASM_FLAGS += -I../max32630fthr/MAX14690
ASM_FLAGS += -I../mbed-os
ASM_FLAGS += -I../mbed-os/cmsis
ASM_FLAGS += -I../mbed-os/cmsis/TARGET_CORTEX_M
ASM_FLAGS += -I../mbed-os/cmsis/TARGET_CORTEX_M/TOOLCHAIN_GCC
ASM_FLAGS += -I../mbed-os/drivers
ASM_FLAGS += -I../mbed-os/events
ASM_FLAGS += -I../mbed-os/events/equeue
ASM_FLAGS += -I../mbed-os/features
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/ble
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/ble/services
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/hci
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/stack
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/stack/cfg
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/stack/include
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/util
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/wsf
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/wsf/generic
ASM_FLAGS += -I../mbed-os/features/FEATURE_BLE/targets/TARGET_Maxim/exactLE/wsf/include
ASM_FLAGS += -I../mbed-os/features/filesystem
ASM_FLAGS += -I../mbed-os/features/filesystem/bd
ASM_FLAGS += -I../mbed-os/features/filesystem/fat
ASM_FLAGS += -I../mbed-os/features/filesystem/fat/ChaN
ASM_FLAGS += -I../mbed-os/features/frameworks
ASM_FLAGS += -I../mbed-os/features/frameworks/greentea-client
ASM_FLAGS += -I../mbed-os/features/frameworks/greentea-client/greentea-client
ASM_FLAGS += -I../mbed-os/features/frameworks/unity
ASM_FLAGS += -I../mbed-os/features/frameworks/unity/unity
ASM_FLAGS += -I../mbed-os/features/frameworks/utest
ASM_FLAGS += -I../mbed-os/features/frameworks/utest/utest
ASM_FLAGS += -I../mbed-os/features/mbedtls
ASM_FLAGS += -I../mbed-os/features/mbedtls/inc
ASM_FLAGS += -I../mbed-os/features/mbedtls/inc/mbedtls
ASM_FLAGS += -I../mbed-os/features/mbedtls/platform
ASM_FLAGS += -I../mbed-os/features/mbedtls/platform/inc
ASM_FLAGS += -I../mbed-os/features/netsocket
ASM_FLAGS += -I../mbed-os/features/netsocket/cellular
ASM_FLAGS += -I../mbed-os/features/netsocket/cellular/generic_modem_driver
ASM_FLAGS += -I../mbed-os/features/netsocket/cellular/utils
ASM_FLAGS += -I../mbed-os/hal
ASM_FLAGS += -I../mbed-os/hal/storage_abstraction
ASM_FLAGS += -I../mbed-os/platform
ASM_FLAGS += -I../mbed-os/rtos
ASM_FLAGS += -I../mbed-os/rtos/TARGET_CORTEX
ASM_FLAGS += -I../mbed-os/rtos/TARGET_CORTEX/rtx4
ASM_FLAGS += -I../mbed-os/rtos/TARGET_CORTEX/rtx5
ASM_FLAGS += -I../mbed-os/targets/TARGET_Maxim
ASM_FLAGS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630
ASM_FLAGS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/TARGET_MAX32630FTHR
ASM_FLAGS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/device
ASM_FLAGS += -I../mbed-os/targets/TARGET_Maxim/TARGET_MAX32630/mxc
ASM_FLAGS += -include
ASM_FLAGS += /filer/workspace_data/exports/7/70302a3b6131d463faf4eff012477f0c/FTHR_Bluetooth/mbed_config.h
ASM_FLAGS += -x
ASM_FLAGS += assembler-with-cpp
ASM_FLAGS += -c
ASM_FLAGS += -Wall
ASM_FLAGS += -Wextra
ASM_FLAGS += -Wno-unused-parameter
ASM_FLAGS += -Wno-missing-field-initializers
ASM_FLAGS += -fmessage-length=0
ASM_FLAGS += -fno-exceptions
ASM_FLAGS += -ffunction-sections
ASM_FLAGS += -fdata-sections
ASM_FLAGS += -funsigned-char
ASM_FLAGS += -MMD
ASM_FLAGS += -fno-delete-null-pointer-checks
ASM_FLAGS += -fomit-frame-pointer
ASM_FLAGS += -Os
ASM_FLAGS += -g
ASM_FLAGS += -DMBED_TRAP_ERRORS_ENABLED=1
ASM_FLAGS += -mcpu=cortex-m4
ASM_FLAGS += -mthumb
ASM_FLAGS += -mfpu=fpv4-sp-d16
ASM_FLAGS += -mfloat-abi=softfp


LD_FLAGS :=-Wl,--gc-sections -Wl,--wrap,main -Wl,--wrap,_malloc_r -Wl,--wrap,_free_r -Wl,--wrap,_realloc_r -Wl,--wrap,_memalign_r -Wl,--wrap,_calloc_r -Wl,--wrap,exit -Wl,--wrap,atexit -Wl,-n -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=softfp 
LD_SYS_LIBS :=-Wl,--start-group -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys -lexactLE -Wl,--end-group

# Tools and Flags
###############################################################################
# Rules

.PHONY: all lst size


all: $(PROJECT).bin $(PROJECT).hex size


.s.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  


.S.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Assemble: $(notdir $<)"
  
	@$(AS) -c $(ASM_FLAGS) -o $@ $<
  

.c.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CC) $(C_FLAGS) $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	+@$(call MAKEDIR,$(dir $@))
	+@echo "Compile: $(notdir $<)"
	@$(CPP) $(CXX_FLAGS) $(INCLUDE_PATHS) -o $@ $<


$(PROJECT).link_script.ld: $(LINKER_SCRIPT)
	@$(PREPROC) $< -o $@



$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS) $(PROJECT).link_script.ld 
	+@echo "$(filter %.o, $^)" > .link_options.txt
	+@echo "link: $(notdir $@)"
	@$(LD) $(LD_FLAGS) -T $(filter-out %.o, $^) $(LIBRARY_PATHS) --output $@ @.link_options.txt $(LIBRARIES) $(LD_SYS_LIBS)


$(PROJECT).bin: $(PROJECT).elf
	$(ELF2BIN) -O binary $< $@
	+@echo "===== bin file ready to flash: $(OBJDIR)/$@ =====" 

$(PROJECT).hex: $(PROJECT).elf
	$(ELF2BIN) -O ihex $< $@


# Rules
###############################################################################
# Dependencies

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)
endif

# Dependencies
###############################################################################
# Catch-all

%: ;

# Catch-all
###############################################################################