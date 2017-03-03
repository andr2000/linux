# Copyright (C) 2012 The CyanogenMod Project
# Copyright (C) 2016 GloballLogic
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Android makefile to build kernel as a part of Android Build

LOCAL_PATH := $(call my-dir)

KERNEL_SRC := $(TARGET_KERNEL_SOURCE)

KERNEL_DEFCONFIG := $(TARGET_KERNEL_CONFIG)
VARIANT_DEFCONFIG := $(TARGET_KERNEL_VARIANT_CONFIG)
SELINUX_DEFCONFIG := $(TARGET_KERNEL_SELINUX_CONFIG)

## Internal variables
ifeq ($(OUT_DIR),out)
KERNEL_OUT := $(ANDROID_BUILD_TOP)/$(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
else
KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/KERNEL_OBJ
endif

KERNEL_CONFIG := $(KERNEL_OUT)/.config

TARGET_PREBUILT_INT_KERNEL := $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/Image.gz
TARGET_PREBUILT_INT_KERNEL_TYPE := Image.gz

KERNEL_MODULES_INSTALL := vendor
KERNEL_MODULES_OUT := $(TARGET_OUT_VENDOR)/lib64/modules

define mv-modules
    mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.order`;\
    if [ "$$mdpath" != "" ];then\
        mpath=`dirname $$mdpath`;\
        ko=`find $$mpath/kernel -type f -name *.ko`;\
        for i in $$ko; do $(ANDROID_TOOLCHAIN)/aarch64-linux-android-strip --strip-unneeded $$i;\
        mv $$i $(KERNEL_MODULES_OUT)/; done;\
    fi
endef

define clean-module-folder
    mdpath=`find $(KERNEL_MODULES_OUT) -type f -name modules.order`;\
    if [ "$$mdpath" != "" ];then\
        mpath=`dirname $$mdpath`; rm -rf $$mpath;\
    fi
endef

ifeq ($(TARGET_ARCH),arm64)
    ifneq ($(TARGET_KERNEL_CUSTOM_TOOLCHAIN),)
        ifeq ($(HOST_OS),darwin)
            ARM_CROSS_COMPILE:=CROSS_COMPILE="$(ccache) $(ANDROID_BUILD_TOP)/prebuilts/gcc/darwin-x86/aarch64/$(TARGET_KERNEL_CUSTOM_TOOLCHAIN)/bin/aarch64-linux-android-"
        else
            ARM_CROSS_COMPILE:=CROSS_COMPILE="$(ccache) $(ANDROID_BUILD_TOP)/prebuilts/gcc/linux-x86/aarch64/$(TARGET_KERNEL_CUSTOM_TOOLCHAIN)/bin/aarch64-linux-android-"
        endif
    else
        ARM_CROSS_COMPILE:=CROSS_COMPILE="$(ccache) $(ANDROID_TOOLCHAIN)/aarch64-linux-android-"
    endif
endif

ifeq ($(TARGET_KERNEL_MODULES),)
    TARGET_KERNEL_MODULES := no-external-modules
endif

$(KERNEL_OUT):
	mkdir -p $(KERNEL_OUT)

$(KERNEL_MODULES_OUT):
	mkdir -p $(KERNEL_MODULES_OUT)

$(KERNEL_CONFIG): $(KERNEL_OUT)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) VARIANT_DEFCONFIG=$(VARIANT_DEFCONFIG) SELINUX_DEFCONFIG=$(SELINUX_DEFCONFIG) $(KCFLAGS) $(KERNEL_DEFCONFIG)

$(KERNEL_OUT)/piggy : $(TARGET_PREBUILT_INT_KERNEL)
	$(hide) gunzip -c $(KERNEL_OUT)/arch/$(TARGET_ARCH)/boot/compressed/piggy.gzip > $(KERNEL_OUT)/piggy

TARGET_KERNEL_BINARIES: $(KERNEL_OUT) $(KERNEL_CONFIG) $(KERNEL_MODULES_OUT) $(INSTALLED_RAMDISK_TARGET)
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) $(KCFLAGS) $(TARGET_PREBUILT_INT_KERNEL_TYPE) modules dtbs
	$(MAKE) -C $(KERNEL_SRC) O=$(KERNEL_OUT) INSTALL_MOD_PATH=../../$(KERNEL_MODULES_INSTALL) ARCH=$(TARGET_ARCH) $(ARM_CROSS_COMPILE) modules_install
	$(mv-modules)
	$(clean-module-folder)

$(TARGET_KERNEL_MODULES): TARGET_KERNEL_BINARIES

$(TARGET_PREBUILT_INT_KERNEL): $(TARGET_KERNEL_MODULES)
	$(mv-modules)
	$(clean-module-folder)

ifeq ($(TARGET_PREBUILT_KERNEL),)

include $(CLEAR_VARS)

KERNEL_IMG_PATH := $(TARGET_PREBUILT_INT_KERNEL)
$(KERNEL_IMG_PATH): $(TARGET_KERNEL_MODULES)

LOCAL_MODULE := $(PRODUCT_OUT)/kernel
LOCAL_PREBUILT_MODULE_FILE := $(KERNEL_IMG_PATH)
LOCAL_MODULE_PATH := $(ANDROID_BUILD_TOP)

include $(BUILD_EXECUTABLE)

endif
