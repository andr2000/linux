#
# Makefile for the drm device driver.  This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.

drm_xen_front-objs := xen_drm_front.o xen_drm.o xen_drm_kms.o xen_drm_crtc.o \
		  xen_drm_timer.o xen_drm_shbuf.o

ifneq ($(CONFIG_DRM_XEN_FRONTEND_CMA),y)
	drm_xen_front-objs += xen_drm_gem.o
endif

obj-$(CONFIG_DRM_XEN_FRONTEND) += drm_xen_front.o
