# real (hardware) driver
obj-$(CONFIG_ALLEGRO_CODEC_DRIVER) += al5r.o
al5r-objs := al_main.o al_ip.o al_alloc.o al_alloc_ioctl.o

ifeq ($(CONFIG_ALLEGRO_NO_DMABUF),1)
  al5r-objs += al_no_dmabuf.o
else
  al5r-objs += al_dmabuf.o
endif
