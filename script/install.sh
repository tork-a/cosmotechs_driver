mknod /dev/pcio32hadrv c 242 0
chmod a+rw /dev/pcio32hadrv
insmod ../lib/pcio32hadrv.ko

mknod /dev/pcpg23idrv c 241 0
chmod a+rw /dev/pcpg23idrv
insmod ../lib/pcpg23idrv.ko
