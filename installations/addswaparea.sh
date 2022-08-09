#!/bin/bash

JETSON_DTS="$(tr -d '\0' < /proc/device-tree/nvidia,dtsfilename | sed 's/.*\///')"

case $JETSON_DTS in
    *2180*) JETSON_TYPE="TX1" ;;
    *3310*) JETSON_TYPE="TX2" ;;
    *3489-0080*) JETSON_TYPE="TX2 4GB" ;;
    *3489*) JETSON_TYPE="TX2i" ;;
    *2888-0006*) JETSON_TYPE="AGX Xavier [8GB]" ;;
    *2888-0001*) JETSON_TYPE="AGX Xavier [16GB]" ;;
    *2888-0004*) JETSON_TYPE="AGX Xavier [32GB]" ;;
    *2888-0005*) JETSON_TYPE="AGX Xavier [64GB]" ;;
    *2888*) JETSON_TYPE="AGX Xavier [32GB]" ;;
    *3448-0002*) JETSON_TYPE="Nano" ;;
    *3448*) JETSON_TYPE="Nano (Developer Kit Version)" ;;
    *3668-0001*) JETSON_TYPE="Xavier NX" ;;
    *3668-0003*) JETSON_TYPE="Xavier NX [16GB]" ;;
    *3668*) JETSON_TYPE="Xavier NX (Developer Kit Version)" ;;
    *3701*) JETSON_TYPE="Jetson AGX Orin (Developer Kit Version)" ;;
    *) JETSON_TYPE="" ;;
esac

UseDLA=1
case $JETSON_TYPE in
    *Nano*) UseDLA=0;;
esac

if [ "$UseDLA" -eq 0 ] ; then
    sudo dd if=/dev/zero of=/pram6 bs=1024 count=2621440
    sudo chown root:root /pram6
    sudo chmod 0600 /pram6
    sudo mkswap /pram6
    sudo swapon /pram6
    echo "/pram6 none swap sw 0 0" | sudo tee -a /etc/fstab > /dev/null
    free -h
fi