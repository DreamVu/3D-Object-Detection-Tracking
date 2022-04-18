#!/bin/bash

./run

File1=Res2.png 
File2=data/fcm.bin
validated=false 

if test -f "$File1"; then  
    cp Res2.png /data/lut/
    cp Res2.png ../bin_files/root-data/data/lut/
    cp unit.txt /usr/local/bin/data
    
    cp -r er-files/ /data/lut/er-files
    cp -r er-files/ ../bin_files/root-data/data/lut/er-files
    
    rm -rf er-files
    rm Res2.png data.zip unit.txt
    validated=true
fi

if test -f "$File2"; then         
    cp data/fcm.bin /data/bin_files/
    cp data/fcm.bin ../bin_files/root-data/data/bin_files/
fi



if $validated; then     
    rm -rf data	   
    echo "[PAL:INFO] Camera data files are installed successfully." 
fi
 


