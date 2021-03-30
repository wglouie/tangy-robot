#!/bin/bash
# INCOMPLETE - NOT DYNAMIC ACCORDING TO THE KEYBOARD ID'S YET
chmod u+x store_output.sh

declare KEYBOARD_IDS=$(xinput list|grep Logitech|grep id=|cut -f 2|cut -f 2 -d = )
wait
keyboard_array=( $KEYBOARD_IDS )
wait


xinput -test 13 > keyboard0.txt &
xinput -test 14 > keyboard1.txt &


