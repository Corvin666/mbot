"c:\Program Files\Arduino\hardware\tools\avr\bin\avrdude" "-CC:\Program Files\Arduino\hardware/tools/avr/etc/avrdude.conf" -q -q -patmega328p -carduino -P\\.\COM4 -b115200 -D -Uflash:w:mBot_IR_Ivan.default.hex:i