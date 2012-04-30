#!/bin/bash

sudo avrdude -c avrispmkII -P usb -p x128a1 -U flash:w:medulla.hex -x pdi
