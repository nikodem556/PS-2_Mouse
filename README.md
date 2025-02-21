# PS2_Mouse
This code was written on STM32-L476RG. It allows user to receive data packets from mouse that uses PS2 communication protocol, then processes it and send useful information, like which buttons are pressed or where the mouse has moved via USART 

Header file "MousePacket.h" contains more info about PS2 communication protocol frame. At the start of the "main.c" you got some predefined variables, that you can change to alter the way program works. As default it doesn't send anything via USAR when not receiving data packets from the mouse.
