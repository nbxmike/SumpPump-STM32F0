# SumpPump-STMF0
Development stopped on this repo as I've moved to STM32F103
based "Blue Pill" boards for most of my work.  As a result, 
because I have plenty of them around, this project is now
run on a bluepill.  Nothing is necessarily wrong with
this code, I've just moved to a different platform.

 A sump pump controler based on the STM32F0 Discovery board.
 
 My house is built on granite slab so it is impossible to dig a normal
 depth sump basin.  Without sufficent basin depth, commercial sump 
 pumps were unusable (I tried) and so I have created this firmware to allow
 a drainage system based on a garden fountain pump which only  requires 
 1 inch / 25mm of water to operate. 
 
 Software is developed using the STM32CubeIDE avaliable from ST (at the
 time of publication: https://www.st.com/en/development-tools/stm32cubeide.html
 though probably subject to change).  The FreeRTOS is release 9 as provided
 inside the IDE using the "Middleware" tab to create the image.
 
 I am attempting to provide everything needed to clone and then import this
 into your workspace as an STM32 project.  If you find there are issues with
 this let me know and I will try to fix the missing files.
 
 Licensed under the BSD 3-Clause to be consistant with what ST licenses their
 code as.
 
 Best of luck
 Mike
