# MPU_6050_SSD1306
Displaying data of mpu 6050 on ss1306 128*64 display


I2C1 of stm32 f410RB-Nucleo used on fast mode.

Data is read from mpu6050, converted into string and send to the display.

Dsiplay library used from: https://github.com/afiskon/stm32-ssd1306
For MPU 6050 register map refer to: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6500-Register-Map2.pdf
