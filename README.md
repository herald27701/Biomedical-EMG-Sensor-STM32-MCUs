# Qua nhánh master để lấy code.
Mạch y sinh đo điện cơ EMG được xử lí bởi MCU STM32
Chi tiết về 2 con STM32: 1 con STM32F103C8T6 bluepill có gắn mạch EMG và I2C LCD để hiển thị thông số đo của mạch EMG, động cơ servo sẽ do mạch EMG điều khiển
                          1 con STM32F103RCT6 Grapini của Grapetech, tham khảo tại https://store.grapetech.vn/product/bo-mach-vdk-grape32-unleashed-kit/
Ý tưởng của mạch: board Grapini thu nhận tín hiệu đo được từ board bluepill và xuất thông số ra màn hình, còn module HC06 thì sẽ dùng để ngắt interupt tín hiệu emg và điều khiển động cơ servo bằng app điện thoại

Tài liệu tham khảo:
UART I2C example: https://www.instructables.com/UART-and-I2C-Communications-Between-UNO-and-MEGA25/
Xung PWM: http://arduino.vn/reference/xung-pwm
Xung PPM: http://arduino.vn/reference/xung-ppm
STM32 Servo Motor control: https://www.youtube.com/watch?v=WMS0t9WGqVw&feature=youtu.be
Thiết lập Mạch EMG: https://www.engineersgarage.com/arduino-based-emg-monitor-ad8226/
Lọc nhiễu mềm: https://arduinogetstarted.com/faq/how-to-filter-noise-from-sensor
Paper về EMG và Mạch EMG: http://jst.tnu.edu.vn/jst/article/viewFile/4115/pdf
Interupt: https://tapit.vn/su-dung-uart-interrupt-rx-voi-cube-mx-tren-mcu-stm32f103c8t6/
I2C Module: https://tapit.vn/giao-tiep-stm32f103c8t6-voi-lcd-16x2-thong-qua-module-i2c/
STM32 Servo Control Config: https://controllerstech.com/servo-motor-with-stm32/

