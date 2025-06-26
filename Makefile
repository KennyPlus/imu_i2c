imu: imu.c driver/bmi270.c
	gcc -o imu imu.c driver/bmi270.c driver/bmi270_config_data.c -Idriver -lm


clean:
	rm -f imu
