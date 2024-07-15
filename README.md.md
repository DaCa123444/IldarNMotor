#Lidar_motor
#### 라이다

 - 라이다의 정보 수집 및 전달 과정
 1. tof 기술로 거리 측정

 2. 거리값을 ADC ( 16bits)

 3. 거리 데이터 저장
 0x00 R Dist_low
 0x01 R Dist_high

4. I2C 통신
 I2C 통신을 위해 슬레이브 주소와 받을 데이터 레지스터 주소가 필요
 - ID_I2C_SLAVE_ADDR=0x22 // value = 0x10
 - 장치가 일치하면 작동을 위해 장치의 환경설정을 변경한다.

5. 데이터 읽기

 - 0x00 Dist_low
 - 0x01 Dist_high
 - 0x02 Amp_low
 - 0x03 Amp_high
 - 0x04 Temp_low
 - 0x05 Temp_high
 - 0x06 Tick_low
 - 0x07 Tick_high
 - 0x08 Err_low
 - 0x09 Err_high
 - 0x0A version_Revision
 - 0x0B version_minor
 - 0x0C version_major


 * 주소값 확인시, R레지스터에 접근하는 것이므로 7+1(R)의 8bit로 구성되어 있을 수 있다?
#### 스텝모터



----------------------------




]"