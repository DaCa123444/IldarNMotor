# 1. lidar 연결
 - i2c 통신
 * CLOCK SPEED : 10,000
 * slave Primary address length : 7bit

 - DMA 설정
 * I2C1_RX 요청에 대해 DMA1_Stream_0을 사용. 데이터 방향은 PERIPHERAL TO MEMORY 로 설정
 * I2C1의 이벤트, 에러 인터럽트 활성화
   * 이벤트 인터럽트 : 데이터 전송 완료, 데이터 수신 완료, 주소 매칭 등의 다양한 이벤트 처리(과정의 완료를 알림)
   * 에러 감지로 적절한 처리
 * `HAL_I2C_Mem_Read(&hi2c1,tfluna,Address,1,(uint8_t*)&data,1,10)`
 * `HAL_I2C_Mem_Write(&hi2c1,tfluna,Address,1,(uint8_t*)&data,1,10)`

 - 주소 찾기
 * device 장치 주소(장치 ID) = 0x10
 * 장치 내 장치 주소 저장소 = 0x20
 * `#define tfluna 0x10<<1`
 * 읽을 데이터 위치 : 0x00(low) , 0x01(high)
 * `dist = (tfluna_Read(0x00)<<8)|tfluna_Read(0x01); // uint16_t dist `


 - **lidar의 설정을 변경하기**
 * ?-?

