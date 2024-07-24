# lidar car

## 차례

 1. lidar
 2. servo_motor
 3. 바퀴 작동 x      : 키 조종 장비가 없이 측정값을 확인하기가 어려워 제외
 4. 키조종 x 	: 시간이 부족해 제외
 5. 이벤트 및 메세지큐
## lidar

#### i2c 데이터 발신
```
void tfluna_Write(uint8_t Address, uint8_t data)
{
  HAL_I2C_Mem_Write(&hi2c1,tfluna,Address,1,(uint8_t*)&data,1,10);
}
```
#### i2c 데이터 수신
```
uint8_t tfluna_Read(uint8_t Address)
{
	uint8_t data = 0;
	HAL_I2C_Mem_Read(&hi2c1,tfluna,Address,1,(uint8_t*)&data,1,10);
	// hi2c1  : handle i2c ,tfluna: devaddress(7bit addr+1bit command) ,
	return data;
}
``` 
#### 거리값 읽기 (0x00과 0x01에 존재)
```
void Read_tfluna_Data()
{
	dist = (tfluna_Read(0x00)<<8)|tfluna_Read(0x01);
}
```
#### 초기화 
```
void Init_tfluna()
{
	device_address_temp = tfluna_Read(0x22); //default = 0x10
	printf("\n\r Device Address= 0x%02X",device_address_temp); // 0x%02X : 	%02X : 16진수
	if(device_address_temp!=0x10)  // i2c id 일치 확인
	{
		printf("\n\r Deveice Address Error! Please Reset CPU!!");
		while(1);
	}
```

#### 위 함수들을 바탕으로 T2 function함수를 생성
```
void T2function(){
	printf("\n\r * dist = %ld",dist);
	Read_tfluna_Data();

}
```

#### 세마포어 : 태스크 이용중 타 태스크의 접근을 제한
 * 특히 통신을 사용하기 위해 메세지큐를 작성. 작성 중 데이터 업데이트를 방지하기 위함
`myBinarySem01Handle = osSemaphoreNew(1, 1, &myBinarySem01_attributes);`

#### 태스크 설정
`t2_lidarHandle = osThreadNew(StartTask02, NULL, &t2_lidar_attributes);`
`t4_detect_objHandle = osThreadNew(StartTask04, NULL, &t4_detect_obj_attributes);`

#### T2(라이다 태스크)
 * 세마포어의 목적인 데이터값 신뢰성을 확보하기 위해 세마포어 푸는 것은 Task4인 메시지큐에 데이터 넣기 후에 진행
```
void StartTask02(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreAcquire(myBinarySem01Handle,10)==osOK){
		  T2function(); 
		  osDelay(100);
		  if(dist >=1280)
		  {
		  osSemaphoreRelease(myBinarySem01Handle);
		  osDelay(20);
		  }
	  }
  }
}

```



#### Task4
```
void StartTask04(void *argument)
{
  /* Infinite loop */
  for(;;)
  {
	  T4function();
  }
}
```


#### 특정 조건에서 이벤트 생성 및 메세지 큐 생성
 * 실행시 세마포어핸들러01을 풀어줌
```
void T4function(){

	if(dist <1280)
	{
		QueueData_t queueData;
		queueData.dist = (uint32_t)dist; // 
		queueData.servo_motor = (uint32_t)angle;
		//printf("%d!!!!!!!!",osMessageQueueGetMsgSize(msgQHandle));
		osEventFlagsSet(myEvent01Handle, 1); // 0bit 이벤트 flag on
		osStatus_t status = osMessageQueuePut(msgQHandle, &queueData, 0, 10); 
		if (status == osOK)
		{
			printf("\n\rMessage sent");
		}
		else
		{
			printf("\n\rFailed to send message");
		}
	}
	if(dist <1280){
		QueueData_t receivedData;
		osStatus_t status = osMessageQueueGet(msgQHandle, &receivedData, NULL, osWaitForever);
		uint16_t dist_;
		uint32_t servo_motor_;
	  		  if (status == osOK) {
	  			  dist_ = (uint16_t)receivedData.dist;
	  			  servo_motor_ = (uint8_t)receivedData.servo_motor;
	  		  }
	  		  printf("\r\n------------ From msgq-----------");
	  		  printf("\n\r dist : %d , receivedData : %ld \n",dist_,servo_motor_ );
	}
	osSemaphoreRelease(myBinarySem01Handle);
	osDelay(20);
	  //osDelay(100); // 감�??�� 10hz�?? ?��?��
}

```


## 서보모터 


#### 서보모터 작동
```
void T1function(){
	  //if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0) == GPIO_PIN_SET)
	  //{
		  set_Servo_Angle(&htim1, TIM_CHANNEL_1);
		  osDelay(10);
		  if (angle == 0 || angle == 90) {
			  direction = -direction; // Reverse direction
		  }
		  angle = angle + direction;
		  printf("\n\r * Tim1->CCR1 : %d", TIM1->CCR1);
	  //}
}
```


#### 각도 변환
 * 90도만 회전하기 위해 펄스 길이 결정
```
void set_Servo_Angle(TIM_HandleTypeDef *htim, uint32_t channel)
{
  uint32_t pulse_length = (angle * 2000 / 180) + 1000;
  __HAL_TIM_SET_COMPARE(htim, channel, pulse_length);
}
```

#### 모터 작동 태스크
```
void StartDefaultTask(void *argument)
{

  for(;;)
  {
	  //printf("\n\r 11");
	  if(osSemaphoreAcquire(myBinarySem02Handle,10)==osOK){
		  T1function(); //?���???? 모터 ?��?��

		  osSemaphoreRelease(myBinarySem02Handle);osDelay(100);
	  }
  }
}
```



## 이벤트 및 메세지큐

#### 구조체
 * dist 16bit , servo_motor 8bit이지만 전달하는 과정에서 하드웨어의 규격을 맞추는 것이 효율을 높이기 때문에 32bit 2개로 설정
```
typedef struct {
    uint32_t dist;
    uint32_t servo_motor;
} QueueData_t;
```

#### 이벤트 생성 및 메세지큐에 데이터 넣기
 * 거리값 dist가 1280보다 작으면 메세지큐를 생성하여 통신에 활용한다.
 * 이를 실행하기 위해 조건에서 이벤트를 flag on 시키고, 메세지큐를 넣는다.
 * 통신과 관련하여 선택하지 못한 것이 있어 여기서 큐를 꺼내서 값을 확인하는 과정도 포함되어 있다.
```
void T4function(){

	if(dist <1280)
	{
		QueueData_t queueData;
		queueData.dist = (uint32_t)dist; // 
		queueData.servo_motor = (uint32_t)angle;
		//printf("%d!!!!!!!!",osMessageQueueGetMsgSize(msgQHandle));
		osEventFlagsSet(myEvent01Handle, 1); // 0bit?�� ?��벤트 flag on
		osStatus_t status = osMessageQueuePut(msgQHandle, &queueData, 0, 10); 
		if (status == osOK)
		{
			printf("\n\rMessage sent");
		}
		else
		{
			printf("\n\rFailed to send message");
		}
	}
	if(dist <1280){
		QueueData_t receivedData;
		osStatus_t status = osMessageQueueGet(msgQHandle, &receivedData, NULL, osWaitForever);
		uint16_t dist_;
		uint32_t servo_motor_;
	  		  if (status == osOK) {
	  			  dist_ = (uint16_t)receivedData.dist;
	  			  servo_motor_ = (uint8_t)receivedData.servo_motor;
	  		  }
	  		  printf("\r\n------------ From msgq-----------");
	  		  printf("\n\r dist : %d , receivedData : %ld \n",dist_,servo_motor_ );
	}
	osSemaphoreRelease(myBinarySem01Handle);
	osDelay(20);

}

```