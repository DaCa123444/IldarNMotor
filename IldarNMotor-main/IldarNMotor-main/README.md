# IldarNMotor

#### 결과물

#### 목록
 1. 인터럽트 - 직접 연결되는 작업은 없으나 모든 부분에서 활용되기 때문에 넣었다.
    + Point : EXTI 인터럽트 테이블의 메모리 주소
  * 과정 및 용어 
  * 레지스터 설정 및 기능 ( 메모리 관련해서는 메모리맵관련 파일에 정리)

 2. uart 
  * uart 개념
  * uart 설정
  * HAL_UART_Transmit 함수 분석
  * 인터럽트 이용 (간단히)
  * DMA 사용

 3. DMA
  * DMA 개념
  * DMA 제어기 블록 및 구조
  * 인터럽트와 DMA 호출 과정 차이

 4. timer
  * tick
  - interrupt에 의한 구동
  - sys clock 과 PSR, COUNTER PERIOD에 의한 클럭 주기와 듀티값
  - TIM1의 동작 타이밍도(COUNTER OVERFLOW , UPDATE EVENT, UPDATE INTERRUPT FLAG 등) 

-----------------------

#### 시작
 1. 외부 인터럽트
 * 과정 및 용어
 - 용어 
  + NVIC : Nested Vectored interrupt controller)
    * mcu의 인터럽트 요청 관리, 우선순위 제어
    * exti로부터 인터럽트 요청을 받아 ISR을 실행
    * 간단히 하면 `EXTI 인터럽트 테이블`

  + EXTI : External Interrupt/Event Controller
    * 신호의 라이징 엣지, 폴링 엣지를 감지하고 이를 인터럽트 요청으로 변환하여 NVIC에 요청을 전달
    * 간단히 하면 `사건 감지기`

  + EXTI Vector Table ( EXTI  인터럽트 테이블)
     * **이는 handler가 테이블에 사용되는 이유(추상화,범용성)**
  
 * 참고(in as)
 - `  .word     EXTI0_IRQHandler `
 - `   .weak      EXTI0_IRQHandler ` // 재구현 가능

##### 함수 내 기능 구현
```
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == 1)  // 1 대신 0x0001으로 하는게 맞나?
	{
		HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_0);
	}
}
```
##### switch는  
  - OFF 시 GND 및 4.7k옴과 연결되어 pull down을 구성한다.
  - ON 시 GND의 높은 저항으로 인해 5V를 PB0가 high신호를 받게 된다.



#### 레지스터 관련 설정 
 +  HAL_..
  1. 상승 엣지, 하강 엣지 선택
  2. 인터럽트 활성화 여부 결정
  3. 인터럽트의 NVIC의 활용 ENABLE 및 우선순위 결정

 - 2. 인터럽트 활성화 여부 결정
   + void HAL_NVIC_EnableIRQ(EXTI1_IRQn);
```
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn)
{
  /* Check the parameters */
  assert_param(IS_NVIC_DEVICE_IRQ(IRQn));
  
  /* Enable interrupt */
  NVIC_EnableIRQ(IRQn);
}
```

 - 3. 인터럽트 우선순위 결정
  + `HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);`
```
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{ 
  uint32_t prioritygroup = 0x00U;
  
  /* Check the parameters */
  assert_param(IS_NVIC_SUB_PRIORITY(SubPriority));
  assert_param(IS_NVIC_PREEMPTION_PRIORITY(PreemptPriority));
  
  prioritygroup = NVIC_GetPriorityGrouping();
  
  NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));
}
```


  + CMSIS 내 헤더 파일
 1. EXTI 및 NVIC 관련 레지스터 (stm32xxx..h)
  * 메모리 관련해서 다룰 내용이 있어 메모리맵 관련 파일에서 정리한다.
```
typedef struct
{
  __IO uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __IO uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __IO uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __IO uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __IO uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __IO uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

```

 
#### 인터럽트 기능 및 과정
 * 외부 장치, 호출, 예외 등 틍정한 사건이 발생했음을 알리고, CPU가 이를 처리할 수 있도록 하는 신호.
 - 과정 
  1. EXTI에 의해 인터럽트 신호 발생
  2. NVIC- 벡터 테이블 내 설정된 주소로 이동 -주소에 들어있는 함수(EXTI_IRQ_HANDLER) 실행
  3. EXTI_IRQ_HANDLER는 함수 내부에서 함수(HAL_GPIO_EXTI_IRQHANDLER) 호출
   * EXTI_IRQ_HANDLER는 범용성을 위해 다른 함수를 호출하는 용도로만 사용된다.
   * 대신 HAL_GPIO에서 하드웨어에 맞게 구성을 짠다. **하드웨어의 드라이버를 구성하는 원리** 
  4. EXTI_IRQ_HANDLER는 정해진 CALLBACK함수를 호출하고 문제를 처리한다.
 
#### 헷갈리는 부분: 구현의 차이이기는 한대 초기화는 어디서 진행하는가 // PR(Pending Reg)의 리셋은 어디서?



#### 2. uart 
  * uart 개념
   + **직렬(1:1) 통신 장치**
   + 동기 / 비동기 지원
   + 레지스터 구성 
      - DR(data reg) 
     * 송신 데이터 레지스터와 수신 데이터 레지스터 : 데이터 임시저장
      - SR(Status Reg) 
     * usart의 현 상태를 나타낸다.( 버퍼가 비어있는지, 데이터가 도착했는지)
      - CR(Control Reg)
     * 설정값(동기/비동기, 데이터 비트 길이, 패리티 비트 등)
   + 프레임 구성
      - pull up에서 시작( 작성을 알리기 위해 데이터 라인이 1->0 ; 클럭 1인 상태)
      - 7개의 비트 전송 후 패리티 비트 1개를 추가해 8개의 비트 전송
      - 여기서 패리티 비트는 무결성을 확인하기 위한 아주 약한 수준의 알고리즘으로 1byte의 합이 짝수 
      - 전송 속도 : 보 레이트(baud rate) : 1 / 비트 폭 : 1초에 전달되는 비트 개수
      * 1/115200 = 115200 개의 비트 전달
      * **72MHZ / 115,200 = 625 이므로 매우 섬세하게 측정되는 것을 이해할 수 있음** 


  * uart 설정
   + baudrate 115200
   + word length : 8bits
   + parity : none
   + stop bits : 1

  * `HAL_UART_Transmit` 함수
   - 8,9bit 확인 후 DR를 맞게 초기화 (0x01FF 및 0xFF) // 01ff에서 01: 상위 // ff 하위
   - 전송 완료 후 플래그 확인,초기화,종료

  * 인터럽트 이용 (간단히)
  + 시간 초과를 위해 타이머 사용(특히 systick) = 인터럽트 사용
  + 작동 자체를 변경하여, ISR을 통해 callback 함수에 입/출력 관련 함수를 정의한 후 인터럽트 처리로 기능을 수행할 수 있음( 인터럽트 작동원리는 인터럽트 참조)
   ( weak로 정의된 uart_Receive_IT 와 uart_RxCpltCallback을 재정의함)

  * DMA 사용
   + 프로세서가 데이터 블록을 준비해주고 전송을 완료하면 정리
   + DMA는 프로세서 대신하여 데이터 전송을 담당. 프로세서는 다른 일처리 가능

##### 구현
 - 거리값 출력 확인을 위한 프린트기능
`	  printf("\n\r dist = %ld",dist);`
 - 출력 함수
```
void __io_putchar(int ch)
{
	HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,0xFFFF); // 1 : 1byte , 0xffff : 65s
						      // huart2->instance->DR을 업데이트 후 전달하는 방식
}
```
**dma 방식을 적용하면 우선순위에 맞게 실행 후 기능할 것이므로 적용하는 것이 맞는지**
**uart2와 i2c1의 우선순위 찾아놓기 (datasheet)**

 - i2c의 경우 주소를 찾게 되는데 이때, 주소값이 일치하는지 확인하기 위해 프린트 기능
`	printf("\n\r Device Address= 0x%02X",device_address_temp);`
 * 이는 주변장비에 전원 문제에도 같이 걸림

 - 서보 모터의 회전도를 확인하는데에도 프린트 기능
`printf("\n\rTim1->CCR1 : %d",TIM1->CCR1);` // tim1의 CCR(Capture/Compare Register1): 듀티를 저장


#### 3. DMA
  * DMA 개념
   - **효율적 프로세서 사용을 위해, DMA가 일정 처리를 프로세서 대신 진행한다. DMA의 작업 전후로 데이터 블록을 처리만 프로세서가 처리한다.**
   - 처리란? DATA 버퍼에 있는 값을 메모리에 작성한다.
  * DMA 제어기 블록 및 구조
   - Stream과 ch로 구분된 매트릭스 형태
   - 그림 참조
   - UART,TIMER,SPI,I2C등 많은 요소의 DMA 설정이 잡혀있다. 
   - 우선순위는 숫자가 작을수록 크다.
   - 기능의 구현 - 인터럽트와 같이 weak 함수로 구성된 것들을 재구현
  * 인터럽트와 DMA 호출 과정 차이

 프로젝트에서의 적용 : DMA를 통해, ble 통신에서의 uart 통신을 담당할 것으로 예상 (키 모듈 제작 후 구현 계획)


 


#### 4. timer
   + **인터럽트에 의한 실시**
  * tick
   - main() 내 while(1)에 아무 내용이 없을 수 있다. - 작업을 인터럽트로 실시
   - 과정
     + Systick_Handler함수가 초기화 및 Systick_IRQHandler함수 호출
     + HAL_Systick_IRQHandler 함수는 HAL_Systick_Callback 함수 호출
     + HAL_Systick_Callback 함수를 재선언하여 사용자 설정한 기능 수행

  * timer 입장에서 본 **pwm**
   - 타이머의 설정은 system timer에서 psr 등을 통해 APB마다 설정하여 사용.
  + 각각 기능에서도 타이머를 변경할 수 있음(PSR 등을 통해) 
   - 카운터는 펄스 간격을 바꾸는 것이지 주기와 상관없음
   - 펄스 간격을 통해 pwm width를 제어
   - count에 도달(up방식)할 경우 update event 및 update interrupt flag 펄스가 발생하며 초기화 발생



  - interrupt에 의한 구동
  - sys clock 과 PSR, COUNTER PERIOD에 의한 클럭 주기와 듀티값
  - TIM1의 동작 타이밍도(COUNTER OVERFLOW , UPDATE EVENT, UPDATE INTERRUPT FLAG 등) 




#### 5.i2c 
 - **1대다 통신**
  * 따라서 각 장치를 제어하는 주소가 중요 ( slave 들 각각 주소)
  * 최소 2btye씩 전달( 구성 : 주소 + 내용물)
 
 - 조건
  * sdl, scl 모두 풀업
  * scl : 마스터가 보낸 클럭 신호
  * sdl : 데이터값 전달 / ** 오직 scl이 low일때만 sdl도 low가 될 수 있다**
  * 위 sdl low 조건에 의해 scl이 high를 유지할때, sdl이 low가 됨을 통해 시작과 끝을 인식한다.
   + 그럼 연결된 모든 슬레이브들은 이 시작을 확인하고 첫 바이트에서 주소를 인식한 후, 아님을 확인하고 대기상태로 들어가는가?
  * 시작 조건은 위에서 끝
  * 정지 조건은 데이터 전송이 시작된 이후, sdl은 low상태를 유지한다. scl(마스터가 보낸 클럭 신호)가 high를 유지하고 sdl이 high로 변경됨을 통해 정지를 인식한다.
  
 - 위 내용을 바탕으로 lidar 부품을 연결하여 통신을 확인하였다.
 - 그림 추가
  * 먼저 master에서 lidar 주소 + 쓰기모드로 읽을 데이터의 주소를 전달한다.
  * 그 후, lidar 주소 + 읽기 모드로 데이터를 읽는다.

#### 6. 서보 모터
 - 


#### 7. rtos
 - 하나의 mcu를 효율적으로 사용하여 여러 일(task)를 동시에 처리하는 것처럼 보이게 한다.
 - 가능한 원리 - task별로 스택을 설정하고 cpu에서 처리하는 관련 레지스터들을 스택에 저장하는 블록형식으로 데이터를 관리함으로써, 필요한 작업을 넣어서 처리하고 필요가 끝났을 때, 적절한 알고리즘으로 다음 태스크를 진행함

##### task 분리
 1. dc모터 회전
 2. 레이더 장비 측정
 3. 서보 모터 회전
 4. ble uart 입력 - **interrupt로 구현하면 불필요?**



### 이미지
![Alt text](/IldarNMotor-main/img/1.jpg)
![Alt text](/IldarNMotor-main/img/2.jpg)