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



