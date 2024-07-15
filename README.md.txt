#RTOS_Delay 종류

task 두개에 대해 osdelay()함수대신 hal_delay()를 사용하게되면 priority가 높은 task가 cpu를 점령하게된다.
 * 원인 : systick은 hal_wait을 통해 바쁜 상태를 유지하고 있기 때문.
 * 대안 : 우리는 wait함수에 대응하여 interrupt를 발생시켰다. 여기서도 타이머 인터럽트를 통한다.

 - **osdelay는 역할을 끝내거나, 중간에 높은 우선순위가 올경우, 현 태스크를 대기상태로 만들고 우선순위 작업을 진행할 수 있게 블록 상태로 전환한다.**

 * wait함수와 interrupt를 실행하는 방법 vs freertos
  freertos 입장
  1. 태스크 스케쥴링 : 효율적 관리 및 스케쥴링 기능(우선순위 및 동적 생성,삭제)
  2. 태스크 간 동기화와 통신을 위한 다양한 메커니즘
  3. 동적 메모리와 고정 메모리 할당을 위한 다양한 메커니즘
  .. 등등


#### 구현

 - 우선순위 다르게 설정 (nomal / low)
```
osThreadId_t led1TaskHandle;
const osThreadAttr_t led1Task_attributes = {
  .name = "led1Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led2Task */
osThreadId_t led2TaskHandle;
const osThreadAttr_t led2Task_attributes = {
  .name = "led2Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
```

 - task에 hal_delay()로 넣음 // tim10을 통해 freertos를 작동
```
void startLED1Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	  HAL_Delay(500);		//osDelay(500);
  }
  /* USER CODE END 5 */
}
```


```
void startLED1Task(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_1);
	  HAL_Delay(500);		//osDelay(500);
  }
  /* USER CODE END 5 */
}
```


#### 결과물

 - HAL_Delay :  led1만 작동
 - os_Delay : led1, led2  작동