 2. uart 
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
   - 8,9bit 확인 후 DR를 맞게 초기화 (0x01FF 및 0xFF)
   - 전송 완료 후 플래그 확인,초기화,종료

  * 인터럽트 이용 (간단히)
  + 시간 초과를 위해 타이머 사용(특히 systick) = 인터럽트 사용
  + 작동 자체를 변경하여, ISR을 통해 callback 함수에 입/출력 관련 함수를 정의한 후 인터럽트 처리로 기능을 수행할 수 있음( 인터럽트 작동원리는 인터럽트 참조)
   ( weak로 정의된 uart_Receive_IT 와 uart_RxCpltCallback을 재정의함)

  * DMA 사용
   + 프로세서가 데이터 블록을 준비해주고 전송을 완료하면 정리
   + DMA는 프로세서 대신하여 데이터 전송을 담당. 프로세서는 다른 일처리 가능


 3. DMA
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


 4. timer
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

5.i2c 
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

