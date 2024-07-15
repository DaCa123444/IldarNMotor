# 2. motor 연결

#### 연결 구조
 - l293d 모터 드라이버 사용.
 - PA8 : TIM1-PWM 이용
 - PB1 : GPIO OUTPUT, SET
 - PB2 : GPIO OUTPUT, RESET
 - GND : L293D의 IN2와 연결 ( GND 연결을 위해)
   * 파워의 GND와 연결해야하나?

#### L293D 연결
 - 외부 전원 12V
 - 모터 OUT1 :GND / OUT2 : 5V
 - ENA : TIM1-PWM
 - IN1 : PB1 - gpio_pin_writepin
 - IN2 : PB2 - gpio_pin_writepin
  * in1,in2 관계 :  (0,1) : 정방향이면 (1,0) : 역방향 , (0,0),(1,1) : stop
 - tim->ccr1 = 0; : 타이머1의 채널1에 대한 compare 레지스터 기준을 0으로 설정한다. 이는 pwm 신호의 duty cycle을 0%로 설정하여 항상 low신호를 출력하도록 한다.

 - tim-> ccr1 = 1000 ; duty cycle동안 계속 high를 출력하도록 한다.
  * prescaler : 36 - 1 // 타이머 클럭을 1MHZ로 설정
  * counter period 1000  // 타이머가 1000마다 카운트됨

 - 정리용 예문
 * **36 MHz의 클럭에서 PSR을 36,000 - 1로 설정하고, counter period를 1,000 - 1로 설정한다.**
 * **그러면 1초 동안 1 Hz로 작동하게 되고, duty를 500으로 설정한 PWM ch1이 있을 경우 0.5초 동안 HIGH를 공급한다. (0.5초는 pwm 주기의 절반을 의미한다.)**



파워가 연결된 상태에서  프로그램 안올려둔 상태라도 선 분리시, GND로 인식하고 모터 작동됨

#### 제어 















