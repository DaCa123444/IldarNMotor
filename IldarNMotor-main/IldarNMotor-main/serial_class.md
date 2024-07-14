#Serial_Class

#### 구성
1. 생성자 `Serial(PinName tx, PinName rx[,int baud=115200])`
2. 통신 설정 `baud(int baud=115200);`
3. 프레임 설정 `format(int bits=8,Parity parity = SerialBase::None,int stopbit=1)`
4. 콜백 함수 설정 `attach(Callback<void()> func,IrqType type = RxIrq)`
 * 파라미터1. 반환x, 호출할 함수 주소
 * 파라미터2. 인터럽트 종류
 * 멤버 함수를 콜백함수로 지정하는 경우
  `attach(Callback(this,&myclassname::ISRfunction),RxIrq);`
 * 아래 과정으로 파라미터1, 호출할 주소를 지정한다.
  + this로 함수가 포함되어 있는 객체의 주소를 전달
  + 절대 주소로 함수 지정

5. 확인 함수 `bool writable();` 
6. 쓰기 함수 ` int putc(int c)`
 * 전송 버퍼에서 1바이트 쓰기
7. 읽기 함수 `int getc();`
 * 수신 버퍼에서 1바이트 읽기







