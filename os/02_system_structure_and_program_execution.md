# System Structure & Program Execution

> 하드웨어 위에서 프로그램이 어떻게 돌아가는가



## 컴퓨터 시스템 구조

| Computer    | I/O device                             |
| ----------- | -------------------------------------- |
| CPU, Memory | Disk, Monitor, Keyboard, Printer, etc. |

- CPU는 매 클럭마다 Memory에서 기계어(instruction set)를 읽어 명령을 실행
- register는 memory 보다 빠른 저장 공간
- mode bit은 cpu에서 실행되는 프로그램이 운영체제인지 사용자 프로그램인지 구분
- interrupt line
  - Disk와 같은 I/O device는 처리 속도가 CPU에 비해 굉장히 느림
  - I/O device에 접근하는 것은 운영체제를 통해서만 가능(보안 등의 이유로)
  - 어떤 프로그램이 Disk에 있는 정보를 읽어와야 할 때 해당 프로그램은 Disk controller에 정보를 요청
  - 그러다가 Disk controller가 정보를 찾으면(수행이 끝나면) 해당 정보는 local buffer에 입력되게 되고, controller가 interrupt를 걺
  - CPU는 하나의 instruction이 끝나고나면 interrupt line을 check하고, interrupt가 존재하면 CPU의 권한을 운영체제에게 넘겨주게 됨
  - 운영체제는 I/O device의 buffer에 있는 정보를 가져와서 main memory에 복사하고, CPU 권한을 다시 그 전에 interrupt 당한 프로그램에게 넘겨주게 됨
- timer에는 각 프로그램이나 운영체제가 CPU의 권한을 가질 할당 시간을 정해놓는 곳
- device controller
- local buffer



## Mode bit

- 사용자 프로그램의 잘못된 수행으로 다른 프로그램 및 운영체제에 피해가 가지 않도록 하기 위한 보호 장치 필요

- Mode bit을 통해 하드웨어적으로 두 가지 모드의 operation 지원

  ```
  1 사용자 모드: 사용자 프로그램 수행
  0 모니터 모드: OS 코드 수행
  ```

  - 보안을 해칠 수 있는 중요한 명령어는 모니터 모드(또는 커널 모드, 시스템 모드라고도 함)에서만 수행가능한 **특권명령**으로 규정
  - Interrupt나 Exception 발생 시 하드웨어가 mode bit을 0으로 바꿈
  - 사용자 프로그램에게 CPU를 넘기기 전에 mode bit을 1로 세팅



## Timer

- 타이머
  - 정해진 시간이 흐른 뒤 운영체제에게 제어권이 넘어가도록 인터럽트를 발생시킴
  - 타이머는 매 클럭 틱 때마다 1씩 감소
  - 타이머 값이 0이 되면 타이머 인터럽트 발생
  - CPU를 특정 프로그램이 독점하는 것으로부터 보호
- 타이머는 time sharing을 구현하기 위해 널리 이용됨
- 타이머는 현재 시간을 계산하기 위해서도 사용



## Device Controller

- I/O device controller
  - 해당 I/O 장치유형을 관리하는 일종의 작은 CPU
  - 제어 정보를 위해 control register, status register를 가짐
  - **local buffer**를 가짐 (일종의 data register)
- I/O는 실제 device와 local buffer 사이에서 일어남
- Device controller는 I/O가 끝났을 경우 interrupt로 CPU에 그 사실을 알림



- device driver(장치구동기)
  - OS 코드 중 각 장치별 처리루틴 -> software
  - device controller를 작동시키기 위한 코드는 firmware driver
- device controller(장치제어기)
  - 각 장치를 통제하는 일종의 작은 CPU -> hardware



## 입출력(I/O)의 수행

- 모든 입출력 명령은 특권 명령
- 사용자 프로그램은 어떻게 I/O를 하는가?
  - 시스템콜(System call)
    - 사용자 프로그램이 운영체제의 서비스를 받기 위해 커널 함수를 호출하는 것
    - I/O 접근 및 요청은 운영체제만 가능
    - 일반적인 함수호출과는 다르게 Interrupt를 걸어서 CPU 권한이 OS에게 넘어가게 한다
  - trap을 사용하여 인터럽트 벡터의 특정 위치로 이동
  - 제어권이 인터럽트 벡터가 가리키는 인터럽트 서비스 루틴으로 이동
  - OS는 올바른 I/O 요청인지 확인(예. 해당 파일에 대한 접근 권한이 있는지) 후 I/O 수행
  - I/O 완료 시 제어권을 시스템콜 다음 명령으로 옮김



## Interrupt

> 인터럽트 당한 시점의 레지스터와 program counter를 save한 후 CPU의 제어를 인터럽트 처리 루틴에 넘기는 것
>
> 현대의 운영체제는 인터럽트에 의해 구동된다.

- 넓은 의미
  - Interrupt (하드웨어 인터럽트)
    - 하드웨어가 발생시킨 인터럽트
    - I/O 장치가 정보 교신을 하기 위해
  - Trap (소프트웨어 인터럽트)
    - Exception: 프로그램이 오류를 범한 경우
    - System call: 프로그램이 커널 함수를 호출하는 경우(사용자 프로그램이 직접 처리하지 못하고 OS의 도움을 받아야 할 때)
- 인터럽트 관련 용어
  - 인터럽트 벡터
    - 해당 인터럽트의 처리 루틴 주소를 가지고 있음
  - 인터럽트 처리 루틴(=Interrupt Service Routine, 인터럽트 핸들러)
    - 해당 인터럽트를 처리하는 커널 함수
  - 예를 들어, disk에서 어떤 자료를 읽는 정보를 완료했을 때 인터럽트를 발생시키면, 인터럽트 3번 라인이 세팅이 된다. 3번 라인에 있는 주소는 디스크 로컬 버퍼에 있는 정보를 가져오는 처리 루틴 함수를 가리킨다. 인터럽트 종류마다 처리루틴 함수의 주소를 가리키는 인터럽트 벡터가 존재한다.



## DMA Controller

> I/O 장치가 많아질수록 CPU에 걸리는 Interrupt가 증가해서 overhead 발생, 이는 효율적이지 않다. DMA(Direct Memory Access) Controller가 그 역할을 대신 처리해서 local buffer의 내용이 모두 main memory에 복사가 되거나 했을 경우 CPU에 인터럽트를 걸어 알려줌



