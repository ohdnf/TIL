# Process Synchronization

> 프로세스 동기화, 다른 말로는 Concurrency Control(병행 제어)

Storage(Memory/Address Space)를 공유하는 Execution(CPU/Process)가 여럿 있을 경우 Race Condition의 가능성이 있다.



## OS에서 race condition은 언제 발생하는가?

### 1. kernel 수행 중 인터럽트 발생 시

- 커널 모드 running 중 interrupt가 발생하여 인터럽트 처리루틴이 수행
- 양쪽 다 커널 코드이므로 kernel address space 공유

#### 해결책

- 커널모드 사용 중일 때는 interrupt를 막는다.



### 2. Process가 system call을 하여 kernel mode로 수행 중인데 context switch가 일어나는 경우

#### If you preempt CPU while in kernel mode

1. System call read()
2. Time quantum expires & other process B needs PCU. Process A preempted CPU while in kernel
3. CPU returns to the process A

#### 해결책

> interrupt enable/disable

커널 모드에서 수행 중일 때는 CPU를 preempt하지 않음

커널 모드에서 사용자 모드로 돌아갈 때 preempt



### 3. Multiprocessor에서 shared memory 내의 kernel data

#### 해결책

1. 한 번에 하나의 CPU만이 커널에 들어갈 수 있게 하는 방법(비효율적)
2. 커널 내부에 있는 각 공유 데이터에 접근할 때마다 그 데이터에 대한 lock/unlock을 하는 방법



## Process Synchronization 문제

- 공유 데이터(shared data)의 동시 접근(concurrent access)은 데이터의 불일치 문제(inconsistency)를 발생시킬 수 있다.
- 일관성(consistency) 유지를 위해서는 협력 프로세스(cooperating process) 간의 실행 순서(orderly execution)를 정해주는 메커니즘 필요

#### Race condition

- 여러 프로세스들이 동시에 공유 데이터를 접근하는 상황
- 데이터의 최종 연산 결과는 마지막에 그 데이터를 다룬 프로세스에 따라 달라짐
- Race condition을 막기 위해서는 concurrent process는 동기화(synchronize)되어야 한다.



## The Critical-Section Problem(임계구역 문제)

- N개의 프로세스가 공유 데이터를 동시에 사용하기를 원하는 경우
- 각 프로세스의 code segment에는 공유 데이터를 접근하는 코드인 critical section이 존재

### Initial Attempt to Solve problem

- 하나의 프로세스가 critical section에 있을 때 다른 모든 프로세스는 critical section에 들어갈 수 없어야 한다.

- 두 개의 프로세스가 있다고 가정(P~1~, P~2~)

- 프로세스들의 일반적인 구조

  ```c
  do {
    entry section
    critical section
    exit section
    remainder section
  } while (1);
  ```

- 프로세스들은 수행의 동기화(Synchronization)을 위해 몇몇 변수들을 공유할 수 있다.

  - Synchronization variable



## 프로그램적 해결법의 충족 조건

### Mutual Exclusion(상호 배제)

프로세스 P~i~가 critical section 부분을 수행 중이면 다른 프로세스들은 그들의 critical section에 접근하면 안 된다.

### Progress(진행)

아무도 critical section에 있지 않은 상태에서 critical section에 들어가고자 하는 프로세스가 있으면 허용해주어야 한다.

### Bounded waiting(유한 대기)

어떤 프로세스가 critical section에 들어가려고 요청한 후부터 요청이 허용될 때까지 다른 프로세스들이 critical section에 들어가는 횟수에 제한이 있어야 한다.

#### 가정

- 모든 프로세스들의 수행 속도는 0보다 크다.
- 프로세스 간의 상대적인 수행 속도는 고려하지 않는다.



## Algorithm 1

### Synchronization variable

```c
int turn;
turn = 0;	// initialization
```

> P~i~ can enter its critical section `if (turn == i)` 

### Process P~0~

> Process P~1~의 경우 `while (turn != 1);`, `turn = 0;`

```c
do {
  while (turn != 0);	/* My turn? */
  // critical section
  turn = 1;	/* Now it's your turn */
  // remainder section
} while (1);
```

### Limitation

- Progress 조건을 만족시키지 못 함



## Algorithm 2

### Synchronization variable

```c
boolean flag[2];
/* No one is in CS */
flag[0] = false;
flag[1] = false;
```

> P~i~ is ready to enter its critical section `if (flag[i] == true)`

### Progress P~i~

```c
do {
  flag[i] = true;		/* Pretend I am in */
  while (flag[j]);	/* Is he also in? then wait */
  // critical section
  flag[i] = false;	/* I am out now */
  // remainder section
} while (1);
```

### Limitation

- Satisfies mutual exclusion, but **not progress requirement**.
- 둘 다 2행까지 수행 후 끊임없이 양보하는 상황 발생 가능



## Algorithm 3(Peterson's algorithm)

> Combined synchronization variable of algorithm 1 and 2

### Process P~i~

```c
do {
  flag[i] = true;	/* My intention is to enter */
  turn = j;				/* Set to his turn */
  while (flag[j] && turn == j);	/* wait only if */
  // critical section
  flag[i] = false;
  // remainder section
} while (1);
```

- 세 가지 조건을 모두 충족
- 두 프로세스에 대한 Critical section 문제를 해결
- **Busy waiting(=spin lock, 계속 CPU와 memory를 쓰면서 waiting)**



## Synchronization Hardware

하드웨어적으로 test & modify를 *atomic*하게 수행할 수 있게 instruction을 지원하는 경우 앞의 문제는 간단하게 해결

### test_and_set(a)

1. READ a
2. SET a TRUE

### Mutual exclusion with Test & Set

```c
// Synchronization variable
boolean lock = false;

// Process Pi
do {
  while (test_and_set(lock));
  // critical section
  lock = false;
  // remainder section
} while (1);
```



## Semaphores

> 앞의 방식들을 추상화시킨 자료형

### Semaphore `S`

- integer variable

- 아래 두 가지 atomic 연산에 의해서만 접근 가능

  > If positive, decrement & enter.
  >
  > Otherwise, wait until positive. (busy-wait)

  ```c
  // P(S): 자원을 획득하는 연산; Lock을 거는 연산
  while (S <= 0) do no-op;	// wait
  S--;
  
  // V(S): 자원을 반납하는 연산; Lock을 푸는 연산
  S++;
  ```



## Critical Section of *n* Processes

```c
// Synchronization variable
semaphore mutex;	/* initially 1: 1개의 프로세스가 CS에 들어갈 수 있다. */

// Process Pi
do {
  P(mutex);		/* if positive, dec-&-enter. Otherwise, wait */
  // critical section
  V(mutex);		/* increment semaphore */
  // remainder section
} while (1);
```

- busy-wait는 효율적이지 못 함(=spin lock)
- Block & wake-up 방식의 구현(=sleep lock)



## Block & Wake-up Implementation

### Semaphore를 다음과 같이 정의

```c
typedef struck
{
  int value;					/* semaphore */
  struct process *L;	/* process wait queue */
} semaphore;
```

### block과 wakeup을 다음과 같이 가정

#### block

- 커널은 block을 호출한 프로세스를 suspend시킴
- 이 프로세스의 PCB를 semaphore에 대한 wait queue에 넣음

#### wakeup(p)

- block된 프로세스 P를 wakeup시킴
- 이 프로세스의 PCB를 ready queue로 옮김

### Semaphore 연산이 다음과 같이 정의됨

```c
// P(S)
S.value--;				/* prepare to enter */
if (S.value < 0)	/* Oops, negative. I cannot enter */
{
  // add this process to S.L;
  block();
}

// V(S)
S.value++;
if (S.value <=0)
{
  // remove a process P from S.L;
  wakeup(P);
}
```



## Which is better?

### Busy-wait vs. Block/Wakeup

- **Block/Wakeup 방식**이 CPU를 쓰면서 기다릴 필요 없이 자원을 누군가가 가지고 있으면 CPU를 반납하고 Blocked 상태로 변경하는 것이 전체적으로 CPU를 의미있게 이용하게 된다.

### Block/Wakeup overhead vs. Critical Section 길이

- Critical Section의 길이가 긴 경우 Block/Wakeup이 적당
- Critical Section의 길이가 매우 짧은 경우 Block/Wakeup 오버헤드가 Busy-wait 오버헤드보다 더 커질 수 있음
- 일반적으로는 Block/wakeup 방식이 더 좋음



## Two types of Semaphores

### Counting Semaphore

- 도메인이 0 이상인 임의의 정수값
- 주로 resource counting에 사용

### Binary Semaphore(=mutex)

- 0 또는 1의 값만 가질 수 있는 semaphore
- 주로 mutual exclusion(lock/unlock)에 사용



## Deadlock & Starvation

### Deadlock

둘 이상의 프로세스가 서로 상대방에 의해 충족될 수 있는 event를 무한히 기다리는 현상

```
S와 Q가 1로 초기화된 semaphore라 하자.

	P0			P1
	p(S)		P(Q)		// 하나씩 차지
	p(Q)		P(Q)		// 상대방 것을 요구
	...			...
	v(S)		v(Q)		// 여기와야 release
	V(Q)		v(S)
```



### Starvation

indefinite blocking. 프로세스가 suspend된 이유에 해당하는 세마포어 큐에서 빠져나갈 수 없는 현상





## Classical Problems of Synchronization

### Bounded-Buffer Problem (Producer-Consumer Problem)

> 버퍼의 크기가 유한한 상황에서 다수의 Producer와 Consumer 존재

#### Producer

1. Empty 버퍼가 있나요? (없으면 기다림)
2. 공유 데이터에 lock을 건다
3. Empty 버퍼에 데이터 입력 및 버퍼 조작
4. Lock을 푼다
5. Full 버퍼 하나 증가

#### Consumer

1. Full 버퍼가 있나요? (없으면 기다림)
2. 공유 데이터에 lock을 건다
3. Full 버퍼에 데이터를 꺼내고 버퍼 조작
4. Lock을 푼다
5. Empty 버퍼 하나 증가

#### Shared data

버퍼 자체 및 버퍼 조작 변수(Empty/Full 버퍼의 시작 위치)

#### Pseudocode

```c
// Synchronization variables
semaphore full = 0, empty = n, mutex = 1;

// Producer
do {
  // produce an item in x
  P(empty);	// wait until the buffer gets empty space by the consumer
  P(mutex);
  // add x to buffer
  V(mutex);
  V(full);
} while(1);

// Consumer
do {
  P(full);	// wait until the buffer get an item in buffer by the producer
  P(mutex);
  // remove an item from buffer to y
  V(mutex);
  V(empty);
  // consume the item in y
} while(1);
```

- Mutual exclusion
  - Need binary semaphore(shared data의 mutual exclusiondmf dnlgo)
- Resource count
  - Need integer semaphore(남은 Full/Empty 버퍼의 수 표시)



### Readers and Writers Problem

- 어떤 프로세스가 DB에 **write** 중일 때 다른 프로세스가 접근하면 안 됨
- read는 동시에 여럿이 해도 됨

#### Pseudocode

```c
// Shared data
int readcount = 0;	// 현재 DB에 접근 중인 Reader의 수
DB; // Database 자체
// Synchronization variables
semaphore mutex = 1, db = 1;
/*
mutex: 공유 변수 readcount를 접근하는 코드(critical section)의 mutual exclusion 보장을 위해 사용
db: Reader와 Writer가 공유 DB 자체를 올바르게 접근하게 하는 역할
*/

// Writer
P(db);
// writing DB is performed
V(db);

// Reader
P(mutex);
readcount++;
if (readcount == 1) P(db);	// block writer
V(mutex);	// readers follow
// reading DB is performed
P(mutex);
readcount--;
if (readcount == 0) V(db);	// enable writer
V(mutex);

/* Starvation 발생 가능 => 동시 접근 가능한 Reader의 수를 제한하여 해결 */
```

#### Solution

- Writer가 DB에 접근 허가를 아직 얻지 못한 상태에서는 모든 대기 중인 Reader들을 DB에 접근하게 해준다.
- Writer는 DB에 대기 중인 Reader가 없을 때 DB 접근이 허용된다.
- 일단 Writer가 DB에 접근 중이면 Reader들은 DB 접근이 금지된다.
- Writer가 DB에서 빠져나가야만 Reader의 접근이 허용된다.



### Dining-Philosophers Problem

#### Pseudocode

```c
// Synchronization variables
semaphore chopstick[5];
/* Initially all values are 1 */

// Philosopher i
do {
  P(chopstick[i]);
  P(chopstick[(i + 1) % 5]);
  // eat();
  V(chopstick[i]);
  V(chopstick[(i + 1) % 5]);
  // think();
} while(1);
```

#### 앞 Solution의 문제점

- Deadlock
  - 모든 철학자가 동시에 배고파져서 왼쪽 젓가락을 점유해버린 경우

#### 해결방안

- 4명의 철학자만이 테이블에 동시에 앉을 수 있도록 한다.
- 젓가락을 두 개 모두 집을 수 있을 때에만 젓가락을 점유할 수 있게 한다.
- 비대칭
  - 홀수(짝수) 철학자는 오른쪽(왼쪽) 젓가락부터 집게 한다.



## Monitor

### Semaphore의 문제점

- 코딩하기 힘들다
- 정확성(correctness)의 입증이 어렵다
- 자발적 협력(voluntary cooperation)이 필요하다
- 한 번의 실수가 모든 시스템에 치명적 영향을 끼친다

### 예시

#### Mutual exclusion 깨짐

```c
V(mutex);
// Critical section
P(mutex);
```

#### Deadlock

```c
P(mutex);
// Critical section
P(mutex);
```



### Monitor?

동시 수행 중인 프로세스 사이에서 abstract data type의 안전한 공유를 보장하기 위한 high-level synchronization construct

```c
monitor monitor-name
{
  // shared variable declarations
  procedure body P1(...) {...}
  procedure body P2(...) {...}
  procedure body Pn(...) {...}
  {
    // initialization code
  }
}
```

- 모니터 내에서는 한 번에 하나의 프로세스만이 활동 가능

- 프로그래머가 동기화 제약 조건을 명시적으로 코딩할 필요가 없음

  - shared data에 대한 동시 접근은 monitor가 막아줌

- 프로세스가 모니터 안에서 기다릴 수 있도록 하기 위해 **condition variable** 사용

  - `condition x, y;`

- Condition variable은 **wait**와 **signal** 연산에 의해서만 접근 가능

  - `x.wait();`

    `x.wait()` 을 invoke한 프로세스는 다른 프로세스가 `x.signal()` 을 invoke하기 전까지 suspend된다.

  - `x.signal();`

    `x.signal()` 은 정확하게 하나의 suspend된 프로세스를 resume한다. suspend된 프로세스가 없으면 아무 일도 일어나지 않는다.



### Bounded-Buffer Problem

```c
monitor bounded_buffer
{
  int buffer[N];
  condition full, empty;
  /* condition variable은 값을 가지지 않고 
  자신의 큐에 프로세스를 매달아서 sleep 시키거나 
  큐에서 프로세스를 깨우는 역할만 함 */
  
  void produce (int x)
  {
    // if there is no empty buffer
    empty.wait();
    // add x to an empty buffer
    full.signal();
  }
  
  void consume (int *x)
  {
    // if there is no full buffer
    full.wait();
    // remove an item from buffer and store it to *x
    empty.signal();
  }
}
```

> Monitor와 Semaphore의 차이점?



### Dining Philosophers

```c
monitor dining_philoshopher
{
  enum { thinking, hungry, eating } state[5];
  condition self[5];
  
  void pickup(int i)
  {
    state[i] = hungry;
    test(i);
    if (state[i] != eating) self[i].wait();	// wait here
  }
  
  void putdown(int i)
  {
    state[i] = thinking;
    /* test left and right neighbors */
    test((i+4) % 5);	// if L is waiting
    test((i+1) % 5);	// if R is waiting
  }
  
  void test(int i)
  {
    if ((state[(i+4) % 5]) != eating) && 
      (state[i] == hungry) && 
      (state[(i+1) % 5] != eating))
    {
      state[i] = eating;
      self[i].signal();	// wake up Pi
    }
  }
  
  void init()
  {
    for (int i=0; i<5; i++) state[i] = thinking
  }
}

// Each Philosopher:
{
  pickup(i);	// Enter monitor
  eat();
  putdown(i);	// Enter monitor
  think();
} while(1);
```

