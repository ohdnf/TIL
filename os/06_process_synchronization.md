# Process Synchronization

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

#### 문제

- 하나의 프로세스가 critical section에 있을 때 다른 모든 프로세스는 critical section에 들어갈 수 없어야 한다.



## 프로그램적 해결법의 충족 조건

6-1부터

