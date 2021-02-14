# CPU Scheduling



## CPU and I/O Bursts in Program Execution



## CPU-burst Time의 분포

> 여러 종류의 job(=process)이 섞여 있기 때문에 CPU 스케줄링이 필요하다.

- Interactive job에게 적절한 response 제공 요망
- CPU와 I/O 장치 등 시스템 자원을 골고루 효율적으로 사용
- I/O bound job이 CPU bound job보다 대부분의 CPU 시간을 쓰는 것인가?
  - I/O bound job은 인터럽트가 잦아 빈도가 높은 것이지 CPU를 많이 쓰는 것은 아니다. 오히려 짧게 쓴다.
  - CPU Bursts가 짧다 --> I/O 인터럽트가 잦다.



## 프로세스의 특성 분류

### I/O-bound process

- CPU를 잡고 계산하는 시간보다 I/O에 많은 시간이 필요한 job
- *many short CPU bursts*

### CPU-bound process

- 계산 위주의 job
- *few very long CPU bursts*



## CPU Scheduler & Dispatcher

> OS 안의 코드(하드웨어가 아니다)

### CPU Scheduler

- Ready 상태의 프로세스 중에서 이번에 CPU를 줄 프로세스를 고른다.

### Dispatcher

- CPU의 제어권을 CPU scheduler에 의해 선택된 프로세스에게 넘긴다.
- 이 과정을 context switch(문맥 교환)라고 한다.



CPU 스케줄링이 필요한 경우는 프로세스에게 다음과 같은 상태 변화가 있는 경우이다.

1. Running => Blocked
   - I/O 요청하는 시스템 콜
2. Running => Ready
   - 할당시간 만료로 timer interrupt
3. Blocked => Ready
   - I/O 완료 후 인터럽트
4. Terminate



> #### 비선점형 & 선점형
>
> - 1, 4에서의 스케줄링은 **non-preemptive(강제로 빼앗지 않고 자진 반납)**
> - All other scheduling is **preemptive(강제로 빼앗음)**



## Scheduling Criteria(Performance Index/Measure, 성능 척도)



### CPU utilization(이용률)

> keep the **CPU as busy as possible**

### Throughput(처리량)

> **\# of processes** that **complete** their execution per time unit

### Turnaround time(소요시간, 반환시간)

> amount of time to **execute a particular process**

CPU를 사용하러 Ready queue에 들어와서 모든 처리를 끝내고 나갈 때까지 걸린 시간

### Waiting time(대기 시간)

> amount of time a process has been **waiting in the ready queue**

선점형 스케줄링의 경우, CPU를 계속해서 뺏길 수 있다. 이 때마다 프로세스가 다시 Ready queue에 들어가게 되면 대기 시간이 증가한다.

### Response time(응답 시간)

> amount of time it takes **from when a request was submitted until the first response is produced**, not output (for time-sharing environment)

처음으로 CPU를 얻기까지 걸린 시간



## Scheduling Algorithm

### First-Come First-Served(FCFS)

- 먼저 온 프로세스를 먼저 처리
- 비선점형 스케줄링
- Convoy effect: 긴 프로세스 때문에 짧은 프로세스가 기다려야 하는 현상



### Shortest Job First(SJF)

- 각 프로세스의 다음 번 CPU birst time을 가지고 스케줄링에 활용
- CPU burst time이 가장 짧은 프로세스를 제일 먼저 스케줄

#### Two schemes:

##### Non-preemptive

- 일단 CPU를 잡으면 이번 CPU birst가 완료될 때까지 CPU를 선점(preemptive) 당하지 않음

##### Preemptive

- 현재 수행 중인 프로세스의 남은 burst time 보다 더 짧은 CPU burst time을 가지는 새로운 프로세스가 도착하면 CPU를 빼앗김
- 이 방법을 Shortest-Remaining-Time-First(SRTF)라고도 부른다

#### Preemptive SJF is optimal

주어진 프로세스들에 대해 minimum average waiting time을 보장

#### 문제점

##### Starvation

- 극단적으로 짧은 프로세스를 선호하므로, 이런 프로세스들이 무한하다면 그보다 **긴 프로세스는 영원히 수행되지 못 할 수**도 있다.

##### 다음 CPU birst time의 예측

- 다음번 **CPU burst time**을 어떻게 알 수 있는가?
- **추정(estimate)만이 가능**하다.
- 과거의 CPU burst time을 이용해서 추정(**exponential averaging**)
  1. at(n) = actual length of nth CPU burst
  2. pt(n+1) = predicted value for the next CPU burst
  3. a (0 <= a <= 1)
  4. pt(n+1) = a * t(n) + (1-a) * pt(n)
  5. a = 0 --> Recent history does not count
  6. a = 1 --> Only the actual last CPU burst counts



### Priority Scheduling

- A priority number(integer) is associated with each process
- highest priority를 가진 프로세스에게 CPU 할당(smallest integer = highest priority)
  - Preemptive
  - Non-preemptive
- SJF는 일종의 priority scheduling이다
  - **priority = predicted next CPU burst time**
- Problem
  - Starvation: low priority processes may **never execute**
- Solution
  - Aging: as time progresses **increase the priority** of the process



### Round Robin(RR)

- 각 프로세스는 동일한 크기의 할당 시간(**time quantum**)을 가짐(일반적으로 10-100 milliseconds)

- 할당 시간이 지나면 프로세스는 선점(preempted)당하고 ready queue의 제일 뒤에 가서 다시 줄을 선다.

- n개의 프로세스가 ready queue에 있고 할당 시간이 **q time unit**인 경우 각 프로세스는 최대 q time unit 단위로 CPU 시간의 1/n을 얻는다.

  > 어떤 프로세스도 (n-1) * q time unit 이상 기다리지 않는다.

- Performance

  - q large ==> FCFS
  - q small ==> context switch 오버헤드가 커진다.

- 일반적으로 SJF보다 average turnaround time이 길지만 response time은 더 짧다.



### Multilevel Queue

highest priority <- system processes / interactive processes / interactive editing processes / batch processes / student processes -> lowest priority

#### Ready queue를 여러 개로 분할

- foreground(interactive)
- background(batch - no human interaction)

#### 각 큐는 독립적인 스케줄링 알고리즘을 가짐

- foreground - RR
- background - FCFS

#### 큐에 대한 스케줄링 필요

- Fixed priority scheduling
  - server all from foreground then from background
  - possibility of starvation
- Time slice
  - 각 큐에 CPU time을 적절한 비율로 할당
  - Eg., 80% to foreground in RR, 20% to background in FCFS



### Multilevel Feedback Queue

- 프로세스가 다른 큐로 이동 가능
- 에이징(aging)을 이와 같은 방식으로 구현할 수 있다.
- Multilevel-feedback-queue scheduling을 정의하는 파라미터들
  - Queue의 수
  - 각 큐의 scheduling algorithm
  - Process를 상위 큐로 보내는 기준
  - Process를 하위 큐로 내쫓는 기준
  - Process가 CPU 서비스를 받으려 할 때 들어갈 큐를 결정하는 기준



### Multiple-Processor Scheduling

- CPU가 여러 개인 경우 스케줄링은 더욱 복잡해짐
- **Homogeneous processor**인 경우
  - Queue에 한 줄로 세워서 각 프로세서가 알아서 꺼내가게 할 수 있다.
  - 반드시 특정 프로세서에서 수행되어야 하는 프로세스가 있는 경우에는 문제가 더 복잡해짐
- **Load sharing**
  - 일부 프로세서에 job이 몰리지 않도록 부하를 적절히 공유하는 메커니즘 필요
  - 별개의 큐를 두는 방법 vs. 공동 큐를 사용하는 방법
- **Symmetric Multiprocessing (SMP)**
  - 각 프로세서가 각자 알아서 스케줄링 결정
- **Asymmetric multiprocessing**
  - 하나의 프로세서가 시스템 데이터의 접근과 공유를 책임지고 나머지 프로세서는 거기에 따름



### Real-Time Scheduling

#### Hard real-time systems

hard real-time task는 정해진 시간 안에 반드시 끝내도록 스케줄링해야 함

#### Soft real-time computing

soft real-time task는 일반 프로세스에 비해 높은 priority를 갖도록 해야 함



### Thread Scheduling

#### Local Scheduling

User level thread의 경우 사용자 수준의 thread library에 의해 어떤 thread를 스케줄할지 결정

#### Global Scheduling

kernel level thread의 경우 일반 프로세스와 마찬가지로 커널의 단기 스케줄러가 어떤 thread를 스케줄할지 결정





## Algorithm Evaluation

### Queueing models

- **확률 분포**로 주어지는 **arrival rate**와 **service rate** 등을 통해 각종 performance index(throughput) 값을 계산

### Implementation(구현) & Measurement(성능 측정)

- **실제 시스템**에 알고리즘을 **구현**하여 실제 작업(**workload**)에 대해서 성능을 **측정** 비교

### Simulation(모의 실험)

- 알고리즘을 **모의 프로그램**으로 작성 후 **trace**(실제 프로그램을 통해서 추출한 데이터)를 입력으로 하여 결과 비교