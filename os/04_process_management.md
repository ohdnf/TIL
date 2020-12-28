# Process Management

## 목차

- [프로세스 생성(Process Creation)](#프로세스-생성process-creation)
- [프로세스 종료(Process Termination)](#프로세스-종료process-termination)



## 프로세스 생성(Process Creation)

- 부모 프로세스(parent process)가 자식 프로세스(child process) 생성
- 부모 프로세스의 문맥을 복제하여 생성
- Copy-on-write(COW) 기법
  - write가 발생했을 때 그제서야 copy를 한다.
  - 그 이전까지는 부모의 정보를 사용
  - write로 데이터 수정이 발생할 때 복제하여 변경
- 프로세스의 트리(계층 구조) 형성
- 프로세스는 자원을 필요로 함
  - 운영체제로부터 받는다
  - 부모와 공유한다
- 자원의 공유
  - 부모와 자식이 모든 자원을 공유하는 모델
  - 일부를 공유하는 모델
  - 전혀 공유하지 않는 모델(일반적)
- 수행(Execution)
  - 부모와 자식은 공존하며 수행되는 모델
  - 자식이 종료(terminate)될 때까지 부모가 기다리는(wait) 모델
- 주소 공간(Address space)
  - 자식은 부모의 공간을 복사함(binary and OS data)
  - 자식은 그 공간에 새로운 프로그램을 올림
- 유닉스의 예
  - `fork()` 시스템 콜이 새로운 프로세스를 생성
    - 부모를 그대로 복사(OS data except PID + binary)
    - 주소 공간 할당
  - `fork()` 다음에 이어지는 `exec()` 시스템 콜을 통해 새로운 프로그램을 메모리에 올림



## 프로세스 종료(Process Termination)

- 프로세스가 마지막 명령을 수행한 후 운영체제에게 이를 알려줌(`exit`)
  - 자식이 부모에게 output data를 보냄 (via `wait`)
  - 프로세스의 각종 자원들이 운영체제에게 반납됨
- 부모 프로세스가 자식의 수행을 종료시킴(`abort`)
  - 자식이 할당 자원의 한계치를 넘어섬
  - 자식에게 할당된 태스크가 더 이상 필요하지 않음
  - 부모가 종료(exit)하는 경우
    - 운영체제는 부모 프로세스가 종료하는 경우 자식이 더 이상 수행되도록 두지 않는다.
    - 단계적인 종료



## 프로세스와 관련한 시스템 콜

### `fork()` 시스템 콜

프로세스는 `fork()` 시스템 콜에 의해 생성된다.

```c
int main()
{
    int pid;
    pid = fork();
    if (pid == 0)	// this is child
        printf("\n Hello, I am child!\n");
    else if (pid > 0)	// this is parent
        printf("\n Hello, I am parent!\n");
}
```

`fork()`가 발생하면 자식 프로세스는 `fork()` 다음 코드부터 실행한다. 이유는 자식 프로세스가 부모 프로세스의 Program counter를 복제하여 기억하고 있기 때문이다.

- `fork()`의 반환값으로 부모와 자식 프로세스를 구분한다.
  - 부모 프로세스는 양수, 자식 프로세스는 0을 반환값으로 가진다.



### `exec()` 시스템 콜

어떤 프로세스에 새로운 프로그램을 덮어씌우기 위해 사용한다.

```c
int main()
{
    printf("\n I'll run date now...\n");
    execlp("/bin/date", "/bin/date", (char *) 0);
    printf("This line never be printed!");
}
```

아래 C언어 코드 예시는

```c
int main()
{
    printf("1");
    execlp("echo", "echo", "hello", "3", (char *) 0);
    printf("2");
}
```

다음 Linux shell command와 같다.

```shell
$ echo "1"
1
$ echo hello 3
hello 3
```



### `wait()` 시스템 콜

프로세스 A가 `wait()` 시스템 콜을 호출하면

- 커널은 child가 종료될 때까지 프로세스 A를 sleep 시킨다. (block 상태)
- child process가 종료되면 커널은 프로세스 A를 깨운다. (ready 상태)

```c
int main()
{
    int childPID;
    S1;
    
    childPID = fork();
    
    if (childPID == 0)
        // code for child process
    else {
        wait();
    }
    
    s2;
}
```

> 리눅스 쉘 프롬프트에서 어떤 프로그램을 실행시키고 해당 프로그램이 다 끝나서야 다른 프로그램을 실행할 수 있는 구조가 바로 `wait()` 시스템 콜 구조이다.



### `exit()` 시스템 콜

> 프로세스의 종료

#### 자발적 종료

- 마지막 statement 실행 후 `exit()` 시스템콜을 통해
- 프로그램에 명시적으로 적어주지 않아도 `main()` 함수가 리턴되는 위치에 컴파일러가 넣어줌

#### 비자발적 종료

- 부모 프로세스가 자식 프로세스를 강제로 종료시킴
  - 자식 프로세스가 한계치를 넘어서는 자원 요청
  - 자식에게 할당된 태스크가 더 이상 필요하지 않음
- 키보드로 `kill`, `break` 등을 친 경우
- 부모가 종료되는 경우
  - 부모 프로세스가 종료하기 전에 자식들이 먼저 종료됨



## 프로세스 간 협력

### 독립적 프로세스(independent process)

- 프로세스는 각자의 주소 공간을 가지고 수행되므로 원칙적으로 하나의 프로세스는 다른 프로세스의 수행에 영향을 미치지 못 함

### 협력 프로세스(cooperating process)

- 프로세스 협력 메커니즘을 통해 하나의 프로세스가 다른 프로세스의 수행에 영향을 미칠 수 있음

### 프로세스 간 협력 메커니즘(IPC: Interprocess Communication)

- 메시지를 전달하는 방법
  - Message passing
    - 커널을 통해 메시지 전달
- 주소 공간을 공유하는 방법
  - shared memory
    - 서로 다른 프로세스 간에도 일부 주소 공간을 공유하게 하는 shared memory 메커니즘이 있음
    - memory를 share하는 것은 커널의 도움을 받아야 함
  - thread
    - thread는 사실상 하나의 프로세스이므로 프로세스 간 협력으로 보기는 어렵지만 동일한 process를 구성하는 thread들 간에는 주소 공간을 공유하므로 협력이 가능



## Message Passing

### Message system

프로세스 사이에 공유 변수(shared variable)를 일체 사용하지 않고 통신하는 시스템



### Direct Communication

통신하려는 프로세스의 이름을 명시적으로 표시

```
Process P			------->		Process Q
send(Q, message)					receive(P, message)
```



### Indirect Communication

mailbox (또는 port)를 통해 메시지를 간접 전달

```
Process P			---Mailbox(M)-->	Process Q
send(M, message)						receive(M, message)
```



