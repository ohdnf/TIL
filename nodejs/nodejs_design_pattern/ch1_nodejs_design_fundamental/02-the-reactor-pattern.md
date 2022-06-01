# The reactor pattern

## I/O is slow

## Blocking I/O

## Non-blocking I/O

- polling algorithm은 보통 CPU 시간을 많이 잡아먹는다.
- non-blocking I/O의 busy-waiting의 pseudocode

```js
resources = [socketA, socketB, pipeA];
while (!resources.isEmpty()) {
  for (i = 0; i < resources.length; i++) {
    resource = resources[i];
    // try to read
    var data = resource.read();
    if (data === NO_DATA_AVAILABLE) {
      // there is no data to read at the moment
      continue;
    } else if (data === RESOURCE_CLOSED) {
      // the resource was closed, remove it from the list
      resource.remove(i);
    } else {
      // some data was received, process it
      consumeData(data);
    }
  }
}
```

## Event demultiplexing

- Busy-waiting이 non-blocking 처리에 적합하지 않다는 것은 분명하다.
- **Synchronous event demultiplexer(또는 event notification interface)** 메커니즘은 동시성의 non-blocking 리소스를 처리할 수 있게 해준다.

```js
socketA, pipeB;
watchedList.add(socketA, FOR_READ); // 1
watchedList.add(pipeB, FOR_READ);
while (events = demultiplexer.watch(watchedList)) { // 2
  // event loop
  foreach(event in events) { // 3
    // This read will never block and will always return data
    data = event.resource.read();
    if (data === RESOURCE_CLOSED) {
      // the resource was closed, remove it from the watched list
      demultiplexer.unwatch(event.resource);
    } else {
      // some actual data was received, process it
      consumeData(data);
    }
  }
}
```

1. 데이터 구조에 리소스를 특정한 수행(위 예의 경우 읽기)과 함께 추가한다.
2. 리소스를 감시하는 event notifier를 설정한다. 이 호출은 감시하는 리소스들 중 어느 하나를 읽을 수 있을 때까지 동기화되고 블로킹한다. 하나의 리소스를 수행할 수(읽을 수) 있게 되면 event demultiplexer가 호출에서 반환되고 새로운 이벤트들이 처리될 수 있는 상태가 된다.
3. event demultiplexer로부터 반환받은 이벤트들이 각각 처리된다. 이 시점에 각 이벤트에 묶여있는 자원들은 읽을 수 있음이 보장되며 실행 중간 블로킹되지 않는다. 모든 이벤트가 처리되면 새로운 이벤트들이 처리될 수 있을 때까지 event demultiplexer를 다시 블로킹한다. 이것을 **event loop**라고 한다.

- 동기적 이벤트 디멀티플렉서(synchronous event demultiplexer)와 논-블로킹(non-blocking) I/O를 사용한 싱글스레드 애플리케이션에서 동시성 제어는 스레드의 유휴 시간을 줄여준다.

## The reactor pattern

1. 애플리케이션이 이벤트 디멀티플렉서(Event Demultiplexer)에 요청해 새로운 I/O 작업을 생성한다. 생성 시 애플리케이션은 핸들러(handler, Node.js의 경우 callback)를 지정해 작업이 완료될 때 호출될 수 있도록 한다. 이벤트 디멀티플렉서에 요청하는 작업은 논-블로킹 호출이며 애플리케이션에게 즉시 제어를 넘겨준다.
2. I/O 작업이 완료되면 이벤트 디멀티플렉서는 이벤트 큐(Event Queue)에 새로운 이벤트를 핸들러와 함께 추가한다.
3. 이 시점에 이벤트 루프(Event Loop)는 이벤트 큐에 있는 이벤트들을 반복 순회한다.
4. 각 이벤트마다 핸들러가 호출된다.
5. 애플리케이션 코드의 일부인 핸들러는 실행이 완료되는대로 이벤트 루프에 제어권을 넘겨준다(5a). 하지만 핸들러 실행 도중 새로운 비동기 조작이 요청될 경우(5b), 해당 조작은 이벤트 루프에 제어권이 넘어가기 전에 이벤트 디멀티플렉서에 삽입된다(1).
6. 이벤트 큐에 존재하는 모든 이벤트들이 처리되면 이벤트 루프는 이벤트 디멀티플렉서를 블로킹해 또다른 순환을 트리거한다.

> Node.js 애플리케이션은 이벤트 디멀티플렉서에 pending 상태의 작업이 없거나 이벤트 큐에 더 이상 처리할 이벤트가 없을 경우 자동으로 종료된다.

Node.js의 핵심 패턴을 아래와 같이 정리할 수 있다.

- Reactor 패턴은 관찰 중인 리소스들로부터 새로운 이벤트를 받을 때까지 블로킹하고, 각 이벤트를 연관된 핸들러로 처리함으로써 I/O를 처리합니다.

## The non-blocking I/O engine of Node.js - libuv

- 운영체제마다 서로 다른 I/O 작업 처리 방식으로 인해 등장한 추상화된 이벤트 디멀티플렉서
- Node.js가 메이저 OS에 호환되고 서로 다른 리소스에 대해 논-블로킹 작업을 가능하게 도와주는 것이 목적

## The recipe for Node.js

리액터 패턴과 `libuv` 외에 알아야 할 Node.js의 컴포넌트

- `libuv`와 다른 로우레벨 함수를 감싸고 노출시키는 binding들
- V8(Node.js가 빠르고 효율적인 이유 중 하나)
- 고차 Node.js API를 구현한 코어 JavaScript 라이브러리(`node-core`)

```
|-----------------------------------|
| Userland modules and applications |
|-----------------------------------|

               Node.js
|-----------------------------------|
|  Core JavaScript API (node-core)  |
|-----------------------------------|
|             Bindings              |
|-----------------------------------|
|        V8       |      libuv      |
|-----------------------------------|
```
