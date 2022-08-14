# The callback pattern

- 리액터 패턴의 핸들러를 실체화한 것이 콜백(callback)이다. 콜백은 Node.js의 특징 중 하나이다. 콜백은 작업 결과를 전파하도록 호출되는 함수여서 비동기 처리에 적합하다.
- 동기적인 `return` 명령어를 대신한다.
- JavaScript는 콜백을 대표하는 언어 중 하나인데, 그 이유는 함수가
  - 일급 객체이며,
  - 변수에 할당 가능하고,
  - 인자를 넘길 수 있으며,
  - 다른 함수 호출로부터 반환되고
  - 자료 구조에 저장될 수 있기 때문이다.
- 또한 **클로저(closure)**는 콜백을 구현하기 위한 이상적인 구조체다. 클로저를 사용해 우리는
  - 함수가 생성된 시점의 환경을 참조할 수 있고,
  - 콜백 호출 시점과 관계없이 비동기 작업이 요청된 컨텍스트를 유지할 수 있다.

## The continuation-passing style

- JavaScript에서 callback이란 작업이 끝났을 때 그 결과와 함께 다른 함수에 인자로 넘겨지는 함수를 말한다.
- 함수형 프로그래밍에서 이렇게 결과를 전파하는 방식을 continuation-passing style(CPS)라고 칭한다.
- 무조건 비동기적인 것은 아니며, 호출자에게 결과를 직접 반환하는 대신 콜백 함수에 전달하는 것을 가리킨다.

### Synchronous continuation-passing style

- 컨셉을 이해하기 위해 아래 간단한 동기 함수를 살펴보자.

```js
function add(a, b) {
  return a + b;
}
```

- `return`을 사용해 곧바로 호출자에게 결과를 반환하였다.
- 이를 direct style이라고도 부르며 대부분의 동기적 프로그래밍이 이러한 방식을 따른다.
- CPS로는 아래와 같이 쓸 수 있다.

```js
function add(a, b, callback) {
  callback(a + b);
}
```

- `add()` 함수는 동기적 CPS 함수다.
- 콜백 함수가 실행되어야 `add()` 함수의 결과를 반환할 수 있디.

```js
console.log('before');
add(1, 2, function (result) {
  console.log('Result: ' + result);
});
console.log('after');
```

- `add()`가 동기적이므로 결과는 아래와 같이 순차적으로 처리된다.

```sh
before
Result: 3
after
```

### Asynchronous continuation-passing style

- `add()` 함수가 비동기적인 경우를 생각해보자.

```js
function addAsync(a, b, callback) {
  setTimeout(function () {
    callback(a + b);
  }, 100);
}
```

- `setTimeout()`을 사용해 콜백 함수를 비동기적으로 호출하였다. 이제 아래 코드에서 작업 순서를 살펴보자.

```js
console.log('before');
addAsync(1, 2, function (result) {
  console.log('Result: ' + result);
});
console.log('after');
```

- 결과는 아래와 같다.

```sh
before
after
Result: 3
```

- `setTimeout()`이 비동기적이기 때문에, 콜백 함수가 실행되길 기다리지 않고 즉시 `addAsync()`에게 제어권을 넘겨준다. `addAsync()` 역시 바로 호출자에게 제어권을 넘겨준다.
- 이 방식은 Node.js에서 굉장히 중요한데, 비동기 요청이 보내지자마자 스택이 차례로 되감기되고 제어권이 이벤트 루프로 넘어가는 방식 덕에 큐에 존재하는 새로운 이벤트가 처리될 수 있기 때문이다.

![flow-of-asynchronous-continuation-passing-style](../../assets/ndp-asynchronous-continuation-passing-style.png)

- 비동기 작업이 완료되면 비동기 함수에 있었던 콜백 함수가 다시 실행된다.
- 이벤트 루프에서 시작되므로 스택은 비어있다.
- JavaScript에는 closure가 있어 비동기 함수 호출자의 컨텍스트를 언제든 유지할 수 있다.

> 동기 함수는 작업이 완료될 때까지 기다린다. 비동기 함수는 즉시 반환되고 결과는 이벤트 루프의 이후 사이클에서 핸들러(콜백 함수)에게 넘겨진다.

### Non continuation-passing style callbacks

- `Array` 객체의 `map()` 함수의 경우 비동기 호출도, CPS도 아니다.

```js
var result = [1, 5, 7].map(function (element) {
  return element - 1;
});
```

- 위 콜백 함수는 단순히 배열 내 원소를 순회하기 위해서 사용되었다.
- 콜백 함수의 의도는 보통 API 문서에 설명되어 있으니 확인하자.

## Synchronous or asynchronous?

- 동기 함수인지 비동기 함수인지에 따라 명령어 처리 순서는 달라진다.
- 이는 전체 애플리케이션 흐름에 정확성과 효율성 측면에서 큰 영향을 준다.
- API를 비일관적이고 이해하기 어렵게 만드는 것은 향후 발생할 문제를 파악하기 어렵게 하고 재성성하기에도 어렵다는 점에서 지양해야 한다.

### An unpredictable function

- API가 어떤 조건에서는 동기적으로, 또 다른 조건에서는 비동기적으로 작동하는 건 가장 위험한 상황 중 하나다.

```js
var fs = require('fs');
var cache = {};
function inconsistentRead(filename, callback) {
  if (cache[filename]) {
    // invoked synchronously
    callback(cache[filename]);
  } else {
    // asynchronous function
    fs.readFile(filename, 'utf8', function (err, data) {
      cache[filename] = data;
      callback(data);
    });
  }
}
```

- 위 함수는 `cache` 변수에 서로 다른 파일 읽기 작업 결과를 저장하고 있다.
- 이 함수가 위험한 이유는 어떤 파일이 캐싱되어 있지 않을 경우(`fs.readFile()`의 결과를 반환받기 전까지) 비동기적으로 작동하면서
- 동시에 또다른 요청에 대해선 파일이 이미 캐싱되어 있을 경우 바로 콜백 함수를 호출하기 때문이다.

### Unleashing Zalgo

- 이제 위와 같이 예측할 수 없는 함수가 어떻게 애플리케이션을 망가뜨리는지 살펴보자.

```js
function createFileReader(filename) {
  var listeners = [];
  inconsistentRead(filename, function (value) {
    listeners.forEach(function (listener) {
      listener(value);
    });
  });

  return {
    onDataReady: function (listener) {
      listeners.push(listener);
    },
  };
}
```

- 위 함수가 호출되면, 파일 읽기 작업을 위한 다중 리스너(listener)를 설정할 수 있는 알림 객체(notifier) 같은 것을 생성한다.
- 모든 리스너들은 읽기 작업 완료 후 데이터가 가용되면 한 번에 호출 된다.

```js
var reader1 = createFileReader('data.txt');
reader1.onDataReady(function (data) {
  console.log('First call data: ' + data);
  // ...sometime later we try to read again from the same file
  var reader2 = createFileReader('data.txt');
  reader2.onDataReady(function (data) {
    console.log('Second call data: ' + data);
  });
});
```

- 위 코드는 아래와 같은 결과를 출력한다.

```js
First call data: some data
```

- 보다시피, 두 번째 작업은 절대 호출되지 않는다. 이유를 알아보자:
  - `reader1`이 생성되는 동안 `inconsistentRead()` 함수는 캐싱된 결과가 없기 때문에 비동기적으로 동작한다. 이 경우 리스너를 등록할 시간이 충분한데, 읽기 작업이 완료되는대로 이벤트 루프의 다른 사이클에서 리스너들이 호출되기 때문이다.
  - 다음으로 `reader2`는 요청한 파일의 캐시가 존재하는 상태의 이벤트 루프에서 생성된다. 이 경우, `inconsistentRead()`의 내부 호출은 동기적이다. 따라서 콜백 함수가 즉시 호출되는데, 이는 `reader2`의 리스너들 또한 동기적으로 호출된다는 뜻이다. 하지만 리스너들은 `reader2` 생성 이후 등록되기 때문에 호출될 수 없다.
- `inconsistentRead()` 함수의 콜백 호출 방식은 호출 빈도, 인자로 넘겨지는 파일 이름, 파일을 불러오는 데 걸리는 시간 등 여러 가지 변수로 인해 굉장히 예측하기 어렵다.
- 실제 애플리케이션에서 이와 같은 버그를 찾아내고 재생산하는 것은 대단히 어려울 것이다. 웹 서버에서 단순한 함수를 쓰더라도 다중 동시 요청이 들어왔을 때 어떤 요청들은 아무런 이유 없이 또는 에러도 출력되지 않은 채 처리되지 않을 때가 있다. 이는 분명 더러운 결함이다.
- `npm` 개발자이자 전 Node.js 프로젝트 리드인 Isaac Z. Schlueter는 [그의 블로그](http://blog.izs.me/post/59142742143/designing-apis-for-asynchrony)에서 이런 예측 불가능한 함수를 쓰는 것을 *unleashing Zalgo*에 비유했다.

### Using synchronous APIs

- unleashing zalgo 예제에서 얻을 수 있는 교훈은, API가 동기적인지 비동기적인지 그 속성을 확실하게 정의해야 한다는 것이다.
- `inconsistentRead()` 함수는 동기적으로 만들면 고칠 수 있다. 이는 Node.js가 기본적인 I/O 작업에 대해 동기적 Direct 스타일의 API를 제공(예: `fs.readFileSync()`)하기 때문이다.

```js
var fs = require('fs');
var cache = {};
function consistentReadSync(filename) {
  if (cache[filename]) {
    return cache[filename];
  } else {
    cache[filename] = fs.readFileSync(filename, 'utf8');
    return cache[filename];
  }
}
```

- 동기적인 경우 CPS 스타일을 고집할 이유가 없다. direct 스타일로 동기적 API를 구현하면 헷갈리지도 않고 성능적인 관점에서도 보다 효율적이다.
- 하지만 비동기 대신 동기적 API를 사용하면 몇 가지 단점이 존재한다:
  - 필요한 기능을 반드시 동기적으로 구현하지 못할 수 있다.
  - 동기적 API는 이벤트 루프를 블로킹하고 동시 요청을 대기시키기 때문에 Node.js 동시성을 사용하지 못하고 전체 애플리케이션을 느리게 만들 수 있다.
- `consistentReadSync()` 함수의 경우 이러한 단점이 살짝 가려지는데, 그 이유는 각 파일명마다 동기 I/O API가 한 번만 호출되며 잇따른 호출에 대해서는 캐시 데이터가 사용되기 때문이다. 제한된 정적 파일에 대해서 `consistentReadSync()`를 사용하는 것은 이벤트 루프에 큰 영향을 주지 않는다.
- 항상 두 대안 중 어떤 방식으로 구현하는 것이 좋을지 고민하는 것이 필요하다.

### Deferred execution

- `inconsistentRead()` 함수를 고치는 또 다른 방법은 완전히 비동기적으로 바꾸는 것이다.
- Node.js의 경우 `process.nextTick()`을 사용해 동기적 콜백 호출을 비동기적으로 만들 수 있다.
- `process.nextTick()`은 콜백함수를 인자로 받아 이벤트 큐의 다른 어떤 I/O 이벤트보다도 가장 앞에 집어넣고 즉시 반환한다. 콜백함수는 이벤트 루프가 다시 돌면 바로 호출된다.
- 아래와 같이 코드를 수정하면 어떤 상황에도 비동기적으로 콜백을 호출한다.

```js
var fs = require('fs');
var cache = {};
function consistentReadAsync(filename, callback) {
  if (cache[filename]) {
    process.nextTick(function () {
      callback(cache[filename]);
    });
  } else {
    // asynchronous function
    fs.readFile(filename, 'utf8', function (err, data) {
      cache[filename] = data;
      callback(data);
    });
  }
}
```

- 실행을 늦추는 다른 API로 `setImmediate()`가 있다. (`process.nextTick()`보다는 느릴 수 있다.)
- `process.nextTick()`과 다르게 `setImmediate()`는 이벤트 큐의 맨 뒤에 집어넣어져 다른 I/O 이벤트가 다 실행되고 나서 실행된다.
- `process.nextTick()`은 미리 스케줄링된 I/O보다도 먼저 실행하기 때문에, 재귀적으로 호출되는 경우 등에 의해 I/O 기아상태를 발생시킬 수 있다.

## Node.js callback conventions

- Node.js에서는 CPS 스타일 API, 콜백에 적용되는 컨벤션이 존재한다.
- Node.js 핵심 API 뿐만 아니라 일반 사용자 모듈이나 애플리케이션에서도 이런 컨벤션이 동일하게 적용된다.
- 비동기 API를 디자인하기 위해서 이러한 컨벤션을 잘 이해해야 한다.

### Callbacks come last

- 함수 인자로서의 콜백은 항상 마지막에 위치해야 한다.

```js
fs.readFile(filename, [options], callback);
```

- 선행 함수 시그니처에서 콜백은 옵션 인자보다도 뒤에 위치한다. 이러한 컨벤션의 배경에는 함수 호출의 가독성을 높이는 데 있다.

### Error comes first

- CPS 함수에서 생성되는 에러는 항상 콜백 함수의 첫 번째 인자여야 한다. 다른 실제 결과들은 두 번째 인자부터 지정한다.
- 에러 없이 작업이 성공되면 첫 번째 인자는 `null`이나 `undefined`가 된다.

```js
fs.readFile('foo.txt', 'utf8', function (err, data) {
  if (err) {
    handleError(err);
  } else {
    processData(data);
  }
});
```

- 에러 존재 유무를 확인하는 것은 좋은 습관이다.
- 에러는 항상 `Error` 타입이어야 한다. 문자열이나 숫자를 에러 객체로 전달하면 안 된다.

### Propagating errors

- 동기적 직접 스타일 함수에서 에러를 전파하기 위해선 `throw` 명령어를 사용한다. 에러는 콜스택에 들어가 처리된다.
- 반대로 비동기적 CPS 스타일 함수에서는 다음 콜백 함수에게 에러를 넘기는데, 전형적인 패턴은 아래와 같다.

```js
var fs = require('fs');
function readJSON(filename, callback) {
  fs.readFile(filename, 'utf8', function (err, data) {
    var parsed;
    if (err) {
      // propagate the error and exit the current function
      return callback(err);
    }

    try {
      // parse the file contents
      parsed = JSON.parse(data);
    } catch (err) {
      // catch parsing errors
      return callback(err);
    }
    // no errors, propagate just the data
    callback(null, parsed);
  });
}
```

- 콜백 함수에 유효한 결과를 넘길 때와 그렇지 않고 에러를 넘길 때 호출 방식의 차이를 비교해보면, 앞서 말했던 것처럼 에러는 첫 번째 인자로, 콜백은 마지막 인자로 넘기고 있다.

### Uncaught exceptions

- 방금 전 `readJSON()` 함수에서 `fs.readFile()` 콜백에 예외 처리를 하기 위해 `JSON.parse()`를 `try-catch` 블록으로 감싼 것을 확인할 수 있다.
- 비동기 콜백에서 에러를 throw하는 것은 예외가 바로 이벤트 루프로 넘어가게 하기 때문에, 다음 콜백으로 전파될 수 없게 된다.
- 이러한 경우 Node.js에서는 애플리케이션이 `stderr` 인터페이스에 에러를 출력하고 종료되도록 하기 때문에 회복할 수 없는 상태가 된다.
- 위 예시에서 `try-catch` 블록을 빼보면 알 수 있다.

```js
var fs = require('fs');
function readJSON(filename, callback) {
  fs.readFile(filename, 'utf8', function (err, data) {
    if (err) {
      // propagate the error and exit the current function
      return callback(err);
    }
    // no errors, propagate just the data
    callback(null, JSON.parse(data));
  });
}
```

- 위 함수에서는 `JSON.parse()`에서 발생하는 이벤트적 예외 처리를 할 수 없다. 아래 코드를 실행하면 결과는 그 아래와 같다.

```js
readJSON('nonJSON.txt', function (err) {
  console.error(err);
});
```

```sh
SyntaxError: Unexpected token h in JSON at position 0
    at JSON.parse (<anonymous>)
    at [...]/nodejs_design_pattern/test.js:9:25
    at FSReqCallback.readFileAfterClose [as oncomplete] (node:internal/fs/read_file_context:68:3)
```

- 스택 추적(stack trace)을 보면 Node 내부 모듈인 `fs`에서 시작되었고, `fs.readFile()` 함수로 (이벤트 루프를 통해) 결과를 반환했음을 알 수 있다.
- 이것은 콜백에서 발생한 예외가 곧바로 이벤트 루프로 들어가서 처리되며 콘솔에 보여진다는 것을 말해준다.
- 또한 `readJSON()`을 `try-catch` 블록으로 감싸도 소용이 없다는 것을 의미하는데, 그 이유는 우리가 호출하고자 하는 콜백 스택과 `try-catch` 블록의 스택이 다르게 작동되기 때문이다. 아래 안티 패턴을 통해 확인해보자.

```js
try {
  readJSON('nonJSON.txt', function (err, result) {
    // ...
  });
} catch (err) {
  console.log('This will not catch the JSON parsing exception');
}
```

- 위 `catch` 구문은 JSON 파싱 예외를 절대 받지 못하는데, 이유는 방금 본 것처럼 비동기 작업을 트리거하는 함수가 아닌 스택에서 예외가 발생한 곳으로 가서 끝나기 때문이다.
- 예외가 이벤트 루프에 도착하면 애플리케이션이 멈춘다고 했지만, 종료되기 전 로깅을 하거나 마지막 처리를 할 수 있는 방법이 있긴 하다.
- Node.js가 프로세스를 종료시키기 전 특별한 이벤트인 `uncaughtException`를 발생시키는데, 아래와 같이 사용할 수 있다.

```js
process.on('uncaughtException', function (err) {
  console.error(
    'This will catch at last the ' + 'JSON parsing exception: ' + err.message
  );
  // without this, the application would continue
  process.exit(1);
});
```

- Uncaught exception는 애플리케이션을 예측할 수 없는 상태로 만들기 때문에 문제를 예상할 수 없게 한다.
- 예를 들어, 불완전한 I/O 요청이 실행될 경우나 closure가 비일관적일 경우다.
- 이 때문에 Uncaught exception을 무조건 받을 수 있도록 설정하길 권장한다.
