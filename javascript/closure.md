https://developer.mozilla.org/ko/docs/Web/JavaScript/Closures

# Closure

함수와 함수의 Lexical scoping의 조합

## Lexical scoping

```js
function init() {
  var name = "ohdnf";
  function displayName() {
    // 중첩된 내부 함수(`displayName()`)는 외부 함수(`init()`)의 변수에 접근할 수 있다.
    console.log(name);
  }
  displayName();
}
init();
```

## Closure

```js
function makeFunc() {
  var name = "ohdnf";
  function displayName() {
    console.log(name);
  }
  return displayName;
}

var myFunc = makeFunc();
myFunc();
```

- 위 코드가 바로 그 전 예시 코드와 다른 점은 `displayName()` 함수가 외부 함수에 의해 반환된다는 점이다.
- `makeFunc()`를 호출해 반환된 `displayName()`을 `myFunc`에 할당하고 난 뒤에도 여전히 `name` 변수에 접근할 수 있을까?
- JavaScript의 함수는 *Closure*라고 하는 함수와 함수의 lexical environment의 조합을 형성한다.
- lexical environment는 closure가 생성될 때의 지역 변수를 가지고 있다.
- `displayName` 인스턴스는 `name` 변수가 존재하는 lexical environment에 대한 참조를 유지하여, `myFunc`가 호출될 때 `name` 변수에 접근할 수 있게 한다.

```js
function makeAdder(x) {
  return function (y) {
    return x + y;
  };
}

var add5 = makeAdder(5);
var add10 = makeAdder(10);

console.log(add5(2)); // 7
console.log(add10(2)); // 12
```

- `add5`와 `add10`은 모두 closure이다.
- 둘 다 함수 선언부는 공유하지만, 서로 다른 lexical environment를 저장한다.
- `add5`의 lexical environment에서 `x`는 5이고, `add10`의 lexical environment에서 `x`는 10이다.

## Emulating private methods with closures

- private method는 같은 클래스 내에 존재하는 다른 method에 의해서만 호출될 수 있는 method를 말한다.
- private method는 코드 접근 제한뿐 아니라 전역 네임스페이스 관리에도 유용하다.

```js
var counter = (function () {
  var privateCounter = 0;
  function changeBy(val) {
    privateCounter += val;
  }
  return {
    increment: function () {
      changeBy(1);
    },
    decrement: function () {
      changeBy(-1);
    },
    value: function () {
      return privateCounter;
    },
  };
})();

console.log(counter.value()); // 0

counter.increment();
counter.increment();
console.log(counter.value()); // 2

counter.decrement();
console.log(counter.value()); // 1
```

- `counter.increment`, `counter.decrement`, `counter.value`는 `increment`, `decrement`, `value` 함수들이 공유하고 있는 lexical environment이다.
- 공유 중인 lexical environment는 IIFE로 생성되었는데, 두 private 속성을 포함하고 있다: `privateCounter`, `changeBy`
- IIFE 외부에서 두 속성에 접근하는 것은 불가능하다. 대신 위 세 가지 퍼블릭 함수를 통해 간접적으로 접근이 가능하다.

## Closure Scope Chain

모든 closure는 세 가지 scope를 가진다.

- Local Scope
- Outer Functions Scope
- Global Scope

```js
// global scope
var e = 10;
function sum(a) {
  return function (b) {
    return function (c) {
      // outer functions scope
      return function (d) {
        // local scope
        return a + b + c + d + e;
      };
    };
  };
}

console.log(sum(1)(2)(3)(4)); // log 20
```

- 중첩 함수의 경우 내부 함수는 모든 외부 함수의 scope에 접근이 가능하다.

## Performance considerations

- closure가 필요하지 않음에도 함수 내부에 함수를 생성하는 것은 메모리 소모, 프로세싱 속도 등 성능적 측면에서 바람직하지 않다.
- 새로운 object나 class를 만들 때 method는 보통 object의 constructor보다는 prototype에 붙어야 한다. constructor가 호출될 때마다 method가 재할당되기 때문이다.

```js
function MyObject(name, message) {
  this.name = name.toString();
  this.message = message.toString();
  this.getName = function () {
    return this.name;
  };

  this.getMessage = function () {
    return this.message;
  };
}
```

- 위 코드는 closure의 이점을 전혀 사용하고 있지 않기 때문에 아래와 같이 재작성한다.

```js
function MyObject(name, message) {
  this.name = name.toString();
  this.message = message.toString();
}
MyObject.prototype = {
  getName: function () {
    return this.name;
  },
  getMessage: function () {
    return this.message;
  },
};
```

- prototype을 재정의하는 것은 권장되지 않는다. 대신 기존 prototype에 method들을 추가해보도록 하자.

```js
function MyObject(name, message) {
  this.name = name.toString();
  this.message = message.toString();
}
MyObject.prototype.getName = function () {
  return this.name;
};
MyObject.prototype.getMessage = function () {
  return this.message;
};
```
