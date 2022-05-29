# 함수와 함수의 호출, 고차함수

## 함수와 함수의 호출

```js
// 함수의 선언
function func() {
  console.log("this is a function");
}

const func2 = function () {
  console.log("this is a function, too");
};

// 화살표 함수
const arrowFunc = (value) => {
  console.log("this is arrow function");
  return value;
};
```

```js
const add = (a, b) => a + b;
add(3, 5); // 8
const add2 = (a, b) => ({ answer: a + b }); // 객체를 반환할 때는 괄호로 감싸야 합니다.
add2(3, 5); // { answer: 8 }

const calculator = (func, a, b) => {
  return func(a, b);
};

// 함수의 호출값을 인자로 넘겼으므로 잘못된 코드입니다.
// calculator(add(), 3, 5);
// 아래와 같이 함수를 인자로 넘겨야 정상적으로 작동합니다.
calculator(add, 3, 5); // 8
```

```js
import { useCallback } from "react";

export const App = () => {
  const onClick = useCallback((e) => {
    console.log(e.target);
  }, []);

  return <div onClick={onClick}></div>;
};
```

## 고차함수

```js
// 고차함수는 함수를 반환하는 함수입니다.
const onClikc = () => () => {
  console.log("hello world");
};

// `onClick`이 아닌 `onClick()`은 함수 호출이고, 호출 결과값은 함수이므로 아래 코드는 정상입니다.
document.querySelector("#header").addEventListener("click", onClick());
```

## 함수의 호출 스택

```js
const x = "x";

function c() {
  const y = "y";
  console.log("c");
  // debugger;
}

function a() {
  const x = "x";
  function b() {
    const y = "y";
    c();
    console.log("b");
  }
  b();
  console.log("a");
}

a(); // c b a
c(); // c
```

## 호이스팅

```js
// hoist function
console.log(x); // x
function x() {
  console.log("x");
}

// cannot hoist arrow function
console.log(y); // ReferenceError: Cannot access 'y' before initialization
const y = () => {
  console.log("y");
};

// cannot hoist const variable
console.log(z); // ReferenceError: Cannot access 'z' before initialization
const z = "z";
```

```js
function a() {
  console.log(b);
}

const b = "b";
a(); // b
```

```js
function a() {
  console.log(b);
}

a(); // ReferenceError: Cannot access 'b' before initialization
const b = "b";
```

## this

- `this`는 기본적으로 `window`
- JavaScript: window -> globalThis
- Node: global -> globalThis
- ES2015 모듈에서는 자동 strict 모드 적용

```js
console.log(this); // {}

function a() {
  "use strict";
  console.log(this);
}
a(); // undefined
```

- **this는 함수가 호출될 때 정해진다**

```js
const obj = {
  name: "ohdnf",
  sayName() {
    console.log(this.name);
  },
};

obj.sayName(); // ohdnf
```

```js
const obj = {
  name: "ohdnf",
  sayName: function () {
    console.log(this.name);
  },
};

obj.sayName();
```

```js
const name = "test";
const obj = {
  name: "ohdnf",
  sayName: () => {
    console.log(this.name);
  },
};

obj.sayName();
```

- 화살표 함수의 this는 정적이다.

```js
function Counter1() {
  this.value = 0;
  this.add = (amount) => {
    // 화살표 함수의 this는 함수가 생성될 당시의 this를 가리킴
    this.value += amount;
  };
}
// counter 인스턴스를 생성, this는 counter를 가리킴
const counter = new Counter1();
console.log(counter.value); // 0
counter.add(5); // 화살표 함수의 this는 생성될 당시의 this, counter 객체를 가리킴
console.log(counter.value); // 5
const add = counter.add;
add(5);
console.log(counter.value); // 10
```

- 일반 함수의 this는 동적이다.

```js
function Counter2() {
  this.value = 0;
  this.add = function (amount) {
    // 일반 함수의 this는 해당 함수를 호출한 주체를 가리킴
    this.value += amount;
    console.log(this === global);
  };
}

const counter2 = new Counter2();
console.log(counter2.value); // 0
counter2.add(5); // 여기서 this는 counter2를 가리킴
console.log(counter2.value); // 5
const add = counter2.add;
add(5); // 여기서 this는 전역 객체(global)를 가리킴
console.log(counter2.value); // 5
```

- 클래스의 경우도 마찬가지다.

```js
class Counter3() {
  value = 0;
  // 일반함수의 this는 동적으로 결정된다.
  add1(amount) {
    this.value += amount;
  };
  // 화살표함수의 this는 정적으로 결정(여기서는 항상 Counter3 클래스 객체)된다.
  add2 = (amount) => {
    this.value += amount;
  }
}

const counter2 = new Counter2();
console.log(counter2.value); // 0
counter2.add(5); // 여기서 this는 counter2를 가리킴
console.log(counter2.value); // 5
const add = counter2.add;
add(5); // 여기서 this는 전역 객체(global)를 가리킴
console.log(counter2.value); // 5
```

- 함수가 아닌 객체의 this

```js
const counter3 = {
  value: 0,
  add: function (amount) {
    this.value += amount;
  },
};
console.log(counter3.value); // 0
counter3.add(5);
console.log(counter3.value); // 5
const add = counter3.add;
add(5);
console.log(counter3.value); // 5
```

```js

```
