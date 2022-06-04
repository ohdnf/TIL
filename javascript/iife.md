# IIFE

- IIFE(Immediately Invoked Function Expression, 즉시 실행 함수 표현)는 정의되고 나서 곧바로 실행되는 JavaScript 함수 디자인 패턴이다. (용어는 [이 블로그](https://web.archive.org/web/20171201033208/http://benalman.com/news/2010/11/immediately-invoked-function-expression/#iife)에서 처음 쓰였다고 한다.)

```js
// IIFE
(function () {
  /* ... */
})();

// Arrow function IIFE
(() => {
  /* ... */
})();

// async IIFE
(async () => {
  /* ... */
})();
```

- Self-Executing Anonymous Function이라고도 불리는 IIFE는 두 괄호로 구성되어 있다.
  1. 첫 번째 괄호(`()`, grouping operator) 내에는 lexical scope를 가지는 익명 함수가 있다. 이는 IIFE 내 변수에 접근하는 것을 막아 전역 스코프에 불필요한 변수들이 추가되는 것을 방지한다.
  2. 두 번째 괄호는 IIFE가 JavaScript 엔진으로 하여금 즉시 함수로서 해석되고 실행되도록 한다.

## Use cases

### Avoid polluting the global namespace

재사용하지 않는 초기화 코드 같은 경우 IIFE 패턴을 사용하면 전역 네임스페이스에 쓸데없는 변수들이 저장되지 않도록 관리하는데 도움이 된다.

```js
(() => {
  // some initiation code
  let firstVariable;
  let secondVariable;
})();

// firstVariable과 secondVariable은 IIFE가 실행되고 나면 바로 삭제된다.
```

### Execute an async function

`async` IIFE를 사용하면 [`top level await`](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Operators/await#top_level_await)를 사용할 수 있다.

```js
const getFileStream = async (url) => {
  /* implementation */
};

(async () => {
  const stream = await getFileStream("https://domain.name/path/file.ext");
  for await (const chunk of stream) {
    console.log({ chunk });
  }
})();
```

### The module pattern

```js
const makeWithdraw = (balance) =>
  ((copyBalance) => {
    let balance = copyBalance; // this variable is private
    const doBadThings = () => {
      console.log("I will do bad things with your money");
    };
    doBadThings();
    return {
      withdraw(amount) {
        if (balance >= amount) {
          balance -= amount;
          return balance;
        }
        return "Insufficient money";
      },
    };
  })(balance);

const firstAccount = makeWithdraw(100); // "I will do bad things with your money"
console.log(firstAccount.balance); // undefined
console.log(firstAccount.withdraw(20)); // 80
console.log(firstAccount.withdraw(30)); // 50
console.log(firstAccount.doBadThings); // undefined; this method is private
const secondAccount = makeWithdraw(20); // "I will do bad things with your money"
console.log(secondAccount.withdraw(30)); // "Insufficient money"
```

### ES6 이전 `var`를 사용한 For loop

- 두 개의 버튼, 누르면 `0`을 팝업 창에 보여주고 `0`이 써져있는 버튼과 누르면 `1`을 팝업 창에 보여주고 `1`이 써져있는 버튼을 만들고자 한다.
- 아래 코드는 원하는 바와 같이 작동하지 않는다.

```js
for (var i = 0; i < 2; i++) {
  const button = document.createElement("button");
  button.innerText = "Button " + i;
  button.onclick = function () {
    console.log(i);
  };
  document.body.appendChild(button);
}
console.log(i); // 2
```

- `i`가 전역 변수이기 때문에 마지막 값인 2가 표시된다. 이 문제를 해결하기 위해선 아래처럼 IIFE 패턴을 사용해야 한다.

```js
for (var i = 0; i < 2; i++) {
  const button = document.createElement("button");
  button.innerText = "Button " + i;
  button.onclick = (function (copyOfI) {
    return () => {
      console.log(copyOfI);
    };
  })(i);
  document.body.appendChild(button);
}
console.log(i); // 2
```

- `let` 선언자를 사용해 아래와 같이 간단하게 작성할 수도 있다.

```js
for (let i = 0; i < 2; i++) {
  const button = document.createElement("button");
  button.innerText = "Button " + i;
  button.onclick = function () {
    console.log(i);
  };
  document.body.appendChild(button);
}
console.log(i); // Uncaught ReferenceError: i is not defined.
```

## Reference

- [MDN Web Docs](https://developer.mozilla.org/en-US/docs/Glossary/IIFE)
