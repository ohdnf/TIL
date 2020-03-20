# JavaScript 문법

## 1. 연산자
- 산술 연산자: 덧셈, 뺄셈, 곱셈, 나눗셈, 나머지(`%`) 연산
- 할당 연산자: `+=`, `-=`, `*=`, `/=`, `%=`
- 동등 연산자: `==`
    ```js
    0 == ''     // true
    0 == '0'    // true
    '' == '0'   // false
    ```
- 일치 연산자: `===`
    ```js
    0 === ''
    0 === '0'
    '' === '0'
    ```
- 부등, 불일치 연산자: `!=`, `!==`
- 비교 연산자: `<`, `>`, `<=`, `>=`
- 논리 연산자: `&&`, `||`, `!`
- 기타 연산자

## 2. 조건문과 반복문

### 1) 조건문
- `if`문: `if`, `else if`, `else`
- `switch`문
    ```js
    var s;
    switch(s){
        case:
        break;
    }
    ```

### 2) 반복문
- `for`문
    - `for`
        ```js
        for (i=0; i<10; i++) {
            // do something
        }
        ```
    - `for ... of`
    - `forEach`
    - `for ... in`


## 함수
- JavaScript의 함수는 **일급 객체**이므로 아래와 같은 특징이 있다.
    1. 무명의 리터럴로 표현이 가능하다.
    2. 변수나 자료구조(객체, 배열 등)에 저장할 수 있다.
    3. 함수의 파라미터로 전달할 수 있다.
    4. 반환값(return value)으로 사용할 수 있다.

### 1. 함수 선언문
```js
function square(number) {
    return number * number;
}
```

### 2. 함수 표현식
- 함수 표현식으로 정의한 함수는 함수명을 생략할 수 있다. 이러한 함수를 익명 함수(anonymous function)이라 한다. 함수 표현식에서는 함수명을 생략하는 것이 일반적이다.
    ```js
    var square = function(number) {
        return number * number;
    }
    ```

- 함수가 할당된 변수를 사용해 함수를 호출하지 않고 기명 함수의 함수명을 사용해 호출하게 되면 에러가 발생한다. 이는 함수 표현식에서 사용한 함수명은 외부 코드에서 접근 불가능하기 때문이다.
    ```js
    var foo = function(a, b) {
        return a * b;
    };

    var bar = foo;  // 변수 bar와 foo는 동일한 익명 함수의 참조값을 가짐

    console.log(foo(10, 10)); // 100
    console.log(bar(10, 10)); // 100
    ```

### 3. 즉시 실행 함수
- 최초 한번만 호출되며 다시 호출할 수 없다. 초기화 처리 등에 이용
    ```js
    // 기명 즉시 실행 함수(named immediately-invoked fucntion expression)
    (function myFunc() {
        var a = 3;
        var b = 5;
        return a * b;
    }());

    // 익명 즉시 실행 함수(immediately-invoked function expression)
    (function () {
        var a = 3;
        var b = 5;
        return a * b;
    }());

    // SyntaxError: Unexpected token (
    // 함수선언문은 자바스크립트 엔진에 의해 함수 몸체를 닫는 중괄호 뒤에 ;가 자동 추가된다.
    function () {
        // ...
    }();    // ==> };();

    // 따라서 즉시 실행 함수는 소괄호로 감싸준다.
    (function () {
        // ...
    }());

    (function () {
        // ...
    })();
    ```

### 화살표 함수
- 화살표 함수(Arrow function)는 `function` 키워드 대신 화살표(`=>`)를 사용하여 보다 간략한 방법으로 함수를 선언할 수 있다. 하지만 모든 겨우 화살표 함수를 사용할 수 있는 것은 아니다.
    ```js
    // 매개변수 지정 방법
    () => {...}     // 매개변수가 없을 경우
    x => {...}      // 매개변수가 한 개인 경우, 소괄호 생략 가능
    (x, y) => {...} // 매개변수가 여러 개인 경우, 소괄호 생략 불가

    // 함수 몸체 지정 방법
    x => { return x * x }   // single line block
    x => x * x              // 함수 몸체가 한 줄의 구문이라면 중괄호를 생략할 수 있으며 암묵적으로 return이 된다.

    () => { return { a: 1 }; }
    () => ({ a: 1 })    // 위 표현과 동일. 객체 반환 시 소괄호 사용

    () => {     // multi line block
        const x = 10;
        return x * x;
    };

- 화살표 함수는 익명 함수로만 사용할 수 있다. 따라서 화살표 함수를 호출하기 위해서는 함수 표현식을 사용한다.
    ```js
    // ES5
    var pow = function (x) { return x * x; };
    console.log(pow(10));   // 100

    // ES6
    const pow = x => x * x;
    console.log(pow(10));   // 100
    ```

- `this`: 