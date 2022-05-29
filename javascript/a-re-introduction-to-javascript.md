# A re-introduction to JavaScript

> MDN 문서의 [JavaScript 재입문하기](https://developer.mozilla.org/ko/docs/A_re-introduction_to_JavaScript)를 정리한 내용입니다.
>
> JavaScript란?
>
> - 유형 및 연산자, 표준 내장 객체 및 메소드가 있는 다중 패러다임, 동적 언어
> - 클래스 대신 **객체 프로토 타입**을 사용하여 객체 지향 프로그래밍을 지원

## 목차

- [Numbers](#numbers)
- [Strings](#strings)
- [이외의 타입들](#이외의-타입들)
  - [Null](#null)
  - [undefined](#undefined)
  - [Boolean](#boolean)
- [Variables](#variables)
- [Operators](#operators)
- [제어 구조](#제어-구조)
- [Objects](#objects)
- [Arrays](#arrays)
- [Functions](#functions)
- [사용자 정의 객체](#사용자-정의-객체)
  - [Inner functions](#inner-functions)
- [Closures](#closures)

## Numbers

> 이중정밀도 64비드 형식 IEEE 754 값
>
> `BigInt` 제외 정수와 같은 수는 존재하지 않습니다.
> *명백한 정수*는 사실 *암묵적 실수*입니다.

```javascript
// Math라는 내장 객체를 활용해 수학 함수와 상수를 다룰 수 있습니다.
console.log(3 / 2); // 1(X) 1.5(O)
console.log(Math.floor(3 / 2)); // 1

// 0.1 + 0.2 = 0.30000000000000004

Math.sin(3.5);
let circumference = 2 * Math.PI * r;

// 문자열을 숫자로 변환
// 두번째 매개변수로 진수를 정할 수 있습니다.
parseInt("123", 10); // 123
parseInt("010", 10); // 10
parseInt("11", 2); // 3
parseInt("0x10"); // 16, 문자열 앞이 0x로 시작하면 16진수로 취급

// 단항 연산자 +를 사용해도 문자열을 숫자로 변환 가능
typeof +"42"; // number

// parsing하는 범위
parseInt("10.2abc"); // 10
parseFloat("10.2abc"); // 10.2
+"10.2abc"; // NaN

// 부동 소수점 숫자는 parseFloat() 사용

// 문자열이 수가 아닌 경우 = NaN
parseInt("hello", 10); // NaN
isNaN(NaN); // true

// 무한대
1 / 0; //  Infinity
-1 / 0; // -Infinity
isFinite(1 / 0); // false
isFinite(-Infinity); // false
isFinite(NaN); // false
```

## Strings

> JavaScript의 문자열은 각각이 16비트 숫자로 표현된 UTF-16 코드 유닛(유니코드 문자)이 길게 이어져 있는 것입니다.

```javascript
// 문자열의 length 속성을 활용해 문자열의 길이를 알 수 있습니다.
"hello".length; // 5

// 문자열도 객체입니다. 다음과 같은 메소드를 활용할 수 있습니다.
"hello".charAt(0); // "h"
"hello, world".replace("hello", "goodbye"); // "goodbye, world"
"hello".toUpperCase(); // "HELLO"
```

## 이외의 타입들

### Null

- 의도적으로 값이 없음을 가리키는 **객체** 타입

### undefined

- 초기화되지 않은 값, 아직 어떤 값도 할당되지 않은 변수임을 가리키는 _정의되지 않은_ **객체** 타입
- JavaScript에서 변수에 값을 주지 않고 선언하는 것이 가능하기 때문에 선언되고 값을 할당받지 않은 변수의 타입은 `undefined`입니다.

### Boolean

#### `false`로 처리되는 값

- `0`
- `""`
- `NaN`
- `null`
- `undefined`

#### `true`로 처리되는 값

- `false`와 위의 값들을 제외한 다른 모든 값

#### 확인 방법

```javascript
Boolean(""); // false
Boolean(234); // true
```

## Variables

> 새로운 변수는 `let`, `const`, `var` 키워드로 선언됩니다.

### `let`

블록 유효 범위 변수(block scope variable)를 선언합니다.

```javascript
// myLetVariable는 여기에서 보이지 *않습니다*

for (let myLetVariable = 0; myLetVariable < 5; myLetVariable++) {
  // myLetVariable는 여기에서 유효합니다
}

// myLetVariable는 여기에서 보이지 *않습니다*
```

### `const`

변경되지 않는 변수를 선언합니다.

```javascript
const Pi = 3.14; // 변수 Pi를 상수로 설정
Pi = 1; // 상수로 설정된 변수는 변경할 수 없기 때문에 에러 발생
```

### `var`

`let`, `const` 키워드가 가지는 제한을 받지 않습니다. 즉, 할당 및 선언이 자유롭고 블록 스코프의 영향도 받지 않습니다.

```javascript
// myVarVariable는 여기에서 사용 할 수 *있습니다*

for (var myVarVariable = 0; myVarVariable < 5; myVarVariable++) {
  // myVarVariable는 함수 전체에서 사용 할 수 있습니다.
}

// myVarVariable는 여기에서 사용 할 수 *있습니다*
```

## Operators

- `+`, `-`, `*`, `/`, `%`는 산술 연산자
- `=`는 할당 연산자
- `++`, `--`는 증감 단항 연산자

```javascript
// 아래 두 식은 동치
x += 5;
x = x + 5;
```

`+` 연산자는 문자열을 이어붙이는 데도 사용됩니다. 문자열에 어떤 수 또는 다른 값을 더하면 문자열로 바뀌게 됩니다.

```javascript
"3" + 4 + 5; // "345"
3 + 4 + "5"; // "75"

// 빈 문자열에 어떤 값을 더해 해당 값을 문자열로 바꿀 수 있습니다.
"" + 1; // "1"
```

### `==` vs. `===`

> 이중 등호 연산자(`==`)는 서로 다른 타입을 줄 경우 타입 강제 변환을 수행합니다.
>
> 타입 강제 변환을 하지 않으려면 삼중 등호 연산자(`===`)를 사용해야 합니다.

```javascript
123 == "123"; // true
1 == true; // true

123 === "123"; // false
1 === true; // false
```

`!=`와 `!==`도 유사한 결과를 내보냅니다.

```javascript
123 != "123"; // false
1 != true; // false

123 !== "123"; // true
1 !== true; // true
```

즉, `===` 연산자나 `!==` 연산자가 보다 더 엄격한 비교를 수행한다고 할 수 있습니다.

### 단축 평가(short-circuit)

> `&&`와 `||` 연산자는 첫번째 식을 평가한 결과에 따라서 두번째 식을 평가하는 단축 평가 논리를 사용합니다. 객체에 접근하기 전에 객체가 null인지 아닌지를 검사하는데 유용하게 쓰입니다.

```javascript
// o가 null이 아니라면 o 객체의 getName 메소드를 실행합니다.
let name = o && o.getName();

// 캐시된 값이 유효하지 않은 값일 때 새로 캐싱할 수 있습니다.
let name = cachedName || (cachedName = getName());

// 삼중 연산자로 조건문을 한 줄에 표현 가능합니다.
let allowed = age > 18 ? "yes" : "no";
```

## 제어 구조

### `if` ~ `else`

```javascript
let name = "kittens";
if (name == "puppies") {
  name += " woof";
} else if (name == "kittens") {
  name += " meow";
} else {
  name += "!";
}
name == "kittens meow"; // true
```

### `while`, `do-while`

```javascript
while (true) {
  // 무한루프!
}

let input;
do {
  input = get_input();
} while (inputIsNotValid(input));
```

### `for`

```javascript
// C, Java의 for 반복문과 동일
for (let i = 0; i < 5; i++) {
  // 내부 동작을 5번 반복합니다
}

// for ... of
for (let value of array) {
  // value로 작업을 실행합니다
}

// for ... in
for (let property in object) {
  // object의 항목(property)으로 작업을 실행합니다
}
```

### `switch`

> 숫자나 문자열을 기반으로 다중 분기되는 문장을 작성하는데 사용됩니다.
>
> `switch` 부분과 `case` 부분의 표현식은 `===` 연산자로 비교됩니다.

```javascript
switch (action) {
  case "draw":
    drawSomething();
    break;
  case "eat":
    eatSomething();
    break;
  default:
    doNothing();
}
```

- `break` 키워드가 없다면 다음 단계로 넘어가게 됩니다. 대부분 `case`문에 넣게 됩니다.
- `default` 구문의 적용은 선택사항입니다.

## Objects

> JavaScript의 객체는 name-value pairs의 모임입니다. 때문에 다음과 비슷합니다.
>
> - Python의 Dictionaries
> - C와 C++의 Hash tables
> - Java의 HashMaps
>
> JavaScript 내 코어 타입들을 제외한 모든 것은 객체로 취급되기 때문에 어떤 JavaScript 프로그램도 기본적으로 해쉬 테이블을 검색하는데 출중한 성능을 가지고 있습니다.
>
> 값(value)은 객체를 포함하여 아무 타입이 될 수 있는 반면, 이름(name)은 문자열입니다.

빈 객체를 생성하는 두 가지 방법

```javascript
// 생성자 함수
let obj = new Object();

// 객체 리터럴 구문
let obj = {};
// 리터럴 구문을 사용해 속성을 초기화할 수 있습니다.
let obj = {
  name: "Carrot",
  for: "Max",
  details: {
    color: "orange",
    size: 12,
  },
};
```

객체의 속성에 접근하는 방법

```javascript
// dot 표기법
obj.details.color; // orange
// bracket 표기법
obj["details"]["size"]; // 12
```

객체 프로토타입(`Person`)과 프로토타입의 인스턴스(`you`)를 생성하는 예제입니다.

```javascript
function Person(name, age) {
  this.name = name;
  this.age = age;
}

let you = new Person("You", 24);

// key와 value를 입력받아 정의하는 방법
let key = prompt("what is your key?"); // sex
you[key] = prompt("what is its value?"); // male
```

## Arrays

```javascript
// 배열 생성 방법 1
let arr = new Array();
arr[0] = "dog";
arr[1] = "cat";
arr[2] = "hen";
arr.length; // 3

// 배열 생성 방법 2
let arr = ["dog", "cat", "hen"];
arr.length; // 3
```

> 배열 리터럴 `[]`의 마지막 원소 끝에 `,`를 남겨두는 것은 브라우저마다 다르게 처리하므로 권장하지 않습니다.

`array.length`는 배열에 들어있는 항목의 수를 반드시 반영하지는 않습니다. 배열의 `length` 속성은 최대 인덱스에 하나를 더한 값일 뿐입니다.

```javascript
> let a = ["dog", "cat", "hen"];
> a[100] = "fox";
> a.length
101
```

존재하지 않는 배열 인덱스를 참조하려고하면 다음과 같이 `undefined`을 얻게 됩니다.

```javascript
> typeof(a[90]);
undefined
```

### 배열 메서드

| 메서드 이름                                            | 설명                                                                                  |
| ------------------------------------------------------ | ------------------------------------------------------------------------------------- |
| `arr.toString()`                                       | 각 항목에 대한 `toString()`의 출력이 콤마로 구분된 한 개의 문자열을 반환합니다.       |
| `arr.toLocaleString()`                                 | 각 항목에 대한 `toLocaleString()`의 출력이 콤마로 구분된 한 개의 문자열을 반환합니다. |
| `arr.concat(item1[, item2[, ...[, itemN]]])`           | item들이 덧붙여진 한 개의 배열을 반환합니다.                                          |
| `arr.join(sep)`                                        | 배열의 값들을 `sep` 인자로 구분하여 합친 한 개의 문자열로 변환합니다.                 |
| `arr.pop()`                                            | 배열의 마지막 항목을 반환하면서 제거합니다.                                           |
| `arr.push(item1, ..., itemN)`                          | 배열의 끝에 item들을 덧붙입니다.                                                      |
| `arr.shift()`                                          | 배열의 첫 번째 항목을 반환하면서 제거합니다.                                          |
| `arr.unshift(item1[, item2[, ...[, itemN]]])`          | 배열의 앞쪽에 item들을 덧붙입니다.                                                    |
| `arr.sort([cmpfn])`                                    | 옵션으로 비교용도의 함수를 입력받습니다.                                              |
| `arr.splice(start, delcount[, item1[, ...[, itemN]]])` | 배열의 일부분을 제거하고 다른 항목으로 대체하여 배열을 변경합니다.                    |
| `arr.reverse()`                                        | 배열의 순서를 거꾸로 배열합니다.                                                      |

### 배열을 `for` 반복문으로 처리하는 방법

#### `for`

```javascript
for (let i = 0; i < a.length; i++) {
  // a[i] 로 뭔가를 수행
}
```

#### `for ... of`

```javascript
for (const currentValue of a) {
  // currentValue 로 뭔가를 수행
}
```

> `for ... in` 루프는 배열 요소를 반복하는게 아니라 배열 인덱스를 반복하기 때문에 배열을 순회하기엔 적합하지 않습니다.

#### `forEach()`

```javascript
['dog', 'cat', 'hen'].forEach(function(currentValue, index, array) {
    // currentValue나 array[index]로 뭔가를 수행
}
```

## Functions

```javascript
function add(x, y) {
  let total = x + y;
  return total; // return문이 없으면 undefined 반환
}

// 매개변수에 전달될 인자가 없다면 해당 변수는 undefined로 설정
add(); // NaN

// 매개변수보다 많은 인자가 넘겨질 경우 무시됩니다.
add(2, 3, 4); // 5
```

매개변수와 상관없이 함수에 넘겨지는 인자들은 `arguments` 객체로 접근할 수 있습니다.

```javascript
function add() {
  let sum = 0;
  for (let i = 0, j = arguments.length; i < j; i++) {
    sum += arguments[i];
  }
  return sum;
}

add(2, 3, 4, 5); // 14
```

Rest 파라미터 연산자는 `...args`와 같은 형식으로 함수 파라미터 목록에 사용되어, 갯수 제한없이 함수로 인자를 전달받습니다. 다음 예에서 `avg`에 전달된 첫번째 값은 `firstVal` 변수에 저장되고 남은 변수들은 `args`에 저장됩니다.

```javascript
function avg(firstVal, ...args) {
  let sum = 0;
  for (let value of args) {
    sum += value;
  }
  return sum / arr.length;
}

avg(2, 3, 4, 5); // 3.5
```

### 익명 함수

```javascript
let avg = function () {
  let sum = 0;
  for (let i = 0, j = arguments.length; i < j; i++) {
    sum += arguments[i];
  }
  return sum / arguments.length;
};
```

### 재귀 호출

JavaScript에서 재귀적으로 함수를 부를 수 있습니다. 브라우저 DOM과 같은 트리 구조를 다루는 데 유용합니다.

```javascript
function countChars(elm) {
  if (elm.nodeType == 3) {
    // TEXT_NODE
    return elm.nodeValue.length;
  }
  let count = 0;
  for (let i = 0, child; (child = elm.childNodes[i]); i++) {
    count += countChars(child);
  }
  return count;
}
```

익명 함수를 재귀적으로 호출할 때는 다음과 같이 이름을 부여할 수 있습니다. 함수 표현식에 제공된 이름은 함수 자체 범위에서만 유효합니다. 이는 엔진 최적화뿐만 아니라 코드 가독성을 높입니다.

```javascript
let charsInBody = (function counter(elm) {
  if (elm.nodeType == 3) {
    // TEXT_NODE
    return elm.nodeValue.length;
  }
  let count = 0;
  for (let i = 0, child; (child = elm.childNodes[i]); i++) {
    count += counter(child);
  }
  return count;
})(document.body);
```

### Arrow function

1. 클래스 정의에서 사용하지 않는다.(메소드 함수가 아닌 곳에서만 사용)
2. 생성자로 사용하지 않는다.
3. 이벤트 리스너 콜백 함수로 사용하지 않는다.

## 사용자 정의 객체

> 고전 객체지향 프로그래밍에서 객체는 데이터와 해당 데이터를 다루는 메소드의 집합이었습니다. JavaScript는 프로토타입 기반 언어로, class 구문이 따로 없습니다. 대신 JavaScript는 function을 class로 사용합니다.

```javascript
function makePerson(first, last) {
  return {
    first: first,
    last: last,
    fullName: function () {
      return this.first + " " + this.last;
    },
    fullNameReversed: function () {
      return this.last + ", " + this.first;
    },
  };
}

let jp = makePerson("Jupyo", "Hong");
jp.fullName(); // "Jupyo Hong"
jp.fullNameReversed(s); // "Hong, Jupyo"
```

`this` 키워드를 사용해서 개선할 수 있습니다.

> We have introduced another keyword: `new`. `new` is strongly related to `this`. It creates a brand new empty object, and then calls the function specified, with `this` set to that new object. Notice though that the function specified with `this` does not return a value but merely modifies the `this` object. It's `new` that returns the `this` object to the calling site. Functions that are designed to be called by `new` are called constructor functions. Common practice is to capitalize these functions as a reminder to call them with `new`.

```javascript
function personFullName() {
  return this.first + " " + this.last;
}
function personFullNameReversed() {
  return this.last + ", " + this.first;
}

function Person(first, last) {
  this.first = first;
  this.last = last;
  this.fullName = personFullName;
  this.fullNameReversed = personFullNameReversed;
}

let jp = new Person("Jupyo", "Hong");
```

[프로토타입 체인](https://developer.mozilla.org/ko/docs/Web/JavaScript/Guide/Inheritance_and_the_prototype_chain)을 사용해 객체에 메소드를 추가할 수도 있습니다. 객체에 설정되지 않은 속성에 접근을 시도하면, JavaScript는 객체의 `prototype`에 그 속성이 존재하는지 살펴봅니다(lookup).

```javascript
function Person(first, last) {
  this.first = first;
  this.last = last;
}
Person.prototype.fullName = function () {
  return this.first + " " + this.last;
};
Person.prototype.fullNameReversed = function () {
  return this.last + ", " + this.first;
};
```

이미 존재하는 객체에 추가적인 메소드를 실시간으로 추가할 수도 있습니다.

```javascript
let jp = new Person("Jupyo", "Hong");
jp.firstNameCaps(); //TypeError on line 1: jp.firstNameCaps is not a function

Person.prototype.firstNameCaps = function () {
  return this.first.toUpperCase();
};
s.firstNameCaps(); // "JUPYO"
```

JavaScript의 빌트인 객체(`Number`, `String` 등)의 `prototype`에도 추가할 수 있습니다. `String` 객체에 문자열 순서를 거꾸로 배열하여 돌려주는 메소드를 추가해봅니다.

```javascript
var jp = "Jupyo";
jp.reversed(); // TypeError on line 1: jp.reversed is not a function

String.prototype.reversed = function () {
  var r = "";
  for (var i = this.length - 1; i >= 0; i--) {
    r += this[i];
  }
  return r;
};

jp.reversed(); // oypuJ
```

문자열 상수에서도 동작합니다.

```javascript
"This can now be reversed".reversed(); // desrever eb won nac sihT
```

프로토타입 체인에서 루트는 `Object.prototype`입니다.

### Inner functions

> 중첩 함수는 함수 내부에서 선언된 함수입니다. 자식 함수는 부모 함수 스코프(scope)의 변수에 접근할 수 있습니다. 한두개 정도의 함수에서만 호출되며 다른 부분에서 사용되지 않는다면 함수 내에 중첩시키는 것이 좋습니다. 전역 범위 함수의 갯수를 늘리지 않도록 하는 것은 좋은 습관입니다.

```javascript
function parentFunc() {
  var a = 1;

  function nestedFunc() {
    var b = 4; // parentFunc은 사용할 수 없는 변수
    return a + b;
  }
  return nestedFunc(); // 5
}
```

## Closures

```javascript
function makeAdder(a) {
  return function (b) {
    return a + b;
  };
}
var add5 = makeAdder(5);
var add20 = makeAdder(20);
add5(6); // ?
add20(7); // ?
```

`makeAdder` 함수는 `a`라는 매개변수를 받아 `b`라는 매개변수와 합한 값을 반환하는 새로운 함수를 만들고, *이 함수*를 반환합니다.

상식적으로 우리는 `makeAdder` 함수가 호출되었을 때 함수를 반환하고 사라질 것이라고 생각하지만 사실 사라지지 않았습니다. 오히려 매개변수 `a`에 넘겨진 값을 지역변수로 기억하고 있으며 위 코드의 마지막 두 줄의 결과는 다음과 같습니다.

```javascript
add5(6); // returns 11
add20(7); // returns 27
```

JavaScript에서 함수를 실행하면 _Scope_ 객체가 생성되어 함수 내에서 생성된 지역 변수를 저장해둡니다. 함수의 매개변수로 넘겨진 값 또한 마찬가지입니다. 이는 전역 변수와 함수가 존재하는 전역 객체(global object)와 비슷하지만 두 가지 중요한 차이가 있습니다.

1. 함수가 실행될 때마다 새로운 Scope 객체가 생성됩니다.
2. (`this` 또는 브라우저에서 `window`로 접근가능한) 전역 객체와 달리, Scope 객체들은 JavaScript 코드로 직접 접근할 수 없습니다. 예를 들어, 현재 Scope 객체의 속성들을 순환할 방법은 존재하지 않습니다.

따라서 `makeAdder()`가 호출될 때, Scope 객체가 `a`라는 속성과 함께 생성됩니다. `makeAdder()`가 새로 생성된 함수를 반환하면, 보통 JavaScript의 Garbage collector가 `makeAdder()`에서 생성된 Scope 객체를 정리합니다. 하지만 반환된 함수가 Scope 객체를 참조하고 있기 때문에 참조가 없어질 때까지 정리되지 않습니다.

Scope 객체들은 Scope chain이라는 Prototype chain과 유사한 구조를 가집니다.

**Closure**는 함수와 Scope 객체의 조합으로, 상태를 저장할 수 있게 해줍니다.

[How do JavaScript closures work?](https://stackoverflow.com/questions/111102/how-do-javascript-closures-work)
