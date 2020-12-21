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
console.log(3/2);				// 1(X) 1.5(O)
console.log(Math.floor(3/2));	// 1

// 0.1 + 0.2 = 0.30000000000000004

Math.sin(3.5);
let circumference = 2 * Math.PI * r;

// 문자열을 숫자로 변환
// 두번째 매개변수로 진수를 정할 수 있습니다.
parseInt('123', 10);	// 123
parseInt('010', 10);	// 10
parseInt('11', 2);		// 3
parseInt('0x10');		// 16, 문자열 앞이 0x로 시작하면 16진수로 취급

// 단항 연산자 +를 사용해도 문자열을 숫자로 변환 가능
typeof(+'42');			// number

// parsing하는 범위
parseInt("10.2abc");	// 10
parseFloat("10.2abc");	// 10.2
+"10.2abc";				// NaN

// 부동 소수점 숫자는 parseFloat() 사용

// 문자열이 수가 아닌 경우 = NaN
parseInt('hello', 10);	// NaN
isNaN(NaN);				// true

// 무한대
 1 / 0;					//  Infinity
-1 / 0;					// -Infinity
isFinite(1 / 0);		// false
isFinite(-Infinity);	// false
isFinite(NaN);			// false
```



## Strings

> JavaScript의 문자열은 각각이 16비트 숫자로 표현된 UTF-16 코드 유닛(유니코드 문자)이 길게 이어져 있는 것입니다.

```javascript
// 문자열의 length 속성을 활용해 문자열의 길이를 알 수 있습니다.
'hello'.length;		// 5

// 문자열도 객체입니다. 다음과 같은 메소드를 활용할 수 있습니다.
'hello'.charAt(0);	// "h"
'hello, world'.replace('hello', 'goodbye');	// "goodbye, world"
'hello'.toUpperCase();	// "HELLO"
```



## 이외의 타입들

### Null

- 의도적으로 값이 없음을 가리키는 **객체** 타입

### undefined

- 초기화되지 않은 값, 아직 어떤 값도 할당되지 않은 변수임을 가리키는 *정의되지 않은* **객체** 타입
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
Boolean('');	// false
Boolean(234);	// true
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





## 제어 구조





## Objects

> Python의 `dict()`



## Arrays

> Python의 `list()`



## Functions

### Arrow function

1. 클래스 정의에서 사용하지 않는다.(메소드 함수가 아닌 곳에서만 사용)
2. 생성자로 사용하지 않는다.
3. 이벤트 리스너 콜백 함수로 사용하지 않는다.



## 사용자 정의 객체

### Inner functions





## Closures



