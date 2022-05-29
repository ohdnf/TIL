# Type Compatibility

```js
function func1(a: number, b: number | string) {
  // v1이 a보다 더 큰 집합이므로 할당 가능
  const v1: number | string = a;
  // v2는 b가 string일 수도 있으므로 할당 불가
  const v2: number = b;
}

function func2(a: 1 | 2) {
  const v1: 1 | 3 = a; // v1은 2를 포함할 수 없으므로 할당 불가
  const v2: 1 | 2 | 3 = a; // 할당 가능
}
```

## 타입스크립트는 값 자체 타입보다는 값이 가진 내부 구조의 타입에 따라 호환성을 검사(structural typing)합니다.

```js
export {};

interface Person {
  name: string;
  age: number;
}

interface Product {
  name: string;
  age: number;
}

const person: Person = { name: "Jude", age: 23 };
// Person과 Product의 내부 구조 타입이 동일하므로 할당 가능
const product: Product = person;
```

다음은 인터페이스 A가 인터페이스 B로 할당 가능하기 위한 조건입니다.

1. B에 있는 모든 필수 속성의 이름이 A에도 존재해야 합니다.
2. 같은 속성 이름에 대해 A의 속성이 B의 속성에 할당 가능해야 합니다.

> 인터페이스에 속성이 많아질수록 인터페이스 값의 집합은 작아진다.

## 함수의 타입 호환성

함수는 호출하는 시점에 타입 호환성을 검사합니다.
다음은 함수 타입 A가 함수 타입 B로 할당 가능하기 위한 조건입니다.

1. A의 매개변수 개수가 B의 매개변수 개수보다 적어야 합니다.
2. 같은 위치의 매개변수에 대해 B의 매개변수가 A의 매개변수로 할당 가능해야 합니다.
3. A의 반환값은 B의 반환값으로 할당 가능해야 합니다.
