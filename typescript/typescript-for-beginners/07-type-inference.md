# Type Inference

- 타입 정의하는데 생산성이 떨어질 수 있기 때문에 타입 추론 기능을 활용해 꼭 필요한 경우에만 타입을 정의해줌으로써 타입 안정성을 유지할 수 있습니다.

```js
export {};

let v1 = 123;
let v2 = "abc";
v1 = "a";
v2 = 456;
```

```js
export {};

const v1 = 123;
const v2 = "abc";
const v3: number = 456;
let v4: typeof v1 = 345; // error
let v5: typeof v3 = 345;
```

```js
export {};

const arr1 = [10, 20, 30];
const [n1, n2, n3] = arr1;
arr1.push("10"); // error

const obj = { id: "qwer", age: 123, language: "korean" };
const { id, age, language } = obj;
console.log(id === age); // error
```

```js
export {};

interface Person {
  name: string;
  age: number;
}
interface Korean extends Person {
  liveInSeoul: boolean;
}
interface Japanese extends Person {
  liveInTokyo: boolean;
}

const p1: Person = { name: "one", age: 20 };
const p2: Korean = { name: "minsu", age: 20, liveInSeoul: true };
const p3: Japanese = { name: "mina", age: 20, liveInTokyo: false };
// 다른 타입으로 할당 가능한 타입은 제거가 됩니다.
const arr1 = [p1, p2, p3]; // Person[]
// 서로 할당 관계에 있지 않아 제거되지 않으면 여러 타입이 남으면 유니온 타입으로 묶이게 됩니다.
const arr2 = [p2, p3]; // (Korean | Japanese)[]
```

```js
export {};

// 기본값을 입력하면 자동으로 타입 추론이 됩니다.
function func1(a = "abc", b = 123) {
  return `${a} ${b}`;
}
func1(3, 6); // error
const v1: number = func1("a", 1); // error

// 반환값이 여러 개인 경우에도 반환 타입이 추론됩니다.
function func2(value: number) {
  if (value < 10) {
    return value;
  } else {
    return `${value} is too big`;
  }
}
```
