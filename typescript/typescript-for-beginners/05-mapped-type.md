# Mapped Type

- mapped type을 사용해 interface에 있는 모든 속성을 optional(선택 속성), readonly 등으로 바꾸는 일을 할 수 있습니다.

```js
export {};

type T1 = { [K in 'prop1' | 'prop2']: boolean };
// T1 타입은 prop1, prop2라는 boolean 타입 속성을 가집니다.
```

- 입력된 모든 속성을 boolean 타입으로 만들어주는 mapped type을 작성해보겠습니다.

```js
export {};

interface Person {
  name: string;
  age: number;
}

// T가 가진 모든 속성을 boolean 타입의 선택 속성으로 만들어 적용할 수 있음
type MakeBoolean<T> = { [P in keyof T]>: boolean };
const pMap: MakeBoolean<Person> = {};
pMap.name = true;
pMap.age = false;
```

- readonly도 적용할 수 있습니다.

```js
export {};

interface Person {
  name: string;
  age: number;
}

type T1 = Person['name'];
// T[P]를 통해 기존 인터페이스 T의 속성 타입을 변화시키지 않고 있습니다.
type Readonly<T> = { readonly [P in keyof T]: T[P] }; // 모든 속성을 readonly로 적용
type Partial<T> = { [P in keyof T]?: T[P] }; // 모든 속성을 선택 속성으로 적용
type T2 = Partial<Person>;
type T3 = Readonly<Person>;
// Readonly와 Partial은 TypeScript에 내장되어 있습니다.
```

- TypeScript의 내장 기능인 Pick도 아래와 같이 이해할 수 있습니다.

```js
export {};

// K는 인터페이스 T의 속성에 포함될 수 있는 속성들
// type Pick<T, K extends keyof T> = { [P in K]: T[P] };
interface Person {
  name: string;
  age: number;
  language: string;
}
type T1 = Pick<Person, "name" | "language">;
```

- Record 또한 TypeScript의 내장 타입입니다.

```js
export {};

interface Person {
  name: string;
  age: number;
  language: string;
}
// type Record<K extends string, T> = { [P in K]: T };
type T1 = Record<"p1" | "p2", Person>;
type T2 = Record<"p1" | "p2", number>;
```

- mapped type을 활용해 enum 타입의 활용도를 높여봅시다.

```js
export {};

enum Fruit {
  Apple,
  Banana,
  Orange,
}
const FRUIT_PRICE: { [key in Fruit]: number } = {
  [Fruit.Apple]: 1000,
  [Fruit.Banana]: 1500,
  [Fruit.Orange]: 2000,
}
```

- `in` 오른쪽에 `enum` 타입이 온다면 해당 `enum`에 있는 모든 아이템을 나열해줘야 합니다.
