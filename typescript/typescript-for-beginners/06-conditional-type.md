# Conditional Type

- 조건부 타입은 `extends`와 `?`를 사용해 제네릭으로 입력된 타입의 할당 가능 여부를 확인해 타입을 지정합니다.

```js
export {};

// T extends U ? X : Y

type IsStringType<T> = T extends string ? 'yes' : 'no';
type T1 = IsStringType<string>;
type T2 = IsStringType<number>;
```

- 조건부 타입에 유니온 타입을 적용한 예도 확인해봅시다.

```js
export {};

/**
 * 조건부 타입 T에 유니온 타입을 사용(T1)하면
 * T2처럼 각 타입에 조건부 타입을 적용한 뒤에
 * 해당 타입들을 유니온 타입으로 묶어줍니다.
 */
type IsStringType<T> = T extends string ? 'yes' : 'no';
type T1 = IsStringType<string | number>;
type T2 = IsStringType<string> | IsStringType<number>;

type Array1<T> = Array<T>;
type T3 = Array1<string | number>;
```

- TypeScript에는 `Exclude`, `Extract`라는 내장된 조건부 타입이 존재합니다.

```js
export {};

// never는 해당 타입을 사용하지 않는 것으로 조건부 타입에서 사용됩니다.
type T1 = number | string | never;
// Exclude에선 T가 U에 할당 가능하면 never가 사용되고 아니면 T가 사용됩니다.
type Exclude<T, U> = T extends U ? never : T;
// 1, 5, 9에 포함되지 않는 3, 7만 타입으로 사용
type T2 = Exclude<1 | 3 | 5 | 7, 1 | 5 | 9>;
// Function에 할당 가능한 (() => void)만 제거되고 나머지 타입만 남습니다.
type T3 = Exclude<string | number | (() => void), Function>;
// Extract에선 T가 U에 할당 가능하면 T가 사용되고 아니면 never가 사용됩니다.
type Extract<T, U> = T extends U ? T : never;
type T4 = Extract<1 | 3 | 5 | 7, 1 | 5 | 9>;
```

- TypeScript의 내장 타입인 ReturnType은 T가 함수일 때 T 함수의 반환 타입을 뽑아줍니다.
- 내부적으로 조건부 타입을 사용합니다.

```js
export {};

// infer를 사용해 함수 T의 반환값을 추론합니다.
type ReturnType<T> = T extends (...args: any[]) => infer R ? R : any;
type T1 = ReturnType<() => string>; // T1의 타입은 string입니다.
function f1(s: string): number {
  return s.length;
}
type T2 = ReturnType<typeof f1>; // T2의 타입은 number입니다.
```

- `infer` 키워드는 중첩 사용이 가능합니다.

```js
export {};

type Unpacked<T> = T extends (infer U)[]
  ? U
  : T extends (...args: any[]) => infer U
  ? U
  : T extends Promise<infer U>
  ? U
  : T;
type T0 = Unpacked<string>; // string
type T1 = Unpacked<string[]>; // string
type T2 = Unpacked<() => string>; // string
type T3 = Unpacked<Promise<string>>; // string
type T4 = Unpacked<Promise<string>[]>; // Promise<string>
type T5 = Unpacked<Unpacked<Promise<string>[]>>; // string
```

- 조건부 타입을 사용해 유틸리티 타입 만들어보기

```js
export {};

type StringPropertyNames<T> = {
  // mapped type을 사용해 T에서 string 타입의 property만 추출합니다.
  [K in keyof T]: T[K] extends string ? K : never;
}[keyof T]; // union type을 사용해 never를 제거한 속성 이름만 추출합니다.

interface Person {
  name: string;
  age: number;
  nation: string;
}
type T1 = StringPropertyNames<Person>; // "name" | "nation"

type StringProperties<T> = Pick<T, StringPropertyNames<T>>;
type T2 = StringProperties<Person>; // { name: string; nation: string; }
```

```js
export {};

type Omit<T, U extends keyof T> = Pick<T, Exclude<keyof T, U>>;
interface Person {
  name: string;
  age: number;
  nation: string;
}
type T1 = Omit<Person, 'nation' | 'age'>; // name 속성만 남습니다.
```

```js
export {};

// 인터페이스 T를 U로 덮어쓰는 유틸리티 타입
type Overwrite<T, U> = { [P in Exclude<keyof T, keyof U>]: T[P] } & U;
interface Person {
  name: string;
  age: number;
}
type T1 = Overwrite<Person, { age: string; nation: string }>;
const p: T1 = {
  name: "mike",
  age: "23",
  nation: "USA",
};
```
