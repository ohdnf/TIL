# Type Guard

- 타입 가드는 자동으로 타입의 범위를 좁혀주는 타입스크립트의 기능입니다.
- 값의 영역에서 사용한 코드(예: `typeof`, `instanceof`)를 분석해 타입의 범위를 좁힐 수 있습니다.
- `as`와 같은 타입 단언 코드를 피해 생산성과 가독성을 높여주는 효과를 줄 수 있습니다.

```js
export {};

function print(value: number | string) {
  if (typeof value === "number") {
    // console.log((value as number).toFixed(2));
    console.log(value.toFixed(2));
  } else {
    // console.log((value as string).trim());
    console.log(value.trim());
  }
}
```

```js
export {};

class Person {
  name: string;
  age: number;
  constructor(name: string, age: number) {
    this.name = name;
    this.age = age;
  }
}

class Product {
  name: string;
  price: number;
  constructor(name: string, price: number) {
    this.name = name;
    this.price = price;
  }
}

function print(value: Person | Product) {
  console.log(value.name);
  if (value instanceof Person) {
    console.log(value.age);
  } else {
    console.log(value.price);
  }
}
```

```js
export {};

interface Person {
  name: string;
  age: number;
}

interface Product {
  name: string;
  price: number;
}

function print(value: Person | Product) {
  console.log(value.name);
  // interface는 타입을 위한 코드이기 때문에 컴파일 이후엔 사라집니다.
  // instanceof는 컴파일 이후에 돌아가는 코드이기 때문에 클래스나 생성자 함수가 올 수 있습니다.
  if (value instanceof Person) {
    console.log(value.age);
  } else {
    console.log(value.price);
  }
}
```

```js
export {};

// discriminated union(식별 가능한 유니온) 타입을 사용해 interface를 구분합니다.

interface Person {
  type: "person";
  name: string;
  age: number;
}

interface Product {
  type: "product";
  name: string;
  price: number;
}

function print(value: Person | Product) {
  console.log(value.name);
  if (value.type === "person") {
    console.log(value.age);
  } else {
    console.log(value.price);
  }
}
```

```js
export {};

interface Person {
  name: string;
  age: number;
}

interface Product {
  name: string;
  price: number;
}

function isPerson(x: Person | Product): x is Person {
  return (x as Person).age !== undefined;
}

function print(value: Person | Product) {
  console.log(value.name);
  if (isPerson(value)) {
    console.log(value.age);
  } else {
    console.log(value.price);
  }
}
```

```js
export {};

// in 키워드로 속성 이름을 검사해 타입 가드를 활성화시킬 수 있습니다.

interface Person {
  type: "person";
  name: string;
  age: number;
}

interface Product {
  type: "product";
  name: string;
  price: number;
}

function print(value: Person | Product) {
  console.log(value.name);
  if ("age" in value) {
    console.log(value.age);
  } else {
    console.log(value.price);
  }
}
```
