# Generic

- 제네릭은 타입 정보가 동적으로 결정되는 타입입니다.
- 제네릭을 통해 같은 규칙을 여러 타입에 적용해 중복 코드를 제거할 수 있습니다.

```js
export {};

function makeNumberArray(defaultValue: number, size: number): number[] {
  const arr: number[] = [];
  for (let i = 0; i < size; i++) {
    arr.push(defaultValue);
  }
  return arr;
}
function makeStringArray(defaultValue: string, size: number): string[] {
  const arr: string[] = [];
  for (let i = 0; i < size; i++) {
    arr.push(defaultValue);
  }
  return arr;
}
const arr1 = makeNumberArray(1, 10);
const arr2 = makeStringArray("empty", 10);
```

- 위 예에서 중복되는 코드를 없애고 싶다...!

```js
export {};

function makeArray(defaultValue: number, size: number): number[];
function makeArray(defaultValue: string, size: number): string[];
function makeArray(defaultValue: number | string, size: number): Array<number | string> {
  const arr: Array<number | string> = [];
  for (let i = 0; i < size; i++) {
    arr.push(defaultValue);
  }
  return arr;
}
const arr1 = makeArray(1, 10);
const arr2 = makeArray('empty', 10);
```

- 함수 오버로드를 사용하면 중복을 제거할 수 있다.
- 계속해서 필요한 타입이 늘어나면 어떨까?

```js
export {};

function makeArray<T>(defaultValue: T, size: number): T[] {
  const arr: T[] = [];
  for (let i = 0; i < size; i++) {
    arr.push(defaultValue);
  }
  return arr;
}
const arr1 = makeArray < number > (1, 10);
const arr2 = makeArray < string > ("empty", 10);
// <> 안에 타입 정보를 입력하지 않아도 자동으로 타입을 추론합니다.
const arr3 = makeArray(1, 10);
const arr4 = makeArray("empty", 10);
```

- 위와 같이 `<>`를 사용해 제네릭 타입을 설정할 수 있습니다.
- `T`는 임의의 이름으로 설정이 가능합니다. `makeArray<myType>`과 같이 설정하는 것도 가능하다는 뜻입니다.
- `T`는 매개변수 쪽과 구현하는 쪽에서 모두 사용가능하며, `T`의 타입은 함수를 호출할 때 동적으로 결정됩니다.
- 제네릭은 데이터의 타입에 다양성을 부여해주기 때문에 자료구조에서 많이 사용됩니다.

```js
export {};

class Stack<D> {
  private items: D[] = [];
  push(item: D) {
    this.items.push(item);
  }
  pop() {
    return this.items.pop();
  }
}

const numberStack = new Stack<number>();
numberStack.push(10);
const v1 = numberStack.pop();
const stringStack = new Stack<string>();
stringStack.push('a');
const v2 = stringStack.pop();

let myStack: Stack<number>;
myStack = numberStack;
myStack = stringStack; // error
```

- 제네릭 타입에는 아무 타입이나 입력할 수 있지만 React.js와 같은 라이브러리의 API는 입력가능한 값의 범위를 제한합니다.
- React.js의 속성값 전체는 객체 타입만 허용됩니다.
- 제네릭 타입에 `extends`를 사용해 타입의 종류를 제한할 수 있습니다.

```js
export {};

function identity<T extends number | string>(p1: T): T {
  return p1;
}
identity(1);
identity('a');
identity([]); // error
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

// type T1 = keyof Person; === type T1 = "name" | "age";
function swapProperty<T extends Person, K extends keyof Person>(
  p1: T,
  p2: T,
  key: K,
): void {
  const temp = p1[key];
  p1[key] = p2[key];
  p2[key] = temp;
}

const p1: Korean = {
  name: '홍길동',
  age: 23,
  liveInSeoul: true,
};
const p2: Korean = {
  name: '김선달',
  age: 31,
  liveInSeoul: false,
};
swapProperty(p1, p2, 'age');
```
