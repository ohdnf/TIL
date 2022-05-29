타입스크립트는 컴파일 시점에 자바스크립트의 타입을 검사해주는 언어다.

```js
const size: number = 123;
const isBig: boolean = size > 100;
const msg: string = isBig ? "Big" : "Small";

const values: number[] = [1, 2, 3];
const values2: Array<number> = [1, 2, 3];
// values.push("a"); // Argument of type 'string' is not assignable to parameter of type 'number'.

const data: [string, number] = [msg, size];
data[0].substring(1);
// data[1].substring(1); // Property 'substring' does not exist on type 'number'.

console.log(typeof 123); // number
console.log(typeof "abc"); // string
console.log(typeof [1, 2, 3]); // object

let v1: undefined = undefined;
let v2: null = null;
// v1 = 123; // Type '123' is not assignable to type 'undefined'.

let v3: number | undefined = undefined;
v3 = 123;

console.log(typeof undefined); // undefined
console.log(typeof null); // null

let anyValue: any;
anyValue = 123;
anyValue = "abc";
anyValue = false;
anyValue = undefined;
anyValue = () => {};

/* 함수의 반환 타입 */
function f1(): void {
  console.log("f1");
}

function f2(): never {
  // Return type 'never' is used when
  // a function is not end due to infinite loop
  while (true) {}
  // or function always throw error
  throw new Error("INFINITE LOOP");
}

/* 타입 만들기 */
type StringOrNumber = string | number;
let strOrNum: StringOrNumber = "abc";
strOrNum = 123;

/* Enum */
enum Color {
  Red,
  Green = 5,
  Blue,
}
const c1: Color = Color.Green;
const c2: Color.Blue | Color.Green = Color.Green;

/**
 * enum의 각 원소에는 숫자 또는 문자를 할당할 수 있다.
 * 맨 처음 원소에 값을 할당하지 않으면 0이 할당된다.
 * 그 다음 원소는 이전 원소에서 1만큼 증가한 값이 할당된다.
 */
enum Fruit {
  Apple, // 0
  Banana = 5,
  Mango, // 6
}
console.log(Fruit.Apple, Fruit.Banana, Fruit.Mango); // 0, 5, 6

/**
 * enum의 원소에 숫자를 할당하면 양방향으로 매핑된다.
 * 문자열을 할당하는 경우에는 단방향으로 매핑된다.
 */

function getIsValidEnumValue(enumObject: any, value: number | string) {
  return Object.keys(enumObject)
    .filter((key) => isNaN(Number(key)))
    .some((key) => enumObject[key] === value);
}

/**
 * enum을 사용하면 컴파일 후에도 enum 객체가 남아있어 번들 파일의 크기가 불필요하게 커진다.
 * enum 객체에 접근하지 않는다면 const enum을 사용해 컴파일 결과에 enum 객체를 남기지 않도록 할 수 있다.
 */
const enum Language {
  Korean = "ko",
  English = "en",
  Japanese = "ja",
  Chinese = "ch",
  French = "fr",
  Spanish = "es",
}
/**
 * 'const' enums can only be used in property or index access expressions or the right hand side of an import declaration or export assignment or type query.
 */
// getIsValidEnumValue(Language, "ko"); // error
```
