# JavaScript 함수

```js
// 1. 기본 인자
function defaultArgs(a = 1, a = 2) {
  console.log(a + b);
  return a + b;
}
defaultArgs();
defaultArgs(20, 30);

// 2. 함수의 인자 개수
function wrongArgCount(a, b) {
  console.log(a + b);
}
wrongArgCount();
wrongArgCount(1);
wrongArgCount(1, 2, 3);
function noArgs() {
  console.log("no args...");
}
noArgs(1, 2, 3, 4, 5);

// 3. JS rest parameter
function restParameter1(...numbers) {
  console.log(numbers);
}
restParameter1(1, 2, 3, 4, 5);

function restParameter2(a, b, ...numbers) {
  console.log(a, b, numbers);
}
restParameter2(1, 2, 3, 4, 5);

// 4. Spread operator
function spreadOperator(a, b, c) {
  console.log(a, b, c);
}
let numbers = [1, 2, 3];

spreadOperator(numbers[0], numbers[1], numbers[2]);
spreadOperator(...numbers);
numbers = [1, 2, 3, 4];
spreadOperator(...numbers);

// 배열 합치기
let newNumbers = [0, ...numbers, 5];
console.log(newNumbers);
```
