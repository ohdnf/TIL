# 배열 관련 함수

콜백 함수(A)는 다른 함수(B)의 인자로 넘겨지는 함수로, 다른 함수(B)의 내부에서 실행되는 함수(A)를 말한다.

```js
// 1. map
["1", "2", "3"].map(Number);

const numbers = [0, 9, 99];
function addOne(number) {
  return number + 1;
}

const newNumbers1 = numbers.map(addOne);
const newNumbers2 = [0, 9, 99].map(function (number) {
  return number + 1;
});

// 2. forEach
let sum = 0;
numbers = [1, 2, 3];
numbers.forEach(function (number) {
  sum += number;
});

// 3. filter
const odds = [1, 2, 3].filter(function (number) {
  return number % 2;
});

// 4. reduce
const oneTwoThree = [1, 2, 3];
let result = oneTwoThree.reduce((acc, cur, i) => {
  console.log(acc, cur, i);
  return acc + cur;
}, 0);
// 0 1 0
// 1 2 1
// 3 3 2
console.log(result); // 6
```
