# Destructuring

구조화된 배열 또는 객체를 비구조화, 파괴하여 개별적인 변수에 할당하는 것

```js
// Array destructuring

// ES5
var arr = [1, 2, 3];

var one = arr[0];
var two = arr[1];
var three = arr[2];

console.log(one, two, three);

// ES6
const arr = [1, 2, 3];

const [one, two, three] = arr;

console.log(one, two, three);

console.log("------------------------");

let x, y, z;

[x, y] = [1];
console.log(x, y); // 1 undefined

[x, y] = [1, 2, 3];
console.log(x, y); // 1 2

[x, , z] = [1, 2, 3];
console.log(x, z); // 1 3

[x, y, z = 3] = [1, 2];
console.log(x, y, z); // 1 2 3

[x, y = 10, z = 3] = [1, 2];
console.log(x, y, z); // 1 2 3

// spread 문법
[x, ...y] = [1, 2, 3];
console.log(x, y); // 1 [ 2, 3 ]

// 활용 예시
const today = new Date(); // Tue Sep 22 2020 22:19:42 GMT+0900 (한국 표준시)
const formattedDate = today.toISOString().substring(0, 10); // "2020-09-22"
const [year, month, day] = formattedDate.split("-");
console.log([year, month, day]); // [ '2020', '09', '22' ]

// Object destructuring

// ES5
var obj = { firstName: "ohdnf", lastName: "6uoy" };

var firstName = obj.firstName;
var lastName = obj.lastName;

console.log(firstName, lastName); // ohdnf 6uoy

// ES6
const obj = { firstName: "ohdnf", lastName: "6uoy" };

const { lastName, firstName } = obj;

console.log(firstName, lastName); // ohdnf 6uoy

console.log("------------------------");

const { prop1: p1, prop2: p2 } = { prop1: "a", prop2: "b" };
console.log(p1, p2); // 'a' 'b'
console.log({ prop1: p1, prop2: p2 }); // { prop1: 'a', prop2: 'b' }

console.log("this can be shorten to...");

const { prop1, prop2 } = { prop1: "a", prop2: "b" };
console.log({ prop1, prop2 }); // { prop1: 'a', prop2: 'b' }

const { prop1, prop2, prop3 = "c" } = { prop1: "a", prop2: "b" };
console.log({ prop1, prop2, prop3 }); // { prop1: 'a', prop2: 'b', prop3: 'c' }

// 활용 예시

const todos = [
  { id: 1, content: "HTML", completed: true },
  { id: 2, content: "CSS", completed: false },
  { id: 3, content: "JS", completed: false },
];

// todos 배열의 요소 객체로부터 completed 프로퍼티가 true인 요소만을 추출한다.
const completedTodos = todos.filter(({ completed }) => completed);
console.log(completedTodos); // [ { id: 1, content: 'HTML', completed: true } ]

// 중첩 객체

const person = {
  name: "Lee",
  address: {
    zipCode: "03068",
    city: "Seoul",
  },
};

const {
  address: { city },
} = person;
console.log(city); // 'Seoul'
```
