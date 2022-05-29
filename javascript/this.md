# this

method ==> 객체 안에 정의된 함수 (Object.methodName() 으로 실행하는 함수)
function ==> method가 아닌 모든 함수

this는 기본적으로 window

단, 아래의 두 가지 경우는 예외

1. method 정의 블록 안의 this ==> 해당 method가 정의된 객체(object)
   (method 정의할 때는 arrow function을 쓰지 않는다!)
2. 생성자 함수 안의 this

```js
const obj = {
  name: "obj",
  method1: function () {
    console.log(this); // obj
  },
  objInObj: {
    name: "object in object",
    oioMethod() {
      console.log(this); // objInObj
    },
  },
  arr: [0, 1, 2],
  newArr: [],
  method2() {
    this.arr.forEach(
      /*
아래 function은 메소드가 아니다. 그러므로 this는 window

function(number) {
this.newArr.push(number * 100)
}.bind(this)

*/

      (number) => {
        this.newArr.push(number * 100);
      }
    ); // obj
  },
};
```

```js
const counter3 = {
  value: 0,
  add: (amount) => {
    // 일반 객체 안의 화살표 함수는 감싸고 일반 함수가 없기 때문에 this는 항상 전역 객체를 가리킴
    this.value += amount;
  },
};
console.log(counter3.value); // 0
counter3.add(5); // counter3 객체의 value는 변하지 않는다.
console.log(counter3.value); // 0
const add = counter3.add;
add(5);
console.log(counter3.value); // 0
```
