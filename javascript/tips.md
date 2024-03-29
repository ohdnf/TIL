# tips

```js
/* let vs const */
// let ==> 재할당 가능
let a = 1;
a = 1;
// const ==> "재할당 불가능" but "값은 바뀔 수 있다"
const b = 1;
b = 'a' // error
const c = [1, 2, 3];
c.push(4) // 가능

/**
 * Style Guide: 돈 주는 회사의 방침에 따르자
 * Airbnb: https://github.com/airbnb/javascript
 * Google https://google.github.io/styleguide/jsguide.html
 */

// Naming Convention 꼭 지키자. 맞춤법 틀린 글은 읽기 싫다.
const add_one_to_number // (bad)
const addOneToNumber // (good)

// 느슨한 일치 연산자를 절대 사용하지 않는다.
// ==, != (x)

// Function ==> 1급 객체
// 1. 변수에 저장할 수 있다.
// 2. 함수의 리턴값이 될 수 있다.
// 3. 함수의 인자가 될 수 있다.

// Warning!
// 콜백 함수를 쓴다 => non-blocking한 비동기 작업을 하는 것이다. (X)
// non-blocking한 비동기 작업을 한다 => 콜백을 쓸 수 밖에 없도록 설계되어 있다. (O)

// Promise는 왜 사용하는가?
// ==> 불확실하고 기다려야하는 작업(AJAX axios)을 비동기적으로 처리하기 위해서
```
