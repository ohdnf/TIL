# 배열 순환

```javascript
const array = [1, 2, 3, 4];

// index read/write, break ok
for (let i = 0; i < array.length; i++) {
    if (i == 3) break
    console.log('basic ' + array[i]);
};

// index read, break ok
for (const i in array) {
    if (i == 3) break
    console.log('in ' + array[i]);
}

// break ok
for (const v of array) {
    if (i == 3) break
    console.log('of ' + v);
}

// break x
array.forEach(v => console.log('each ' + v));
```

- 아래로 내려올 수록 기능이 적어진다.
- 기능이 적어질 수록 문법이 간결해져 이해하기가 쉬워진다.
- 간결하게 코딩한 다음 필요한 기능을 추가하는 방식으로 구현하자.


## map

- 배열의 요소들을 하나씩 돌며 함수에 매핑시킨 새로운 배열을 만들어준다.
- 기존 배열 객체는 수정하지 않는다.

```javascript
const arr = [1, 2, 3];
let result = arr.map((element, index, arr) => {
    return element + 1;
});

console.log(result);    // [2, 3, 4]
```



## reduce

`배열.reduce((누적값, 현잿값, 인덱스, 요소) => { return 결과 }, 초깃값);`



```javascript
result = arr.reduce((acc, cur, i) => {
    console.log(`acc: ${acc}, cur: ${cur}, i: ${i}`);
    return acc + cur;
}, 0);
// acc: 0, cur: 1, i: 0
// acc: 1, cur: 2, i: 1
// acc: 3, cur: 3, i: 2
// 6
```

```javascript
result = arr.reduce((acc, cur) => {
    if (cur % 2) acc.push(cur);
    return acc;
}, []);
// [1, 3]
```


```javascript
const promiseFactory = (time) => {
  return new Promise((resolve, reject) => {
    console.log(time); 
    setTimeout(resolve, time);
  });
};
[1000, 2000, 3000, 4000].reduce((acc, cur) => {
  return acc.then(() => promiseFactory(cur));
}, Promise.resolve());
// 바로 1000
// 1초 후 2000
// 2초 후 3000
// 3초 후 4000
```


## 기타 배열 메서드

sort, filter, every, some, find, findIndex, includes
