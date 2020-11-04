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
