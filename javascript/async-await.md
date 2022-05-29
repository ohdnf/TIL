# async와 await

## Promise

> JavaScript 비동기 처리에 사용되는, Object 안에 Object를 포함하는 JavaScript의 특별한 객체다.
>
> async 함수는 promise를 리턴하고, 모든 await 함수는 일반적으로 promise가 된다.
>
> promise에는 `.then()`을 사용하여 접근할 수 있다. promise 체인에서 error는 `.catch()`를 사용해 처리한다.

```javascript
function getFirstUser() {
  return getUsers()
    .then(function (users) {
      return users[0].name;
    })
    .catch(function (err) {
      return {
        name: "default user",
      };
    });
}
```

### Promise의 3가지 상태(states)

#### Pending(대기)

> 비동기 처리 로직이 아직 완료되지 않은 상태

`new Promise()` 메서드를 호출하면 대기(Pending) 상태가 됩니다. 이 때 `resolve`와 `reject`를 인자로 갖는 콜백 함수를 선언할 수 있다.

```javascript
new Promise(function(resolve, reject)) {
	// ...
}
```

#### Fulfilled(이행)

> 비동기 처리가 완료되어 프로미스가 결과값을 반환해준 상태

콜백 함수의 인자 `resolve`를 실행하면 이행 상태가 된다. 이행 상태가 되면 `then()`을 사용해 처리 결과값을 받을 수 있다.

```javascript
function getData() {
  return new Promise(function (resolve, reject) {
    let data = 100;
    resolve(data);
  });
}

// resolve()의 결과 값 data를 resolvedData로 받음
getData().then(function (resolvedData) {
  console.log(resolvedData); // 100
});
```

#### Rejected(실패)

> 비동기 처리가 실패하거나 오류가 발생한 상태

콜백 함수의 인자 `reject`를 호출하면 실패 상태가 된다. 실패 상태가 되면 `catch()`를 사용해 실패 처리의 결과값을 받을 수 있다.

```javascript
function getData() {
  return new Promise(function (resolve, reject) {
    reject(new Error("Request is failed"));
  });
}

// reject()의 결과 값 Error를 err에 받음
getData()
  .then()
  .catch(function (err) {
    console.log(err); // Error: Request is failed
  });
```

#### 처리 흐름

<img src="https://mdn.mozillademos.org/files/8633/promises.png" alt="Promise process" />

## async/await

> JavaScript의 비동기 처리 패턴
>
> 콜백 함수와 프로미스의 단점을 보완하고 개발자가 읽기 좋은 코드를 작성할 수 있게 한다.

```javascript
async function 함수명() {
  await 비동기_처리_메서드_명();
}
```

비동기 처리 메서드가 프로미스 객체를 반환해야 `await`가 의도한 대로 동작한다.

## Reference

https://medium.com/@kiwanjung/번역-async-await-를-사용하기-전에-promise를-이해하기-955dbac2c4a4
