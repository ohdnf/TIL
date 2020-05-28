# Python이랑 어떻게 다른가?

## I. 스코프 & 변수

1. 스코프
> 중2병 히키코모리 막내동생 법칙

- JS의 경우 중괄호(`{}`)가 있을 경우, 스코프를 만든다.
    - function일 경우
    - 제어문(if/switch/while/for 등)일 경우
- `var`는 function에서만 스코프로 묶임(function scope)
- `let`은 중괄호가 있을 경우(block scope) 동일하게 작동

```py
name = 'john'

if True:
    # name = 'ohdnf'
    print(name)
```

```js
// ES6+ ==> 모던 JavaScript 문법
name = 'john' // 전역 깡패

if (true) {
    var age = 30;
    // let age = 19;
}
// const
console.log(name)
```


2. `var`, `let`, `const`

- ~~`var`: 할당 및 선언은 자유, 함수 스코프 ==> 절대 쓰지 않는다~~
- `let`: 할당 자유, 선언은 한 번, 블록 스코프
- `const`: 할당 & 선언 한 번, 블록 스코프

## VI. 자료구조
어떻게 저장하고, 조작하는지(CRUD) => 메서드
- Array (`list`)
- Object (`dict`)

## VII. 함수
- Arrow Function
1. 클래스 정의에서 사용 X
> (클래스) 메소드(OOP) 함수가 아닌 곳에서만
2. 생성자로 사용 X
3. 이벤트 리스너 콜백 함수로 사용 X

## VIII. OOP
> Prototypal Inheritance
> `class`를 잠시 잊어주시길...