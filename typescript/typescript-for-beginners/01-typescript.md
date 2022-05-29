# 타입스크립트 시작하기

## 타입스트립트란?

- JavaScript
  - 동적 타입(dynamic type)
  - 변수의 타입은 런타임(runtime)에 결정
- TypeScript
  - JavaScript의 동적 타입을 지원하면서 정적(static type)을 지원
  - 변수 타입을 컴파일 타임에 결정
- 정적 타입 언어
  - 진입 장벽이 높다.
  - 코드의 양이 많을 때 생산성이 높다.
  - 타입 오류가 컴파일 시 발견된다.
- 동적 타입 언어
  - 진입 장벽이 낮다.
  - 코드의 양이 적을 때 생산성이 높다.
  - 타입 오류가 런타임 시 발견된다.

## 설치부터 컴파일까지

1. [Node.js](https://nodejs.org/ko/)를 설치합니다.
2. 프로젝트 폴더에서 아래 명령어로 `package.json` 파일을 생성해줍니다.

```sh
npm init -y
```

3. 타입스크립트를 설치합니다.

```sh
npm install typescript
```

4. 타입스크립트 설정 파일인 `tsconfig.json`을 생성합니다.

```sh
npx tsc --init
```

5. 타입스크립트로 컴파일합니다.

```sh
npx tsc
```

> 타입 정보는 컴파일 타임에만 사용됩니다. 6. 외부 라이브러리(예: lodash) 사용해봅니다.

```sh
npm install lodash @types/lodash
```

## 실행 방법

1. 컴파일 후 Node.js로 실행

```sh
npx tsc
node src/index.js
```
