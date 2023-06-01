# NestJS Documentation

[NestJS Documentation](https://docs.nestjs.com/)을 읽어보면서 요약 정리해보는 글입니다.

## Introduction

- Nest (NestJS)는 Node.js 서버 프레임워크입니다.
- TypeScript를 지원합니다(하지만 순수 JavaScript로도 개발 가능합니다).
- OOP(Object Oriented Programming), FP(Functional Programming)와 FRP(Functional Reactive Programming)의 요소를 포괄하고 있습니다.
- 기본적으로 HTTP 서버 프레임워크인 Express를 사용하며, Fastify도 지원합니다.
- 이러한 Node.js 프레임워크(Express/Fastify)에 대한 추상화를 제공하며, 동시에 API들을 직접 사용할 수 있기도 합니다.

### Philosophy

- 최근 몇년간 Node.js 덕에 JavaScript는 웹 프론트, 백엔드 애플리케이션의 "lingua franca"가 되었습니다.
- 이로 인해 Angular, React와 Vue와 같은 프로젝트를이 생기면서 개발자 생산성을 향상시키고 빠르고 테스트 가능하고 확장성 높은 프론트엔드 애플리케이션을 만들 수 있게 되었습니다.
- 하지만 그 무엇도 아직까지 Node의 핵심 문제 중 하나인 Architecture에 대한 고민을 해결해주진 못했습니다.
- Nest는 즉시 사용가능한 애플리케이션 아키텍처를 제공함으로써 개발자들로 하여금 테스트 가능하고, 확장 가능하고, 약하게 결합된 그리고 쉽게 유지보수할 수 있는 애플리케이션을 만들 수 있게 하였습니다.
- 그 아키텍처는 대부분 Angular로부터 영감을 받았습니다.

### Installation

- NestJS 프로젝트를 시작하기 위해선 [Nest CLI](https://docs.nestjs.com/cli/overview)를 사용하거나 시작 프로젝트를 clone하면 됩니다.
- Nest CLI로 새로운 프로젝트를 시작하려면 터미널에서 아래 명령어를 실행합니다.

```sh
$ npm i -g @nestjs/cli
$ nest new your-project-name
```

> [TypeScript strict mode](https://www.typescriptlang.org/tsconfig#strict)가 활성화되게 하려면 `nest new` 명령어에 `--strict` 옵션을 추가하세요.

### Alternatives

- Git으로 시작 프로젝트(TypeScript)를 설치하는 방법입니다.

```sh
$ git clone https://github.com/nestjs/typescript-starter.git project
$ cd project
$ npm install
$ npm run start
```

> git history를 없애고 clone을 하고 싶다면 [degit](https://github.com/Rich-Harris/degit)을 사용해보세요.

- NestJS 서버를 띄운 후에는 브라우저에서 `http://localhost:3000/`으로 이동해 확인해볼 수 있습니다.

- npm을 사용해서 밑바닥부터 직접 짤 수도 있습니다.

```sh
$ npm i @nestjs/core @nestjs/common rxjs reflect-metadata
```
