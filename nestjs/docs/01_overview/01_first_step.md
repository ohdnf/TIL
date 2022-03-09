# First Step

## Language

TypeScript, JavaScript(Babel 컴파일러 필요) 호환

## Prerequisites

Node.js (>= 10.13.0, except for v13)

## Setup

Nest CLI를 이용해 아래와 같이 프로젝트 시작

```shell
$ npm i -g @nestjs/cli
$ nest new project-name
```

`project-name`이라는 폴더 아래 여러 module들과 템플릿이 생성되며, `src` 폴더는 아래와 같은 구조를 가짐

```js
src
├─app.controller.spec.ts  // The unit tests for the controller.
├─app.controller.ts // A basic controller with a single route.
├─app.module.ts // The root module of the application.
├─app.service.ts // A basic service with a single method.
└─main.ts // The entry file of the application which uses the core function NestFactory to create a Nest application instance.
```

`main.ts`는 애플리케이션을 시동시키는 async 함수를 포함

```js
import { NestFactory } from '@nestjs/core';
import { AppModule } from './app.module';

async function bootstrap() {
  const app = await NestFactory.create(AppModule);
  await app.listen(3000);
}
bootstrap();
```

## Platform?

- Nest는 특정 플랫폼에 구애받지 않는 프레임워크를 지향
- 재사용성이 높은 프레임워크를 지향
- Node.js의 대표적인 HTTP 플랫폼으로는 Express와 Fastify가 있으며, `NestFactory.create()`에 `NestExpressApplication` 타입 또는 `NestFastifyApplication` 타입을 지정하여 플랫폼을 명시 가능

## Running the application

아래 명령어로 HTTP 요청을 받는 애플리케이션을 실행

```shell
$ npm run start
```
