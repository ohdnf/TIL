# Middleware

- middleware는 route 핸들러 전에 호출되는 함수
- request-response 싸이클에서 `request`와 `response` 객체, 그리고 `next()` 함수에 접근 가능

```
Client Side -(HTTP Request)-> Middleware -> Route Handler(@RequestMapping)
```

- NestJS middleware는 기본적으로 express middleware와 동일
- middleware가 수행하는 역할은 다음과 같음:
  - 코드 실행
  - request, response 객체 수정
  - request-response cycle 종료
  - stack에서 `next()` 호출
  - 현 middleware에서 request-response cycle이 종료되지 않는다면 반드시 `next()`를 호출해 다음 middleware로 넘어가도록 처리
- NestJS에서 middleware는 함수 또는 `@Injectable()` decorator를 붙인 클래스 형태로 구현
- 클래스 middleware의 경우 `NestMiddleware` interface를 implement해야 함

```js
// logger.middleware.ts
import { Injectable, NestMiddleware } from '@nestjs/common';
import { Request, Response, NextFunction } from 'express';

@Injectable()
export class LoggerMiddleware implements NestMiddleware {
  use(req: Request, res: Response, next: NextFunction) {
    console.log('Request...');
    next();
  }
}
```

## Dependency injection

- `constructor`를 통해 의존성 주입 가능

## Applying middleware

- `@Module()` decorator에는 middleware 추가 불가, 대신 `configure()` 메서드 사용
- middleware를 사용하고자 하는 class에서는 `NestModule` interface를 implement해야 함
- 아래 예시에서 `AppModule`에 `LoggerMiddleware` 설정

```js
// app.module.ts
import { Module, NestModule, MiddlewareConsumer } from '@nestjs/common';
import { LoggerMiddleware } from './common/middleware/logger.middleware';
import { CatsModule } from './cats/cats.module';

@Module({
  imports: [CatsModule],
})
export class AppModule implements NestModule {
  configure(consumer: MiddlewareConsumer) {
    consumer.apply(LoggerMiddleware).forRoutes('cats');
  }
}
```

- 위 예시에서는 `/cats` 경로의 router handler에 `LoggerMiddleware`를 설정
- `forRoutes()` 메서드에 route `path`와 request `method`를 포함한 객체를 전달해 특정 요청 메서드로 제한 가능

```js
// app.module.ts
import {
  Module,
  NestModule,
  RequestMethod,
  MiddlewareConsumer,
} from '@nestjs/common';
import { LoggerMiddleware } from './common/middleware/logger.middleware';
import { CatsModule } from './cats/cats.module';

@Module({
  imports: [CatsModule],
})
export class AppModule implements NestModule {
  configure(consumer: MiddlewareConsumer) {
    consumer
      .apply(LoggerMiddleware)
      .forRoutes({ path: 'cats', method: RequestMethod.GET });
  }
}
```

> `configure()` 메서드는 `async/await`를 사용해 비동기적으로 설정 가능

## Route wildcards

- 패턴 기반 route

```js
forRoutes({ path: 'ab*cd', method: RequestMethod.ALL });
```

- `ab*cd` route는 `abcd`, `ab_cd`, `abecd` 등과 매칭 가능
- `?`, `+`, `*`, `()` 사용 가능

## Middleware consumer

- middleware를 관리하기 위한 built-in 메서드를 제공하는 helper class
- `apply()` 메서드에 단일 middleware 또는 복수 개의 middleware 설정 가능
- `forRoutes()`에도 controller 배열 전달 가능

```js
// app.module.ts
import { Module, NestModule, MiddlewareConsumer } from '@nestjs/common';
import { LoggerMiddleware } from './common/middleware/logger.middleware';
import { CatsModule } from './cats/cats.module';
import { CatsController } from './cats/cats.controller';

@Module({
  imports: [CatsModule],
})
export class AppModule implements NestModule {
  configure(consumer: MiddlewareConsumer) {
    consumer.apply(LoggerMiddleware).forRoutes(CatsController);
  }
}
```

## Excluding routes

- `exclude()` 메서드를 사용해 middleware가 적용될 route를 **배제** 가능

```js
consumer
  .apply(LoggerMiddleware)
  .exclude(
    { path: 'cats', method: RequestMethod.GET },
    { path: 'cats', method: RequestMethod.POST },
    'cats/(.*)'
  )
  .forRoutes(CatsController);
```

## Functional middleware

- 함수형 middleware도 사용 가능

```js
// logger.middleware.ts
import { Request, Response, NextFunction } from 'express';

export function logger(req: Request, res: Response, next: NextFunction) {
  console.log(`Request...`);
  next();
}
```

```js
consumer.apply(cors(), helmet(), logger).forRoutes(CatsController);
```

## Multiple middleware

- 위 예시처럼 `apply()` 메서드에 복수 개의 middleware 추가 가능

## Global middleware

```js
// main.ts
const app = await NestFactory.create(AppModule);
app.use(logger);
await app.listen(3000);
```

> Global middleware에 있는 DI container 접근은 불가능
> `app.use()` 대신 함수형 middleware를 사용하거나 class middleware에서 `AppModule` 안에서 `.forRoutes(*)`를 사용해 consume하는 방향으로 적용
