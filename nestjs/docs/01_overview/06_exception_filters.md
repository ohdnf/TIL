# Exception filters

- NestJS에는 애플리케이션 내에서 핸들링되지 않은 예외를 처리하는 exception layer가 장착되어 있음
- 애플리케이션 코드에서 예외처리가 되지 않으면 해당 레이어에서 사용자 친화적인 응답을 전송
- 기본적으로 global exception filter 내장 기능은 `HttpException` 타입(+하위 클래스 타입)의 예외를 처리
- 인식할 수 없는 예외는 아래와 같은 JSON 응답 반환

```json
{
  "statusCode": 500,
  "message": "Internal server error"
}
```

> global exception filter는 부분적으로 `http-errors` 라이브러리를 지원
> 기본적으로 `statusCode`와 `message` 속성을 포함한 예외는 자동적으로 응답으로 반환(`InternalServerErrorException` 대신)

## Throwing standard exceptions

- NestJS는 `@nestjs/common` package에 내장된 `HttpException` class를 제공
- HTTP REST/GraphQL API 기반 애플리케이션은 표준 HTTP 응답 객체를 반환하는게 best practice
- 예를 들어 `CatsController`에서 `GET` route handler인 `findAll()` 메서드에서 예외를 반환할 경우 아래와 같이 하드 코딩할 수 있음

```js
// cats.controller.ts
@Get()
async findAll() {
  throw new HttpException('Forbidden', HttpStatus.FORBIDDEN)
}
```

> `HttpStatus`는 `@nestjs/common` package에서 가져온 helper enum

- 클라이언트에서 받는 응답 구조는 아래와 같음

```json
{
  "statusCode": 403,
  "message": "Forbidden"
}
```

- `HttpException` constructor는 두 필수 인자를 받음
  - `response` 인자는 `string`이나 `object`를 받아 JSON response body를 정의
  - `status` 인자는 [HTTP status code](https://developer.mozilla.org/en-US/docs/Web/HTTP/Status)를 정의
- JSON response body는 아래 두 속성이 기본 설정되어 있음
  - `statusCode`: `status` 인자에서 제공된 HTTP status code
  - `message`: `status`에 기반한 HTTP 에러에 대한 간단한 설명
- JSON response body의 message 부분을 덮어쓰기 위해선 `response` 인자에 string을 전달
- JSON response body 전체를 덮어쓰기 위해선 `response` 인자에 object를 전달
- `status` 인자에는 유효한 HTTP status code가 필요, `@nestjs/common`에서 제공하는 `HttpStatus` enum을 사용하는 것이 적절

```js
// cats.controller.ts
@Get()
async findAll() {
  throw new HttpException({
    status: HttpStatus.FORBIDDEN,
    error: 'This is a custom message',
  }, HttpStatus.FORBIDDEN);
}
```

```json
{
  "status": 403,
  "error": "This is a custom message"
}
```

## Custom exceptions

- 예외를 커스터마이징하고 싶다면 `HttpException` class에서 상속된 예외 계층(exceptions hierarchy)을 만드는 것이 좋음

```js
// forbidden.exception.ts
export class ForbiddenException extends HttpException {
  constructor() {
    super('Forbidden', HttpStatus.FORBIDDEN);
  }
}
```

- `HttpException`에서 extend 되었으므로 `ForbiddenException`은 내장 예외 처리 핸들러에서 잘 작동

```js
// cats.controller.ts
@Get()
async findAll() {
  throw new ForbiddenException();
}
```

## Built-in HTTP exceptions

[사이트 참고](https://docs.nestjs.com/exception-filters#built-in-http-exceptions)

## Exception filters

- 기본 내장된 exception layer를 쓰면서, 동적 요소를 기반으로 다른 JSON schema나 로그 추가와 같은 작업을 하고 싶은 경우 exception filter를 활용
- 아래 예시는 `HttpException` class의 인스턴스 예외를 처리하고, 커스텀 응답 로직을 구현하는 exception filter
- `Request`, `Response` 객체가 필요한데, `Request` 객체에서 `url`과 로깅 정보를 뽑아오고, `Response` 객체와 `response.json()` 메서드를 사용해 반환 정보를 제어

```js
// http-exception.filter.ts
import { ExceptionFilter, Catch, ArgumentsHost, HttpException } from '@nestjs/common';
import { Request, Response } from 'express';

@Catch(HttpException)
export class HttpExceptionFilter implements ExceptionFilter {
  catch(exception: HttpException, host: ArgumentsHost) {
    const ctx = host.switchToHttp();
    const response = ctx.getResponse<Response>();
    const request = ctx.getRequest<Request>();
    const status = exception.getStatus();

    response
      .status(status)
      .json({
        statusCode: status,
        timestamp: new Date().toISOString(),
        path: request.url,
      });
  }
}
```

> 모든 exception filter는 generic `ExceptionFilter<T>` interface를 implement해야 함
> 따라서 `catch(exception: T, host: ArgumentsHost)` 메서드에 indicated signature를 제공해야 함
> `T`는 exception 타입을 의미

- `@Catch(HttpException)` decorator는 exception filter에 필수 메타데이터를 설정, 이를 통해 해당 exception filter가 `HttpException` 종류의 예외만 처리할 것을 명시

## Arguments host

- `catch()` 메서드 내 `exception` parameter는 현재 처리 중인 exception object, `host` parameter는 `ArgumentsHost` object
- `ArgumentsHost` object를 통해 `Request`, `Response` object에 접근 가능
- 향후 [execution context](https://docs.nestjs.com/fundamentals/execution-context) chapter에서 설명
- `ArgumentsHost`에 대한 자세한 정보는 [여기](https://docs.nestjs.com/fundamentals/execution-context)서 설명

## Binding filters

```js
// cats.controller.ts
@Post()
@UseFilters(new HttpExceptionFilter())
async create(@Body() createCatDto: CreateCatDto) {
  throw new ForbiddenException();
}
```

- `@Catch()` decorator와 비슷하게 `HttpExceptionFilter` instance를 `@UseFilter()` decorator에 넘김
- 아니면 아래와 같이 class 자체를 넘길 수도 있음

```js
// cats.controller.ts
@Post()
@UseFilters(HttpExceptionFilter)
async create(@Body() createCatDto: CreateCatDto) {
  throw new ForbiddenException();
}
```

> class를 넘기는 것이 NestJS로 하여금 전체 모듈에 걸쳐 인스턴스를 재사용하도록 해 메모리 사용을 줄이게 할 수 있어 권장됨

- exception filter는 서로 다른 레벨에서 scoping 가능, 위 예시는 method-scoped
- 아래 예시는 controller 내에 있는 모든 router handler에 exception filter를 적용(controller-scoped로 설정)

```js
// cats.controller.ts
@UseFilters(new HttpExceptionFilter())
export class CatsController {}
```

- 아래 예시는 전역 설정(global-scoped) filter 예시

```js
async function bootstrap() {
  const app = await NestFactory.create(AppModule);
  app.useGlobalFilters(new HttpExceptionFilter());
  await app.listen(3000);
}
bootstrap();
```

> `useGlobalFilters()` 메서드는 gateway나 hybrid 애플리케이션에 대해 filter 적용 불가

- global-scoped filter는 애플리케이션 전역에서 사용
- 각 모듈 컨텍스트 밖에서 이루어졌기 때문에 종속성 주입이 불가
- 종속성 주입을 위해선 아래와 같이 모듈에 직접 등록

```js
// app.module.ts
import { Module } from '@nestjs/common';
import { APP_FILTER } from '@nestjs/core';

@Module({
  providers: [
    {
      provide: APP_FILTER,
      useClass: HttpExceptionFilter,
    },
  ],
})
export class AppModule {}
```

## Catch everything

- 핸들링되지 않은 모든 예외를 잡고 싶다면 `@Catch()` decorator에 아무 매개변수도 넣지 않아야 함

```js
import {
  ExceptionFilter,
  Catch,
  ArgumentsHost,
  HttpException,
  HttpStatus,
} from '@nestjs/common';
import { HttpAdapterHost } from '@nestjs/core';

@Catch()
export class AllExceptionsFilter implements ExceptionFilter {
  constructor(private readonly httpAdapterHost: HttpAdapterHost) {}

  catch(exception: unknown, host: ArgumentsHost): void {
    // 특정 상황에서는 constructor 메서드에서 `httpAdapter`를 사용할 수 없어 여기서 이행
    const { httpAdapter } = this.httpAdapterHost;

    const ctx = host.switchToHttp();

    const httpStatus =
      exception instanceof HttpException
        ? exception.getStatus()
        : HttpStatus.INTERNAL_SERVER_ERROR;

    const responseBody = {
      statusCode: httpStatus,
      timestamp: new Date().toISOString(),
      path: httpAdapter.getRequestUrl(ctx.getRequest()),
    };

    httpAdapter.reply(ctx.getResponse(), responseBody, httpStatus);
  }
}
```

## Inheritance

- 보통 애플리케이션 요구사항에 따라 exception filter를 커스터마이징
- 내장된 기본 global exception filter를 extend해 특정 요소에 따라 override하고 싶은 경우, `BaseExceptionFilter`를 extend하여 상속된 `catch()` 메서드를 사용해 기본 필터에 예외 처리를 위임

```js
// all-exceptions.filter.ts
import { Catch, ArgumentsHost } from '@nestjs/common';
import { BaseExceptionFilter } from '@nestjs/core';

@Catch()
export class AllExceptionsFilter extends BaseExceptionFilter {
  catch(exception: unknown, host: ArgumentsHost) {
    super.catch(exception, host);
  }
}
```

> `BaseExceptionFilter`를 extend하는 method-scoped 또는 controller-scoped filter는 `new`를 사용해 인스턴스화하지 말고 프레임워크 단에서 자동으로 되도록 구현할 것

- global filter는 기본 filter를 extend할 수 있음
- 한 가지 방법은 커스텀 global filter를 인스턴스화할 때 `HttpAdapter` 참조를 inject

```js
async function bootstrap() {
  const app = await NestFactory.create(AppModule);

  const { httpAdapter } app.get(HttpAdapterHost);
  app.useGlobalFilters(new AllExceptionsFilter(httpAdapter));

  await app.listen(3000);
}
bootstrap();
```

- [다른 방법](https://docs.nestjs.com/exception-filters#binding-filters)은 `APP_FILTER` token을 사용
