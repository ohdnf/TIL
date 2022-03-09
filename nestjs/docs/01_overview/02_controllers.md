# Controllers

## Routing

- controller는 클라이언트로부터 요청을 받고 응답을 반환하는 역할
- routing 메커니즘에서 어떤 controller가 어떤 Request를 받을지 제어
- controller는 class와 decorator로 이루어져 있음

> `nest g controller [name]`: controller 생성
> `nest g resource [name]`: Built-in Validation과 함께 CRUD 기능이 들어 있는 controller 생성

- controller에는 `@Controller()`라는 decorator가 필수
- 해당 decorator에는 경로 Prefix를 인자로 넘겨, 특정 경로의 routing을 해당 controller로 그룹화 가능

```js
// cats.controller.ts
import { Controller, Get } from '@nestjs/common';

@Controller('cats')
export class CatsController {
  @Get()
  findAll(): string {
    return 'This action returns all cats';
  }
}
```

- `findAll()` 메서드에 달려있는 `@Get()` decorator는 특정 endpoint로 들어오는 HTTP 요청(이 경우엔 GET)을 처리하는 핸들러를 생성
- `@Get()` decorator에도 경로에 추가될 수 있는 문자열을 인자로 넘길 수 있으며 위 예제에서 `@Get('profile')`로 쓸 경우 `GET /cats/profile`에 대한 HTTP 요청과 route mapping
- 기본 request 핸들러는 JavaScript Object나 Array에 대해 JSON serializaiton을 실행
- `@Res()` decorator를 이용해 라이브러리 특정 객체를 반환 가능(예: `findAll(@Res() response)`)

## Request object

- 요청 핸들러의 시그니처에 `@Req()` decorator를 추가하여 요청 객체에 접근 가능

```js
// cats.controller.ts
import { Controller, Get, Req } from '@nestjs/common';
import { Request } from 'express';

@Controller('cats')
export class CatsController {
  @Get()
  findAll(@Req() request: Request): string {
    return 'This action returns all cats';
  }
}
```

> 위 예시와 같이 `express` 타입 사용을 위해선 `@types/express` 패키지 설치 필요

- 요청 객체는 HTTP 요청을 나타내며 요청 쿼리 문자열, 매개변수, HTTP 헤더와 본문과 같은 속성을 가짐
- 아래와 같은 decorator들로 각 속성에 접근 가능

| decorator                 | property                           |
| ------------------------- | ---------------------------------- |
| `@Request()`, `@Req()`    | `req`                              |
| `@Response()`, `@Res()`\* | `res`                              |
| `@Next()`                 | `next`                             |
| `@Session()`              | `req.session`                      |
| `@Param(key?: string)`    | `req.params`, `req.params[key]`    |
| `@Body(key?: string)`     | `req.body`, `req.body[key]`        |
| `@Query(key?: string)`    | `req.query`, `req.query[key]`      |
| `@Headers(name?: string)` | `req.headers`, `req.headers[name]` |
| `@Ip()`                   | `req.ip`                           |
| `@HostParam()`            | `req.hosts`                        |

\* `@Res()` decorator를 핸들러에 주입하기 위해선 **Library-specific mode**를 지정해야 하며, `response` 객체에 대한 호출이 필요(예: `res.json(...)` 또는 `res.send(...)`) -> 아닐 경우 HTTP 서버가 응답 반환하기 위해 대기

## Resources

HTTP 요청 메서드에 대한 decorator들: `@Get()`, `@Post()`, `@Put()`, `@Delete()`, `@Patch()`, `@Options()`, `@Head()`, `@All()`

```js
// cats.controller.ts
import {
  Controller,
  Get,
  Query,
  Post,
  Body,
  Put,
  Param,
  Delete,
} from '@nestjs/common';
import { CreateCatDto, UpdateCatDto, ListAllEntities } from './dto';

@Controller('cats')
export class CatsController {
  @Post()
  create(@Body() createCatDto: CreateCatDto) {
    return 'This action adds a new cat';
  }

  @Get()
  findAll(@Query() query: ListAllEntities) {
    return `This action returns all cats (limit: ${query.limit} items)`;
  }

  @Get(':id')
  findOne(@Param('id') id: string) {
    return `This action returns a #${id} cat`;
  }

  @Put(':id')
  update(@Param('id') id: string, @Body() updateCatDto: UpdateCatDto) {
    return `This action updates a #${id} cat`;
  }

  @Delete(':id')
  remove(@Param('id') id: string) {
    return `This action removes a #${id} cat`;
  }
}
```

## Route wildcards

```js
@Get('ab*cd')
findAll() {
  return 'This route uses a wildcard';
}
```

- `ab*cd`는 `abcd`, `ab_cd`, `abecd` 등과 대응
- 또한 `?`, `+`, `*`, `()` 등이 사용될 수 있으며 Regular Expression의 기능과 대응

## Status code

- 기본 200 응답
- POST 요청의 경우 201 응답
- `@HttpCode(...)` decorator를 통해 커스터마이징 가능(`@nestjs/common` 패키지에서 `HttpCode` import 필요)

```js
@Post()
@HttpCode(204)
create() {
  return 'This action adds a new cat';
}
```

- 상황에 따라 다른 상태 코드 반환을 위해선 `@Res()` 주입을 통한 `response` 객체 사용 또는 에러 반환

## Headers

응답 헤더를 바꾸고 싶다면 `@Header()` decorator나 `res.header()`와 같은 응답 객체를 사용

```js
@Post()
@Header('Cache-Control', 'none')
create() {
  return 'This action adds a new cat';
}
```

## Redirection

- 특정 URL로 리다이렉트하기 위해선 `@Redirect()` decorator를 사용하거나 `res.redirect()`와 같은 응답 객체를 사용
- `@Redirect()`는 `url`과 `statusCode`라는 선택 인자를 수용
- Route 핸들러에서 아래와 같이 객체를 반환하여 **HTTP 상태 코드나 리다이렉트할 URL을 동적으로 지정 가능**

```js
@Get('docs')
@Redirect('https://docs.nestjs.com', 302)
getDocs(@Query('version') version) {
  if (version && version === '5') {
    return { url: 'https://docs.nestjs.com/v5/' };
  }
}
```

## Route parameters

- 요청 경로에 동적 데이터를 받기 위해서 사용
- `@Get()` decorator에 Route parameter를 선언하고 `@Param()` decorator를 사용해 접근 가능

```js
@Get(':id')
findOne(@Param() params): string {
  console.log(params.id);
  return `This action returns a #${params.id} cat`;
}
```

- 위의 예처럼 `id` parameter는 `params.id`를 참조해 접근 가능
- 또는 특정 parameter token을 decorator에 전달해 참조할 route parameter를 명시 가능

```js
@Get(':id')
findOne(@Param('id') id: string): string {
  return `This action returns a #${id} cat`;
```

## Sub-Domain Routing

- `@Controller` decorator에 `host` 옵션을 줘 특정 HTTP 호스트만을 요구하도록 설정 가능
- Express 설정에서만 사용 가능(Fastify 지원 불가)

```js
@Controller({ host: 'admin.example.com' })
export class Admincontroller {
  @Get()
  index(): string {
    return 'Admin page';
  }
}
```

- Route 경로에서 동적 변수를 받는 것처럼 `hosts` 옵션에서도 동적 변수 참조가 가능(아래 예시 참고)

```js
@Controller({ host: ':account.example.com' })
export class Admincontroller {
  @Get()
  getInfo(@HostParam('account') account: string) {
    return account;
  }
}
```

## Scopes

- DB connection pool, 전역 상태의 singleton 서비스 등이 요청 내에서 공유됨
- Node.js는 멀티쓰레드 무상태성 모델의 요청/응답 방식을 따르지 않으므로 singleton 인스턴스 사용은 안전함
- 그럼에도 불구하고 예외 존재 -> scope 제어 방법은 추후 설명

## Asynchronicity

- 데이터 추출은 대부분 비동기적
- Nest에선 아래와 같이 두 가지 방식으로 비동기 함수 처리

```js
@Get()
async findAll(): Promise<any[]> {
  return [];
}
```

```js
// RsJS의 observable streams을 반환하는 경우
@Get()
findAll(): Observable<any[]> {
  return of([]);
}
```

## Request payloads

- 요청 인자를 받기 위해 `@Body()` decorator를 사용
- 그전에 TypeScript Interface나 class를 사용해 DTO(Data Transfer Object)가 필요
- TypeScript Interface는 transpilation 과정에서 제거되므로 런타임 환경에서 참조 불가능
- Pipe 같은 기능으로 런타임 환경에서 변수의 Metadata에 접근 가능

```js
// create-cat.dto.ts
export class CreateCatDto {
  name: string;
  age: number;
  breed: string;
}
```

```js
// cats.controller.ts
@Post()
async create(@Body() createCatDto: CreateCatDto) {
  return 'This action adds a new cat';
}
```

- `ValidationPipe`를 사용해 method handler를 통해 전달받지 않아야 할 속성들을 걸러낼 수 있음

## Getting up and running

- `@Module()` decorator에 `controllers` 배열을 포함시켜야 mount 가능

```js
// app.module.ts
import { Module } from '@nestjs/common';
import { CatsController } from './cats/cats.controller';

@Module({
  controllers: [CatsController],
})
export class AppModule {}
```
