# Pipes

- pipe는 `@Injectable()` decorator로 annotate된 class로, `PipeTransform` interface를 implement함
- pipe를 사용하는 전형적인 두 가지 예:
  - **transformation**: 입력 데이터를 원하는 형태로 변형
  - **validation**: 입력 데이터가 유효한지 검사하고 유효하지 않다면 예외 발생 처리
- pipe는 [controller route handler](https://docs.nestjs.kr/controllers#route-parameters)에서 처리되는 `arguments`에 대해 작동
- Nest는 method가 실행되기 바로 전에 해당 method의 arguments를 처리할 수 있도록 pipe를 위치시킴
- 이 때 모든 transformation이나 validation이 실행되고, route handler는 처리된 arguments를 넘겨받음

> pipe는 exception zone 안에서 실행되므로 pipe에서 발생하는 예외는 exception layer에 의해 handle됨
> 이는 곧 pipe에서 예외가 발생하면 controller method는 실행되지 않는다는 것을 의미
> 따라서 pipe를 사용하는 것은 시스템 바운더리로 들어오는 외부 데이터를 검증하는 가장 좋은 방법 중 하나

## Built-in pipes

- NestJS에는 다음과 같은 내장 pipe가 존재(`@nestjs/common` package에서 불러옴)
  - ValidationPipe
  - ParseIntPipe
  - ParseFloatPipe
  - ParseBoolPipe
  - ParseArrayPipe
  - ParseUUIDPipe
  - ParseEnumPipe
  - DefaultValuePipe
- 기타 사항은 [링크(https://docs.nestjs.kr/pipes#built-in-pipes)] 참고

## Binding pipes

- pipe를 사용하기 위해선 적절한 맥락에 pipe를 바인딩하는 것이 필요
- `ParseIntPipe`를 예로 들면, 특정 route handler method에 pipe를 연결시켜 method가 실행되기 전에 pipe가 작동하도록 해야 함
- 아래와 같이 method parameter level에서 pipe를 바인딩시킬 수 있음

```js
@Get(':id')
async findOne(@Param('id', ParseIntPipe) id: number) {
  return this.catsService.findOne(id);
}
```

- 이를 통해 `findOne()` method에서 받을 인자들이 number거나 아닐 경우 예외 처리될 것을 보장
- 예를 들어, 아래처럼 라우트를 호출할 경우

```sh
GET localhost:3000/abc
```

- 다음과 같이 예외가 발생

```json
{
  "statusCode": 400,
  "message": "Validation failed (numeric string is excepted)",
  "error": "Bad Request"
}
```

- 예외는 `findOne()` method가 실행되는 것을 방지
- 위 예에서는 instance가 아닌 class를 넘겨 framework로 하여금 instantiation의 책임을 전가
- 옵션을 추가해 내장 pipe를 커스터마이징하고 싶다면 아래와 같이 in-place instance로 넘기면 됨

```js
@Get(':id')
async findOne(
  @Param('id', new ParseIntPipe({ errorHttpStatusCode: HttpStatus.NOT_ACCEPTABLE }))
  id: number,
) {
  return this.catsService.findOne(id);
}
```

- 다른 transformation pipe(모든 **Parse\*** pipe)도 유사하게 작동
- route parameter, query string parameter, request body value 모두 이런 pipe들로 유효성 검증 가능
- query string의 경우 아래와 같이 pipe 사용

```js
@Get()
async findOne(@Query('id', ParseIntPipe) id: number) {
  return this.catsService.findOne(id);
}
```

- route parameter의 string이 UUID인지 유효성 검증하는 예:

```js
@Get(':uuid')
async findOne(@Param('uuid', new ParseUUIDPipe()) uuid: string) {
  return this.catsService.findOne(uuid);
}
```

> `ParseUUIDPipe()`는 UUID 버전 3~5를 파싱하므로 특정 버전의 UUID를 검증하고 싶다면 옵션으로 넘겨야 함

- validation pipe는 `Parse*` pipe와는 약간 다름

## Custom pipes

- NestJS에서 사용자화 pipe도 구현 가능

```js
// validation.pipe.ts
import { PipeTransform, Injectable, ArgumentMetadata } from '@nestjs/common';

@Injectable()
export class ValidationPipe implements PipeTransform {
  transfrom(value: any, metadata: ArgumentMetadata) {
    return value;
  }
}
```

> `PipeTransform<T, R>`은 모든 pipe가 implement해야 하는 generic interface.
> `T`는 `transform()` method의 입력값인 `value`의 타입을, `R`은 반환값 타입을 나타냄

- 모든 pipe는 `transform()` method를 구현해 `PipeTransform` interface의 조건을 만족시켜야 함
- `transform()`에는 두 가지 매개변수가 존재
  - `value`
  - `metadata`
- `value`는 현재 처리중인 method 인자(route handling method로 가기 전), `metadata`는 현재 처리중인 method 인자의 metadata
- metadata 객체는 아래 속성들을 가짐

```js
export interface ArgumentMetadata {
  type: 'body' | 'query' | 'param' | 'custom';
  metatype?: Type<unknown>;
  data?: string;
}
```

- 위 속성들은 현재 처리중인 인자를 설명함
  - `type`은 인자가 `@Body()`, `@Query()`, `@Param()` 또는 custom parameter인지 가리킴
  - `metatype`은 타입(예: `String`)을 나타냄. 참고: vanilla JS를 사용하거나, route handler method signature에 타입 설정을 빠뜨리면 `undefined`로 전달
  - `data`는 decorator에 전달되는 문자열(예: `@Body('string')`). decorator의 괄호가 비어있으면 `undefined`

> TypeScript interface는 transpilation 중 사라짐. 따라서 만약 method parameter의 타입이 class가 아닌 interface로 지정되어 있다면 `metatype`는 `Object`가 됨

## Schema based validation

- `CatsController`의 `create()` method를 실행하기 전에 POST body 객체가 유효한지 검증하는 validation pipe

```js
@Post()
async create(@Body() createCatDto: CreateCatDto) {
  this.catsService.create(createCatDto);
}
```

```js
// create-cat.dto.ts
export class CreateCatDto {
  name: string;
  age: number;
  breed: string;
}
```

- `create()` method로 들어오는 body 객체가 `createCatDto` 객체의 세 속성을 갖추도록 유효함을 보장받고 싶음
- route handler method 안에서 유효성 검증을 할 수도 있지만 이것은 Single Responsibility Rule(SRP)를 깨는 것
- validator class를 만들어 검증하도록 할 수도 있지만 모든 method마다 이러한 validator를 호출하도록 기억해내야 함
- validation middleware를 만들 수도 있지만 application을 통틀어 모든 context를 만족시킬 generic middleware를 만드는 것은 불가능. 이것은 middleware가 **실행 컨텍스트**를 모르기 때문

## Object schema validation

- Schema-based validation은 객체를 검증하는 DRY한 방법 중 하나
- `Joi` 라이브러리를 사용해 schema를 생성([Github](https://github.com/sideway/joi))

```sh
npm install --save joi
npm install --save-dev @types/joi
```

- 아래 코드엔 `constructor`의 인자로 schema 하나를 받는 class를 생성함
- `schema.validate()` method로 유효성 검증
- 이전에 언급한 것처럼, **validation pipe**는 value를 그대로 반환하거나 예외를 발생시킴

```ts
import {
  PipeTransform,
  Injectable,
  ArgumentMetadata,
  BadRequestException,
} from '@nestjs/common';
import { ObjectSchema } from 'joi';

@Injectable()
export class JoiValidationPipe implements PipeTransform {
  constructor(private schema: ObjectSchema) {}

  transform(value: any, metadata: ArgumentMetadata) {
    const { error } = this.schema.validate(value);
    if (error) {
      throw new BadRequestException('Validation failed');
    }
    return value;
  }
}
```

## Binding validation pipes

- 위에서 본 transformation pipe 연동 방법처럼 validation pipe 연동 방법도 간단
- 이번 예에선 method call level에서 pipe를 연동, `JoiValidationPipe`를 사용하기 위해 아래 세 단계 실행
  1. `JoiValidationPipe`의 인스턴스 생성
  2. pipe의 class constructor에 컨텍스트 지정 Joi schema를 넘김
  3. method에 pipe를 연동

```js
@Post()
@UsePipes(new JoiValidationPipe(createCatSchema))
async create(@Body() createCatDto: CreateCatDto) {
  this.catsService.create(createCatDto);
}
```

> `@UsePipes()` decorator는 `@nestjs/common` 패키지에서 가져옴

## Class validator

> 본 섹션의 기법들은 TypeScript를 필요로 하며, 애플리케이션이 vanilla JavaScript로 쓰여졌을 경우 사용 불가

- Nest는 decorator-based validation을 가능하게 하는 `class-validator` 라이브러리도 사용가능([Github](https://github.com/typestack/class-validator))
- Decorator-based validation은 Nest의 **Pipe**가 `metatype` 속성에 접근하는 기능과 결합하여 매우 강력한 유효성 검증을 제공

```sh
npm i --save class-validator class-transformer
```

- 위 라이브러리를 설치하면 `CreateCatDto` 클래스에 몇가지 decorator들을 추가할 수 있음
- 이로써 POST body 객체를 위한 분리된 유효성 검증 클래스를 추가하는 것이 아닌 기존의 유일한 클래스를 계속 쓸 수 있음

```js
// create-cat.dto.ts
import { IsString, IsInt } from 'class-validator';

export class CreateCatDto {
  @IsString()
  name: string;

  @IsInt()
  age: number;

  @IsString()
  breed: string;
}
```

- 이 annotation들을 사용하는 `ValidationPipe`를 아래와 같이 생성

```js
// validation.pipe.ts
import { PipeTransform, Injectable, ArgumentMetadata, BadRequestException } from '@nestjs/common';
import { validate } from 'class-validator';
import { plainToClass } from 'class-transformer';

@Injectable()
export class ValidationPipe implements PipeTransform<any> {
  async transform(value: any, { metatype }: ArgumentMetadata) {
    if (!metatype || !this.toValidate(metatype)) {
      return value;
    }
    const object = plainToClass(metatype, value);
    const errors = await validate(object);
    if (errors.length > 0) {
      throw new BadRequestException('Validation failed');
    }
    return value;
  }

  private toValidate(metatype: Function): boolean {
    const types: Function[] = [String, Boolean, Number, Array, Object];
    return !types.includes(metatype);
  }
}
```

- `transform()` method가 `async`인 것에 주목
  - Nest는 동기/비동기 pipe 모두를 지원
  - class-validator의 validation들 중 어떤 것은 (Promise를 사용한) 비동기일 수 있음
- `ArgumentMetada`의 `metatype` field를 객체 분해하여 `metatype` 매개변수에 할당
- `toValidate()`는 현재 인자가 native JavaScript 타입일 경우 validation을 우회하도록 하는 함수
- `plainToClass()` 함수는 plain JavaScript 인자 객체를 타입 객체로 만들어 validation을 적용할 수 있게 만듦
  - 이걸 하는 이유는 네트워크 요청을 통해 deserialize된 POST body 객체는 타입 정보가 없기 때문
  - 또한 Class-validator는 DTO에서 정의한 validation decorator를 필요로 하기 때문에 이러한 transformation이 필수
- 앞서 언급한 것처럼 **validation pipe**는 value를 그대로 반환하거나 예외를 발생시킴
- pipe는 parameter-scoped, method-scoped, controller-scoped, global-scoped일 수 있음

```js
// cats.controller.ts
@Post()
async create(
  @Body(new ValidationPipe()) createCatDto: CreateCatDto,
) {
  this.catsService.create(createCatDto);
}
```

- 위와 같이 parameter-scoped pipe는 특정 parameter에만 유효성 검증 로직을 적용할 때 유용

## Global scoped pipes

- 작성한 `ValidationPipe`가 generic하게 만들어졌으므로, global-scoped pipe로 설정해 모든 route handler에 적용되게 함으로서 그 유용성을 체감 가능

```js
// main.ts
async function bootstrap() {
  const app = await NestFactory.create(AppModule);
  app.useGlobalPipes(new ValidationPipe());
  await app.listen(3000);
}
bootstrap();
```

- 의존성 주입 관점에서 global pipe는 모든 모듈 밖에서 `useGlobalPipes()`를 통해 등록되어 모듈 컨텍스트 밖에서 바인딩되기 때문에 의존성 주입이 불가능
- 이 문제를 해결하기 위해선 아래와 같이 모듈에 직접적으로 설정

```js
// app.module.ts
import { Module } from '@nestjs/common';
import { APP_PIPE } from '@nestjs/core';

@Module({
  providers: [
    {
      provide: APP_PIPE,
      useClass: ValidationPipe,
    },
  ],
})
export class AppModule {}
```

> pipe에 의존성 주입을 하기 위해 사용하는 이러한 접근은 적용된 모듈과 관계없이 pipe가 global이란 것에 주목해야 함

## The built-in ValidationPipe

- Nest에서 `ValidationPipe`를 기본 제공하므로 generic validation pipe를 직접 만들 필요는 없음
- 내장된 `ValidationPipe`는 우리가 작성한 예시보다 더 많은 옵션을 제공

## Transformation use case

- Validation이 유일한 커스텀 pipe는 아님
- 앞서 pipe는 입력 데이터를 우리가 원하는 형태로 transform해줄 수 있음을 언급함
- 이것은 `transform` 함수로부터 반환받은 값이 완벽히 인자의 값을 override하기 때문
- 클라이언트로부터 받은 데이터에 변경이 필요하거나 missing data의 기본값을 주기 위해 pipe를 사용할 수 있음
- **transformation pipe**는 클라이언트 요청과 요청 핸들러 사이에 위치해 이런 기능을 수행
- 아래는 `ParseIntPipe`를 모방한 custom transformation pipe로 문자열을 정수로 파싱

```js
// parse-int.pipe.ts
import {
  PipeTransform,
  Injectable,
  ArgumentMetadata,
  BadRequestException,
} from '@nestjs/common';

@Injectable()
export class ParseIntPipe implements PipeTransform<string, number> {
  transform(value: string, metadata: ArgumentMetadata): number {
    const val = parseInt(value, 10);
    if (isNaN(val)) {
      throw new BadRequestException('Validation failed');
    }
    return val;
  }
}
```

- 위 pipe를 아래와 같이 특정 param에 선택적으로 적용 가능

```js
@Get(':id')
async findOne(@Param('id', new ParseIntPipe()) id) {
  return this.catsService.findOne(id);
}
```

- 또 다른 예는 요청에 들어있는 id를 가지고 데이터베이스에서 존재하는 user 엔티티를 조회

```js
@Get(':id')
findOne(@Param('id', UserByIdPipe) userEntity: UserEntity) {
  return userEntity;
}
```

- pipe 구현은 안하겠지만 주목해야할 것은 모든 다른 transformation pipe와 마찬가지로, (`id`라는) 입력값을 받아 (`UserEntity`라는) 출력값을 반환한다는 점
- 이렇게 boilerplate 코드를 추상화하는 방법으로 코드를 더욱 declarative하고 DRY하게 만들 수 있음

## Providing defaults

- `DefaultValuePipe`를 사용해 기본값을 제공 가능

```js
@Get()
async findAll(
  @Query('activeOnly', new DefaultValuePipe(false), ParseBoolPipe) activeOnly: boolean,
  @Query('page', new DefaultValuePipe(0), ParseIntPipe) page: number,
) {
  return this.catsService.findAll({ activeOnly, page });
}
```
