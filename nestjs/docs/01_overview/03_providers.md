# Providers

- provider는 Nest의 핵심 컨셉
- service, repository, factory, helper 등과 같은 많은 Nest 클래스들은 provider로 취급됨
- provider는 module로서 제공될 수 있음 -> 무수한 의존성 관계 생성 가능

## Services

- service는 데이터 저장과 조회를 담당, controller에서 사용
- Nest CLI(`nest g service cats`)로 생성 가능

```js
// cats.service.ts
import { Injectable } from '@nestjs/common';
import { Cat } from './interfaces/cat.interface';

@Injectable()
export class CatsService {
  private readonly cats: Cat[] = [];

  create(cat: Cat) {
    this.cats.push(cat);
  }

  findAll(): Cat[] {
    return this.cats;
  }
}
```

```js
// interfaces/cat.interface.ts
export interface Cat {
  name: string;
  age: number;
  breed: string;
}
```

- `@Injectable()` decorator는 클래스에 해당 클래스가 Nest IoC 컨테이너에 의해 관리된다는 메타데이터를 추가
- service는 아래와 같이 controller 안에서 class constructor를 통해 주입하여 사용
- `private` 구문을 통해 `catsService` 멤버를 선언과 동시에 초기화

```js
// cats.controller.ts
import { Controller, Get, Post, Body } from '@nestjs/common';
import { CreateCatDto } from './dto/create-cat.dto';
import { CatsService } from './cats.service';
import { Cat } from './interfaces/cat.interface';

@Controller('cats')
export class CatsController {
  constructor(private catsService: CatsService) {}

  @Post()
  async create(@Body() createCatDto: CreateCatDto) {
    this.catsService.create(createCatDto);
  }

  @Get()
  async findAll(): Promise<Cat[]> {
    return this.catsService.findAll();
  }
}
```

## Dependency injection

- NestJS는 종속성 주입으로 잘 알려진 강력한 디자인 패턴으로 구성
- NestJS에선 module들을 타입으로 이행하는 TypeScript 덕분에 종속성 관리가 쉬움
- 아래 예시에선 Nest가 `CatsService` 인스턴스를 생성하고 반환하며 `catsService`를 이행(또는 singleton의 경우, 다른 곳에서 요청된 적이 있다면 기존 인스턴스를 반환)
- 이 의존성은 이행되어 controller의 constructor에 넘겨짐(또는 지정된 속성에 할당됨)

```js
constructor(private catsService: CatsService) {}
```

## Scopes

- provider는 일반적으로 애플리케이션 라이프사이클과 동기화됨
- 애플리케이션이 시동되면 모든 의존성이 이행되고, 모든 provider들이 인스턴스화됨
- 애플리케이션이 종료되면 각 provider들은 소멸
- provider를 **request-scoped**로 설정 가능

## Custom providers

- Nest는 내부에 IoC(Inversion of Control) 컨테이너가 있어 provider 간 관계를 이행시킬 수 있음
- 이 기능은 앞서 언급한 의존성 주입 기능의 기초가 됨

## Optional providers

- 이행될 필요가 없는 종속성도 존재 가능(예: configuration object가 필요한 class이지만 전달받은 것이 없을 때 기본값이 사용되어야 함)
- provider가 선택적임을 나타내기 위해선 `@Optional()` decorator를 사용

```js
import { Injectable, Optional, Inject } from '@nestjs/common';

@Injectable()
export class HttpService<T> {
  constructor(@Optional() @Inject('HTTP_OPTIONS') private httpClient: T) {}
}
```

- 위 예에서는 `HTTP_OPTIONS`이라는 custom token을 포함한 custom provider를 사용

## Property-based injection

- 전술한 예제들은 모두 constructor-based injection을 사용(constructor 메서드를 사용해 provider가 주입되는 방식)
- 특정 상황에선 property-based injection이 필요할 수 있음
- 최상위 클래스에 하나 이상의 provider를 주입하는 경우 각 서브 클래스마다 `super()`를 호출해야 하므로 귀찮음
- 이를 피하기 위해 `@Inject()` decorator를 property 레벨에서 사용

```js
import { Injectable, Inject } from '@nestjs/common';

@Injectable()
export class HttpService<T> {
  @Inject('HTTP_OPTIONS')
  private readonly httpClient: T;
}
```

> class에서 다른 provider를 확장시키지 않는다면 constructor-based injection 사용을 적극 권장

## Provider registration

- 선언한 provider를 사용하기 위해선 Nest에 등록이 필요
- `app.module.ts` 모듈 파일에 아래와 같이 설정

```js
// app.module.ts
import { Module } from '@nestjs/common';
import { CatsController } from './cats/cats.controller';
import { CatsService } from './cats/cats.service';

@Module({
  controllers: [CatsController],
  providers: [CatsService],
})
export class AppModule {}
```

- Nest에서 이제 `CatsController` 클래스의 의존성을 이행 가능
- 지금까지 예제를 거쳐 아래와 같은 디렉토리 구조가 완성

```
src
├─cats
│  ├─dto
│  │  └─create-cat.dto.ts
│  ├─interfaces
│  │  └─cat.interface.ts
│  ├─cats.controller.ts
│  └─cats.service.ts
├─app.module.ts
└─main.ts
```

## Manual instantiation

- 지금까지 Nest가 의존성 이행과 관련한 사항들을 어떻게 다루는지 논의함
- 특정 상황에서는 내부 의존성 주입 시스템을 쓰지 않고 수동으로 provider들을 가져오거나 인스턴스화하는 작업이 필요할 수 있음
- 동적으로 provider를 인스턴스화하거나 기존 인스턴스를 가져오는 작업은 `Module` 설명을 참고
- `bootstrap()` 함수 내 provider에 접근하려면 `Standalone applications` 설명을 참고
