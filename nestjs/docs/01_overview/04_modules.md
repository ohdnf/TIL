### Modules

- module은 `@Module()` decorator가 붙은 class
- `@Module()` decorator는 Nest가 애플리케이션 구조를 구성할 수 있도록 메타데이터를 제공
- 애플리케이션에는 적어도 하나의 module(`root module`)이 존재
- `root module`은 Nest **애플리케이션 그래프**의 시작점 역할
  - 애플리케이션 그래프는 Nest에서 module, provider 관계, 의존성을 이행할 때 사용하는 내부 구조
- 이론적으로 `root module`은 하나이지만, 아닐 수도 있음
- module은 컴포넌트들을 구성하기 위한 가장 효과적인 방법
- 결과적으로 대부분의 애플리케이션은 다수의 module들을 사용하게 될 것이며, 각 module은 유사한 기능끼리 묶인 집합으로 캡슐화됨
- `@Module()` decorator는 아래와 같은 속성들을 지닌 하나의 객체
  - `providers`
    Nest 인젝터에 의해 인스턴스화되어 해당 module 내에서 공유되는 provider 집합
  - `controllers`
    해당 module에서 정의된 컨트롤러 집합
  - `imports`
    해당 module에서 필요로 하는 provider를 제공하는 module
  - `exports`
    해당 module에서 제공되는 provider의 부분 집합으로, 다른 module에서 사용 가능
- module은 기본적으로 provider들을 캡슐화
- 따라서 해당 module의 일부분이거나 다른 module에서 가져온 provider를 주입할 수 없음
- 어떤 module에서 내보내진 provider는 해당 module의 public interface 또는 API로 생각

#### Feature modules

- 앞서 만든 `CatsController`와 `CatsService`는 같은 애플리케이션 도메인에 속하므로 같은 feature module에 넣는 것이 바람직
- feature module은 특정한 기능과 관련된 코드들을 모아놓음으로써 코드를 정돈되고 명확하게 구분지을 수 있도록 도와줌
- 애플리케이션이나 팀 크기가 커져도 복잡한 코드를 관리하고, [SOLID](<https://ko.wikipedia.org/wiki/SOLID_(객체_지향_설계)>) 원칙을 지키는데 용이
- 아래는 `CatsModule` 예시

```js
// cats/cats.module.ts
import { Module } from '@nestjs/common';
import { CatsController } from './cats.controller';
import { CatsService } from './cats.service';

@Module({
  controllers: [CatsController],
  providers: [CatsService],
})
export class CatsModule {}
```

> `nest g module cats` CLI로 손쉽게 module을 생성 가능

- `cats.module.ts` 파일에 `CatsModule`을 정의하고, `cats` 디렉토리에 관련된 모든 걸 이동
- root module인 `AppModule`에 `CatsModule`을 import

```js
// app.module.ts
import { Module } from '@nestjs/common';
import { CatsModule } from './cats/cats.module';

@Module({
  imports: [CatsModule],
})
export class AppModule {}
```

- 이제 디렉토리 구조는 아래와 동일

```
src
├─cats
│  ├─dto
│  │  └─create-cat.dto.ts
│  ├─interfaces
│  │  └─cat.interface.ts
│  ├─cats.controller.ts
│  ├─cats.module.ts
│  └─cats.service.ts
├─app.module.ts
└─main.ts
```

#### Shared modules

- Nest에서 module은 기본적으로 **singleton**이기 때문에 어떤 provider의 instance든 서로 다른 module에서 공유 가능
- 생성된 module은 어디서든 재사용 가능하므로 모든 module은 shared module
- module을 재사용하기 위해선 **export** 필요

```js
// cats.module.ts
import { Module } from '@nestjs/common';
import { CatsController } from './cats.controller';
import { CatsService } from './cats.service';

@Module({
  controllers: [CatsController],
  providers: [CatsService],
  exports: [CatsService],
})
export class CatsModule {}
```

- 이제 `CatsModule`을 import한 module은 `CatsService`에 접근 가능

#### Module re-exporting

- 내부 provider를 export할 수도 있지만, import한 module을 다시 export하는 것도 가능

```js
@Module({
  imports: [CommonModule],
  exports: [CommonModule],
})
export class CoreModule {}
```

#### Dependency injection

- module class는 provider 주입도 가능(예: 설정 목적)

```js
// cats.module.ts
import { Module } from '@nestjs/common';
import { CatsController } from './cats.controller';
import { CatsService } from './cats.service';

@Module({
  controllers: [CatsController],
  providers: [CatsService],
})
export class CatsModule {
  constructor(private catsService: CatsService) {}
}
```

- 그러나, module class 그 자체는 상호 참조로 인해 provider로써 주입될 수 없음

#### Global modules

- 같은 module을 모든 곳에 import해야 할 경우 `@Global` decorator를 사용

```js
// cats.module.ts
import { Module, Global } from '@nestjs/common';
import { CatsController } from './cats.controller';
import { CatsService } from './cats.service';

@Global()
@Module({
  controllers: [CatsController],
  providers: [CatsService],
  exports: [CatsService],
})
export class CatsModule {}
```

> 모든 것을 global로 만드는 것은 좋은 디자인이 아님
> global module은 boilerplate의 크기를 줄이는 용도
> `imports` 배열에 넣는 것을 우선적으로 고려

#### Dynamic modules

- Nest module 시스템에는 **dynamic module**이라는 강력한 기능이 존재
- dynamic module을 통해 동적으로 customize 가능한 module을 생성하고, provider를 등록 및 설정 가능

```js
import { Module, DynamicModule } from '@nestjs/common';
import { createDatabaseProviders } from './database.providers';
import { Connection } from './connection.provider';

@Module({
  providers: [Connection],
})
export class DatabaseModule {
  static forRoot(entities = [], options?): DynamicModule {
    const providers = createdDatabaseProviders(options, entities);
    return {
      module: DatabaseModule,
      providers: providers,
      exports: providers,
    };
  }
}
```

> `forRoot()` 메서드를 통해 dynamic module을 동기적 또는 비동기적으로 반환 가능

- 위 module은 `Connection` provider를 선언하면서, `forRoot()` 메서드에 전달된 `entities`와 `options` 객체에 따라 repositories와 같은 provider들을 추가로 등록
- dynamic module에 의해 반환된 property들은 override되는 것이 아니라 `@Module()` decorator에 정의된 module metadata를 extend
- 이것이 static하게 선언된 `Connection` provider와 dynamic하게 생성된 repository provider가 **함께** module에서 export될 수 있는 이유
- dynamic module을 global scope에 등록하고 싶다면 `global` 속성을 `true`로 설정

```js
{
  global: true,
  module: DatabaseModule,
  providers: providers,
  exports: providers,
}
```

- `DatabaseModule`은 아래와 같이 import되고 설정

```js
import { Module } from '@nestjs/common';
import { DatabaseModule } from './database/database.module';
import { User } from './users/entities/user.entity';

@Module({
  imports: [DatabaseModule.forRoot([User])],
})
export class AppModule {}
```

- dynamic module을 다시 export하고 싶다면 exports 배열에서 `forRoot()` 메서드를 생략

```js
import { Module } from '@nestjs/common';
import { DatabaseModule } from './database/database.module';
import { User } from './users/entities/user.entity';

@Module({
  imports: [DatabaseModule.forRoot([User])],
  exports: [DatabaseModule],
})
export class AppModule {}
```
