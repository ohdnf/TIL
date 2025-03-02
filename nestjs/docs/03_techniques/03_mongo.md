# Mongo

- NestJS에서 MongoDB 데이터베이스와 연동할 수 있는 방법은 두 가지
- `TypeORM`을 사용하거나, `Mongoose`를 사용하는 것
- 본 챕터에서는 `@nestjs/mongoose` 패키지를 사용

```
npm i @nestjs/mongoose mongoose
```

```js
// app.module.ts
import { Module } from '@nestjs/common';
import { MongooseModule } from '@nestjs/mongoose';

@Module({
  imports: [MongooseModule.forRoot('mongodb://localhost/nest')],
})
export class AppModule {}
```

`forRoot()` 메서드는 `mongoose.connect()`와 동일한 설정 객체를 인자로 받는다.

## Model injection
