## DB 이전하기

### `dumpdata` & `loaddata`

```shell
$ python manage.py dumpdata > data.json
```

> 위 명령어로 현재 생성된 models 테이블에 따른 데이터들을 모두 JSON으로 변환

### `fixture`

```shell
$ python manage.py loaddata data.json
```

> 위 명령어로 데이터를 불러오려고 하면 위치 에러가 발생한다. templates를 가져올 때와 비슷하게 fixtures 폴더가 필요하다.

```shell
$ mkdir articles/fixtures/articles
$ python manage.py loaddata articles/data.json
```

> 위 명령어로 저장된 데이터들을 가져왔던 데이터의 테이블, 스키마와 동일한 DB 환경이어야 (새로운) 프로젝트에 가져올 수 있다. 즉 modeling과 migrate가 되어있어야 한다는 얘기.

