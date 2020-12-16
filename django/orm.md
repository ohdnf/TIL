# Django ORM



## Making queries

https://docs.djangoproject.com/en/3.1/topics/db/queries/



### Creating objects



### Saving changes to objects



### Retrieving objects

#### Retrieving specific objects with filters

`filter(**kwargs)`

주어진 조건에 맞는 객체들을 포함한 `QuerySet`을 반환합니다. 반환된 `QuerySet`은 변수에 할당될 수 있고 데이터베이스와는 무관한 독립된 데이터로 생성이 됩니다.

#### QuerySets are lazy

`QuerySet`을 생성하는 일은 데이터베이스 활동과 아무런 관련이 없습니다. 모델 objects에 무수히 많은 `filter`를 걸 수 있으며, Django는 `QuerySet`이 **평가되기 전까지 쿼리를 실행하지 않습니다.**

```shell
>>> q = Entry.objects.filter(headline__startswith="What")
>>> q = q.filter(pub_date__lte=datetime.date.today())
>>> q = q.exclude(body_text__icontains="food")
>>> print(q)
```

위의 예시는 데이터베이스에 세 번의 쿼리문을 보낼 것 같지만, `print(q)` 명령 전까지는 실행되지 않습니다. `QuerySet`의 결과는 *요청*되기 전까지는 데이터베이스로부터 가져오지 않습니다.

#### Retrieving a single object with `get()`

`filter()`가 쿼리를 만족하는 하나의 객체라도 있으면 `QuerySet`을 만들어 주는 것에 비해, `get()` 메소드는 쿼리를 통해 가져올 객체가 단 하나 존재할 때 바로 그 객체를 직접 반환받을 수 있는 방법입니다.

- `filter()`를 통해 반환받을 `QuerySet`에 객체가 하나뿐이어도, `QuerySet` 안에 담겨서 반환됨
- `get()`은 객체 하나만 반환해줌

쿼리를 만족하는 객체가 존재하지 않는다면 `get()`은 `DoesNotExist` exception을 발생시킵니다. 쿼리를 만족하는 객체가 여러 개라면, `MultipleObjectsReturned` Exception을 발생시킵니다.

