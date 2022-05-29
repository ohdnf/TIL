# Using models

## 모델 정의

> 모델 정의는 `models.py`에, `django.db.models.Model`을 상속하여 정의한다.

전형적인 `models.py`는 다음과 같다.

```python
from django.db import models

class MyModelName(models.Model):
    """A typical class defining a model, derived from the Model class."""

    # Fields
    my_field_name = models.CharField(max_length=20, help_text='Enter field documentation')
    ...

    # Metadata
    class Meta:
        ordering = ['-my_field_name']

    # Methods
    def get_absolute_url(self):
        """Returns the url to access a particular instance of MyModelName."""
        return reverse('model-detail-view', args=[str(self.id)])

    def __str__(self):
        """String for representing the MyModelName object (in Admin site etc.)."""
        return self.my_field_name
```

### Fields

DB의 Column 영역

#### Common field arguments

- `help_text`
- `verbose_name`
- `default`
- `null`
- `blank`
- `choices`
- `primary_key`

#### Common field types

- `CharField`
- `TextField`
- `IntegerField`
- `DateField`
- `EmailField`
- `FileField`, `ImageField`
- `AutoField`
- `ForeignKey`
- `ManyToManyField`

### Metadata

- `ordering`
- `verbose_name`
- `verbose_name_plural`

### Methods

- `__str__()`

  모든 모델에는 최소한 들어가야 하는 Python 클래스 메소드

  ```python
  def __str__(self):
      return self.field_name
  ```

- `get_absolute_url()`

  ```python
  def get_absolute_url(self):
      """Returns the url to access a particular instance of the model."""
      return reverse('model-detail-view', args=[str(self.id)])
  ```

## 모델 관리

### 모델 생성과 변경

instance를 선언하고 `save()`를 호출한다.

```python
# Create a new record using the model's constructor.
record = MyModelName(my_field_name="Instance #1")

# Save the object into the database.
record.save()
```

필드 값을 변경한 후에도 `save()`를 호출한다.

```python
# Access model field values using Python attributes.
print(record.id) # should return 1 for the first record.
print(record.my_field_name) # should print 'Instance #1'

# Change record by modifying the fields, then calling save().
record.my_field_name = "New Instance Name"
record.save()
```

### 레코드 검색

모델의 `objects` 속성을 사용해 특정 기준에 맞는 레코드를 검색할 수 있다.

> `title`과 `genre` 필드를 가진 `Book` 모델을 예시로 들어보자.
> `genre` 필드는 `name` 필드를 가진 또 하나의 모델이다.

Django에서는 `objects.all()` 구문을 사용해 해당 모델의 모든 레코드를 `QuerySet`으로 불러올 수 있다.

```python
all_books = Book.objects.all()
```

Django의 `filter()`는 특정 기준에 부합하는 **텍스트** 혹은 **숫자** 필드 조건을 걸 수 있는 메소드이다. 예를 들어, `wild`를 포함한 제목을 가진 책들을 골라서 그 갯수를 셀 때 다음과 같이 할 수 있다.

```python
wild_books = Book.objects.filter(title__contains='wild')
number_wild_books = wild_books.count()
```

매개변수는 `field_name__match_type`과 같은 형식이고 `icontains`, `iexact`, `exact`, `in`, `gt`, `startswith` 등이 있다.

관계형의 필드(`ForeignKey`, `ManyToMany` 필드)에서 조건을 걸고 싶을 땐 `model__field__match_type`과 같이 쓴다. 예를 들어 `genre` 필드의 `name`이 `fiction`을 포함하고 있는 조건으로 검색하고 싶다면 다음과 같이 쓴다.

```python
# Will match on: Fiction, Science fiction, non-fiction etc.
books_containing_genre = Book.objects.filter(genre__name__icontains='fiction')
```

> Django Docs
>
> - [Model field reference](https://docs.djangoproject.com/en/3.1/ref/models/fields/)
> - [Making queries](https://docs.djangoproject.com/en/2.2/topics/db/queries/)
> - [QuerySet API Reference](https://docs.djangoproject.com/en/2.2/ref/models/querysets/#field-lookups)
