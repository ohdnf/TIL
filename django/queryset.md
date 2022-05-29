# Django QuerySet

[Django Document](https://docs.djangoproject.com/en/3.0/ref/models/querysets/)

## When QuerySets are evaluated

- 반복
- slicing
  - 기본 (ex. `q[3:10]`): X
  - step이 들어간 경우(ex. `q[3:10:2]`): O
- `print()`
- `len()`
- `bool()`
