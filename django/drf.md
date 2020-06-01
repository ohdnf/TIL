# Django REST Framework

## DRF로 CRUD하는 이유
0. Money
1. UX 안 좋으면 => User X => Money X
    - Data => Tells what human want
    - Mobile application(web)
    - churn(이탈율)
    - ~~Adobe Flash~~ ==> JavaScript
2. Template X, Data만 빠르게

==> javascript & frameworks 분리

## Web APIs for Django

- [django-rest-framework](https://www.django-rest-framework.org/)
- [GitHub](https://github.com/encode/django-rest-framework)

### Install

```shell
$ pip install djangorestframework
```

```py
# settings.py

INSTALLED_APP = [
    ...
    'rest_framework',
]
```

## Serializer

> 직렬화, `Object(언어, 데이터베이스) => String(JSON)`하는 것

- 포맷의 변환 (데이터 전송/이동)

dict => JSON (stringify)    # 직렬화
JSON => dict (parse)        # 역직렬화

### `serializers.py`

```py
# serializers.py

from rest_framework import serializers
from .models import Article

class ArticleSerializer(serializers.ModelSerializer):
    class Meta:
        model = Article
        fields = '__all__'

class ArticleIndexSerializer(serializers.ModelSerializer):
    class Meta:
        model = Article
        fields = ('id', 'title', 'created_at')

class ArticleDetailSerializer(serializers.ModelSerializer):
    class Meta:
        model = Article
        fields = ('id', 'title', 'content', 'created_at', 'updated_at')
```

### `views.py`

```py
# views.py

from rest_framework.response import Response    # rest_framework의 serializer를 리턴하기 위한 클래스
from rest_framework.decorators import api_view  # django rest framework로 동작하는 view 함수에 반드시 필요한 데코레이터

from .models import Article
from .serializers import ArticleSerializer, ArticleIndexSerializer, ArticleDetailSerializer

@api_view(['GET'])
def index(request):
    articles = Article.objects.all()
    serializer = ArticleIndexSerializer(articles, many=True)     # many=True 옵션은 쿼리셋일 때 설정
    return Response(serializer.data)

 @api_view(['GET'])
 def detail(request, pk):
    article = get_object_or_404(Article, pk=pk)
    serializer = ArticleDetailSerializer(article)
    return Response(serializer.data)

```