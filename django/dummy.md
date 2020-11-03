# Django Dummy 데이터 생성하기

> 테스트를 하기 위해 대량의 데이터를 만들 때 유용한 방법을 알아보자.



## Factory Boy 사용하기

> 공장처럼 데이터를 찍어낸다고 해서 Factory Boy라고 이름을 지은 듯하다.
>
> 참고 자료
>
> - [Factory Boy 공식 문서](https://factoryboy.readthedocs.io/en/latest/index.html)
> - [How to generate lots of dummy data for your Django app](https://mattsegal.dev/django-factoryboy-dummy-data.html)
> - [Faker 공식 문서](https://faker.readthedocs.io/en/master/index.html)



### 폴더/파일 구조

```
📁mysite
├─📁mysite
├─📁posts
│    ├─📁management
│    │    ├─📁management
│    │    │    ├─__init__.py
│    │    │    └─setup_test_data.py
│    │    └─__init__.py
│    ├─...
│    ├─factories.py
│    ├─models.py
│    └─...
├─...
├─manage.py
└─requirements.txt
```



### `factories.py`

```python
import factory
import factory.fuzzy
from factory.django import DjangoModelFactory

from users.models import CustomUser
from games.models import Tag, Source, GameHistory


CATEGORIES = [x[0] for x in Source.TYPE_CHOICES]


class UserFactory(DjangoModelFactory):
    class Meta:
        model = CustomUser

    email = factory.Faker('email')
    username = factory.Faker('name')


class TagFactory(DjangoModelFactory):
    class Meta:
        model = Tag
    
    content = factory.Faker('color_name')


class SourceFactory(DjangoModelFactory):
    class Meta:
        model = Source
    
    category = factory.fuzzy.FuzzyChoice(CATEGORIES)
    content = factory.Faker('paragraph', nb_sentences=3, variable_nb_sentences=True)
    length = factory.Faker('random_number')
    difficulty = factory.Faker('random_digit_not_null')


class GameHistoryFactory(DjangoModelFactory):
    class Meta:
        model = GameHistory
    
    game_time = factory.Faker('time')
    precision = factory.Faker('random_int', min=0, max=100)
    typo = factory.Faker('json')
    score = factory.Faker('random_int', min=0, max=100)
    player = factory.SubFactory(UserFactory)
    source = factory.SubFactory(SourceFactory)

```



### `setup_test_data.py`

```python
import random

from django.db import transaction
from django.core.management.base import BaseCommand, CommandError

from users.models import CustomUser
from games.models import Tag, Source, GameHistory
from games.factories import (
    UserFactory,
    TagFactory,
    SourceFactory,
    GameHistoryFactory
)


NUM_USERS = 40
NUM_TAGS = 5
NUM_SOURCES = 10
TAGS_PER_SOURCE = 3
NUM_GAME_HISTORIES = 20


class Command(BaseCommand):
    help = 'Generate test data'

    @transaction.atomic
    def handle(self, *args, **kwargs):
        self.stdout.write("Deleting old data...")
        models = [CustomUser, Tag, Source, GameHistory]
        for m in models:
            m.objects.all().delete()

        self.stdout.write("Creating new data...")
        # Create all the users
        users = []
        for _ in range(NUM_USERS):
            user = UserFactory()
            users.append(user)
        
        # Create all the tags
        tags = []
        for _ in range(NUM_TAGS):
            tag = TagFactory()
            tags.append(tag)
        
        # Create all the Sources
        sources = []
        for _ in range(NUM_SOURCES):
            source = SourceFactory()
            # Add likers and subscribers
            likers = random.choices(
                users,
                k=random.randint(0, 40)
            )
            source.likers.add(*likers)
            subscribers = random.choices(
                users,
                k=random.randint(0, 40)
            )
            source.subscribers.add(*subscribers)
            # Add tags
            tags = random.choices(
                tags,
                k=TAGS_PER_SOURCE
            )
            source.tags.add(*tags)

            sources.append(source)

        # Create game histories
        for _ in range(NUM_GAME_HISTORIES):
            user = random.choice(users)
            src = random.choice(sources)
            game_history = GameHistoryFactory(player=user, source=src)

```



## fixture 사용하기

> 모델 속성에 맞춰 JSON 형식으로 더미 데이터를 작성하고 불러오는 방법

```shell
# fixture 안에 더미데이터를 DB로 불러와 저장하기
python manage.py loaddata fixture폴더_내_json파일
# 앱에 있던 데이터를 dump.json이라는 파일로 저장하기
python manage.py dumpdata 앱이름 > dump.json
# 인덴트 넣어서 파일로 저장
python manage.py dumpdata musics --indent 2 > dump.json
```

