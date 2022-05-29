# django-admin 사용자 지정 명령어

> Django에서는 Application마다 `manage.py`로 실행할 사용자 명령어를 등록할 수 있습니다.

## 시작하기

`polls` 앱에 `closepoll`이라는 명령어를 등록하고 싶다고 가정해봅니다. 사용자 지정 명령어를 등록하기 위해서 우선 `management/commands` 폴더 구조를 만들어 줍니다. Django는 해당 폴더에 있는 Python module(`__init__.py`가 있는 디렉토리, 즉 패키지 안에 있는 Python 파일) 중 underscore(`_`)로 시작하지 않는 파일을 명령어로 등록합니다.

```
📁polls
├─__init__.py
├─models.py
├─📁management
│    ├─__init__.py
│    └─📁commands
│         ├─__init__.py
│         ├─_private.py
│         └─closepoll.py
├─tests.py
└─views.py
```

위와 같은 폴더/파일 구조에서 `closepoll` 명령어는 `polls` 앱이 등록된 모든 프로젝트에서 사용 가능합니다.

- `_private.py` 모듈은 파일명이 underscore로 시작하기 때문에 명령어로 등록되지 않습니다.
- `closepoll.py` 모듈 안에는 `BaseCommand` 클래스를 상속받은 `Command` 클래스가 정의되어 있어야 합니다.

명령어를 실행하기 위해서 `polls/management/commands/closepoll.py`는 다음과 같이 작성할 수 있습니다.

```python
from django.core.management.base import BaseCommand, CommandError
from polls.models import Question as Poll

class Command(BaseCommand):
    help = 'Closes the specified poll for voting'

    def add_arguments(self, parser):
        parser.add_argument('poll_ids', nargs='+', type=int)

    def handle(self, *args, **options):
        for poll_id in options['poll_ids']:
            try:
                poll = Poll.objects.get(pk=poll_id)
            except Poll.DoesNotExist:
                raise CommandError('Poll "%s" does not exist' % poll_id)

            poll.opened = False
            poll.save()

            self.stdout.write(self.style.SUCCESS('Successfully closed poll "%s"' % poll_id))
```

새로운 사용자 지정 명령어는 다음과 같이 실행할 수 있습니다.

```shell
$ python manage.py closepoll <poll_ids>
```
