# Django에서 Redis로 캐싱하기

> 웹 서비스에서 응답 속도는 성공을 좌우하는 요소입니다. 1초도 안 걸리겠지라는 기대를 갖는 환경에서 실행 속도는 매출과 직결됩니다. 돈을 떠나서, 빠른 페이지 로딩은 웹 사이트를 방문하는 사용자 경험을 향상시킬 수 있습니다.
>
> 요청을 받고 응답을 반환하기까지 서버에서 일어나는 모든 일은 더 많은 페이지 로딩 시간을 요구합니다. 일반적으로 서버에서 일어나는 프로세스를 없앨수록 애플리케이션 실행 속도는 더 빨라집니다. 처리된 데이터를 **캐싱**하고, 다음부터 요청된 데이터를 캐시 서버에서 처리하는 것은 서버의 부담을 완화시키는 방법 중 하나입니다.
>
> [Django's cache framework 공식 문서](https://docs.djangoproject.com/en/3.1/topics/cache/)



## Redis란 무엇인가?

[Redis](https://redis.io/)는 

- **Re**mote **Di**ctionary **S**erver라고도 알려져 있는
- **key-value** 구조로,
-  캐시 엔진, 메시지 브로커로 쓰일 수 있는
-  **in-memory** DB이며
-  모든 데이터를 RAM에서 처리하기 때문에
-  데이터를 아주 빠르게 전달할 수 있습니다. 
-  또한 NoSQL DB이고,
-  `strings`, `hashes`, `lists`, `sets`, `sorted sets`와 같은 *range queries*(1차원 쿼리)나 `bitmaps`, `hyperloglogs`, `geospatial indexes`와 같은 *radius queries*(2차원 쿼리)를 지원하며

## 시작하기

> Python, Django, Git이 설치되어 있다고 가정합니다.
>
> 현재 Redis는 공식적으로 Windows OS에서 지원되지 않습니다. 저는 Windows OS에서 Linux용 가상 머신을 설치해 **Linux 환경에서** 실습을 진행하겠습니다.
>
> #### Windows 10에서 WSL 설치하기
>
> [Windows Subsystem for Linux Installation Guide for Windows 10](https://docs.microsoft.com/en-us/windows/wsl/install-win10#install-the-windows-subsystem-for-linux)를 따라하면 됩니다. 설치 과정은 대략적으로 다음과 같습니다.
>
> 1. Powershell로 WSL를 사용가능하도록 활성화
>
>    ```powershell
>    dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
>    ```
>
> 2. WSL 2로 업데이트
>
>    Windows 10 최신 버전으로 업데이트
>
> 3. 가상 머신 플랫폼을 활성화
>
>    ```powershell
>    dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
>    ```
>
> 4. Linux 커널을 다운로드하고 패키지 업데이트하기
>
> 5. WSL 2를 기본 버전으로 설정하기
>
> 6. Linux 배포 버전 설치하기
>
>    [Ubuntu 18.04 LTS](https://www.microsoft.com/ko-kr/p/ubuntu-1804-lts/9n9tngvndl3q?rtc=1&activetab=pivot:overviewtab)
>
> 7. Linux 환경 설정
>
>    ```shell
>    $ sudo apt-get update
>    $ sudo apt-get upgrade
>    ```



## Redis 설치 및 테스트

```shell
$ sudo apt-get install redis-server
$ redis-cli -v
redis-cli 4.0.9
$ redis-cli ping
PONG
```

Redis 서버 

```shell
$ sudo service redis-server restart
```



## Django 설정

### `django-redis` 설치

> Django에서 Redis-server를 사용하기 위한 인터페이스입니다.
>
> [github](https://github.com/jazzband/django-redis)

```shell
$ python -m pip install django-redis
```



### `settings.py` 

다음 내용을 추가합니다.

```python
CACHES = {
    "default": {
        "BACKEND": "django_redis.cache.RedisCache",
        "LOCATION": "redis://127.0.0.1:6379/1",
        "OPTIONS": {
            "CLIENT_CLASS": "django_redis.client.DefaultClient"
        },
        "KEY_PREFIX": "example"
    }
}

INTERNAL_IPS = [
    '127.0.0.1',
]

CACHE_TTL = 60 * 15
```

- `CACHES`
  - `BACKEND` => Redis를 캐시 서버로 지정합니다.
  - `LOCATION` => Redis의 1번 DB를 사용합니다.
  - `OPTIONS`
  - `KEY_PREFIX`
- `INTERNAL_IPS`
- `CACHE_TTL` => Cache time to live. 캐시 데이터가 15분 동안 유지되도록 기본값 설정



### `views.py`

```python
from django.conf import settings
from django.core.cache.backends.base import DEFAULT_TIMEOUT
from django.views.decorators.cache import cache_page
from django.core.cache import cache

from rest_framework.decorators import api_view

from .models GameHistory
from .serializers import GameHistorySerializer, RankSerializer


CACHE_TTL = getattr(settings, 'CACHE_TTL', DEFAULT_TIMEOUT)


# 사용자 순위 관련 API

@api_view(['GET'])
@cache_page(CACHE_TTL)
def rank_retrieve(request):
    """
    각 플레이어마다 갖고 있는 모든 점수를 합산하여 100위까지 랭킹을 반환
    """
    queryset = cache.get_or_set('queryset', GameHistory.objects.values('player').annotate(total_score=Sum('score')).order_by('-total_score')[:100])
    serializer = RankSerializer(queryset, many=True)
    return Response(serializer.data, status=status.HTTP_201_CREATED)

```



## 캐시 데이터 확인하기





## 캐시 데이터 갱신하기

