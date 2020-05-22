# Follow: 사용자 간 팔로우 기능 구현

## 사용자 모델 설정

### `models.py`

```py
# accounts/models.py
from django.db import models
from django.conf import settings
from django.contrib.auth.models import AbstractUser

class User(AbstractUser):
    followers = models.ManyToManyField(
        settings.AUTH_USER_MODEL,
        related_name = 'followings'
    )

```

> 현 상태에서 `migration`을 진행하면 `accounts/models.py`에 정의한 `User` 클래스는 `accounts_user`라는 이름으로 테이블이 잡힌다.
> 하지만 실제 데이터베이스에는 아직 `auth_user` 테이블로 `AUTH_USER_MODEL` 모델을 사용하기 때문에 프로그램이 돌아가지 않는다.
> 따라서 `accounts` 앱에 있는 사용자 모델을 쓰겠다는 설정을 `settings.py`에서 해준다.

### `settings.py`

```py
# settings.py

# AUTH_USER_MODEL = 'auth.User'     # 기존 모델(Default)
AUTH_USER_MODEL = 'accounts.User'   # 새로 내가 설정한 모델로 대체
```

> 이제 마이그레이션이 가능하다.

```shell
$ python manage.py makemigrations
$ python manage.py migrate
```

### `admin.py`

> Custom User Model을 만든 이후부터 admin 사이트에 관리하려면 `admin.py`에 추가해야한다.

```py
# accounts/admin.py
from django.contrib import admin
from .models import User

class UserAdmin(admin.ModelAdmin):
    pass

admin.site.register(User, UserAdmin)
```

### `forms.py`

> `ModelForm`을 상속받는 `UserCreationForm`, `UserChangeForm`은 `AbstractUser`을 모델로 쓰고 있다.
> `AuthenticationForm`은 `forms.ModelForm`이 아닌 `forms.Form`을 상속받기 때문에 수정하지 않아도 된다.

```py
# accounts/forms.py
from django.contrib.auth import get_user_model
from django.contrib.auth.forms import UserChangeForm, UserCreationForm

class CustomUserChangeForm(UserChangeForm):
    class Meta:
        model = get_user_model()
        fields = ('username', 'first_name', 'last_name', 'email', )

class CustomUserCreationForm(UserCreationForm):
    class Meta:
        model = get_user_model()
        fields = ('username', 'email', )
```

> `get_user_model()` 메서드 => `accounts.models.User` 라는 모델 객체를 반환

> `settings.AUTH_USER_MODEL` => `'accounts.User'`라는 문자열을 반환