# User Authentication

[Django Documentation](https://docs.djangoproject.com/en/3.0/topics/auth/default/)



## `auth.User` vs. `setting.AUTH_USER_MODEL`



## `accounts` 앱 생성

```bash
$ python manage.py startapp accounts
```



## 회원가입: `signup`

```python
# accounts/urls.py
from django.urls import path
from . import views

app_name = 'accounts'

urlpatterns = [
    path('signup/', views.signup, name='signup'),
]
```

```python
# accounts/views.py
from django.shortcuts import render
from django.contrib.auth import authenticate
from django.contrib.auth import login as auth_login
from django.contrib.auth.forms import UserCreationForm

def signup(request):
    if request.user.is_authenticated:
        return redirect('posts:index')
    if request.method == 'POST':
        form = UserCreationForm(request.POST)
        if form.is_valid():
            user = form.save()
            # 회원가입 성공 시 자동으로 로그인
            user = authenticate(
                username=form.cleaned_data.get('username'),
                password=form.cleaned_data.get('password1')
                )
            auth_login(request, user)
            # 로그인이 되면 그 전 페이지 또는 메인 페이지로 이동
            return redirect(request.GET.get('next') or 'posts:index')
    else:
        form = UserCreationForm()
    return render(request, 'accounts/signup.html', {'form': form})
```

```html
<!-- accounts/templates/accounts/signup.html -->
{% extends 'base.html' %}

{% block body %}
<form action="" method="POST">
  {% csrf_token %}
  {% form %}
  <button class="btn btn-primary">Join!</button>
  <a href="{% url 'accounts:login' %}">Log in</a>
</form>
{% endblock %}
```



## 로그인: `login`

```python
# accounts/urls.py
urlpatterns = [
    ...
    path('login', views.login, name='login'),
]
```

```python
# accounts/views.py

from django.contrib.auth import login as auth_login
from django.contrib.auth.forms import UserCreationForm, AuthenticationForm


def login(request):
    if request.user.is_authenticated:
        return redirect('posts:index')
    if request.method == 'POST':
        form = Authentication(request, request.POST)
        if form.is_valid():
            form.save()
            auth_login(request, form.get_user())
            # 로그인이 되면 그 전 페이지 또는 메인 페이지로 이동
            return redirect(request.GET.get('next') or 'posts:index')
    else:
        form = AuthenticationForm()
    return render(request, 'accounts/login.html', {'form': form})
```

```html
<!-- accounts/templates/accounts/login.html -->
{% extends 'base.html' %}

{% block body %}
<form action="" method="POST">
  {% csrf_token %}
  {% form %}
  <button class="btn btn-primary">Log in</button>
  <a href="{% url 'accounts:signup' %}">Sign up</a>
</form>
{% endblock %}
```



## 마이페이지: `detail`

```python
# accounts/urls.py
...
urlpatterns = [
    path('<int:pk>/detail/', views.detail, name='detail'),
]
```

```python
# accounts/views.py
...
def detail(request, pk):
    user = User.objects.get(pk=pk)
    return render(request, 'accounts/detail.html', {'user': user})
```

```html
<!-- accounts/templates/accounts/detail.html -->
{% extends 'base.html' %}

{% block body %}
{{ user.username }}'s Info

{% if request.user == user %}
<!-- GET -->
<a href="{% url 'accounts:delete' %}">Delete Account</a>
<!-- POST -->
<form action="{% url 'accounts:delete' %}" method="POST">
  {% csrf_token %}
  <button class="btn btn-danger">Delete my Account</button>
</form>
{% endif %}
{% endblock %}
```



## 회원정보 수정: `update`

```python
# accounts/urls.py
...
urlpatterns = [
    ...
    path('update/', views.update, name='update'),
]
```

```python
# accounts/forms.py
from django.contrib.auth import get_user_model
from django.contrib.auth.forms import UserChangeForm

class CustomUserChangeForm(UserChangeForm):
    class Meta:
        model = get_user_model()
        fields = ['username', 'first_name', 'last_name', 'email']
```

```python
# accounts/views.py

from .forms import CustomUserChangeForm

def update(request):
    if request.method == 'POST':
        form = CustomUserChangeForm(request, instance=request.user)
        if form.is_valid():
            form.save()
            return redirect('accounts:update')
    else:
        form = CustomUserChangeForm(instance=request.user)
    return render(request, 'accounts/update.html', {'form': form})
```

```html
<!-- accounts/templates/accounts/detail.html -->
{% extends 'base.html' %}

{% block body %}
{{ user.username }}'s Info

{% if request.user == user %}
<!-- GET -->
<a href="{% url 'accounts:update' %}">Update info</a>
<!-- POST -->
<form action="{% url 'accounts:delete' %}" method="POST">
  {% csrf_token %}
  <button class="btn btn-danger">Delete account</button>
</form>
{% endif %}
{% endblock %}
```



## 로그아웃: `logout`

```python
# accounts/urls.py
...

urlpatterns = [
    path('logout/', views.logout, name='logout'),
]
```

```python
# accounts/views.py
from django.contrib.auth import logout as auth_logout
from django.contrib.auth.decorators import login_required

@login_required
def logout(request):
    auth_logout(request)
    return redirect('posts:index')
```

```html
<!-- templates/base.html -->
<!DOCTYPE html>
<html lang="ko">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Workshop</title>
</head>
<body>
  <nav class="navbar navbar-expand-lg navbar-light bg-light">
    <a class="navbar-brand" href="{% url 'posts:index' %}">Navbar</a>
    <div class="navbar-nav">
    {% if request.user.is_authenticated %}
    <!-- {% if user.is_authenticated %} -->
    <!-- user는 context에서 넘겨준 값, request.user는 현재 장고 서버에서 보내준 응답 -->
      <a class="nav-item nav-link" href="#">{{ request.user.username }}</a>
      <a class="nav-item nav-link" href="{% url 'accounts:logout' %}">Logout</a>
    {% else %}
      <a class="nav-item nav-link" href="{% url 'accounts:login' %}">Login</a>
      <a class="nav-item nav-link" href="{% url 'accounts:signup' %}">Sign up</a>
    {% endif %}
    </div>
  </nav>
  {% block body %}
  {% endblock %}
</body>
</html>
```



## 회원탈퇴: `delete`

```python
# accounts/urls.py
...
urlpatterns = [
    ...
    path('delete/', views.delete, name='delete'),
]
```

```python
# accounts/views.py

from django.contrib.auth.decorators import login_required
from django.views.decorators import require_POST

@require_POST
@login_required
def delete(request):
    request.user.delete()
    return redirect('posts:index')
```



## Sign UP vs Sign IN

| 구분 | Sign Up | Sign In |
| ---- | ------- | ------- |
| forms | UserCreationForm | AuthenticationForm |
| 로직 | User Object 생성 | Django Session 저장 및 변경 + 사용자에게 쿠키 전달 |



## login_required

```python
from django.contrib.auth.decorators import login_required

@login_required
def func(request):
    ...
    return
```

- 로그인되어있지 않을 경우 로그인 페이지로 이동
    - 로그인 URL Default는 `LOGIN_URL='/accounts/login/'`이며 `settings.py`에서 변경이 가능하다.
- `next` 파라미터를 활용 가능



## 상속관계

> [Django GitHub](https://github.com/django/django/blob/master/django/contrib/auth/models.py) 참고

- `models.Model`
    - `AbstractBaseUser`: password
        - `AbstractUser`: username
            - `User`

- `django/contrib/auth/base_user.py`
    ```python
    class AbstractBaseUser(models.Model):
        password = models.CharField(_('password'), max_length=128)
        last_login = models.DateTimeField(_('last login'), blank=True, null=True)

        is_active = True

        ...
    ```

- `django/contrib/auth/models.py`

    ```python
    class AbstractUser(AbstractBaseUser, PermissionsMixin):
        username_validator = UnicodeUsernameValidator()

        username = models.CharField(
            _('username'),
            max_length=150,
            unique=True,
            help_text=_('Required. 150 characters or fewer. Letters, digits and @/./+/-/_ only.'),
            validators=[username_validator],
            error_messages={
                'unique': _("A user with that username already exists."),
            },
        )
        ...

    class User(AbstractUser):
        class Meta(AbstractUser.Meta):
            swappable = 'AUTH_USER_MODEL'
    ```



## signup: Create

```python
# accounts/views.py
from django.contrib.auth.forms import UserCreationForm

def signup(request):
    if request.method == 'POST':
        form = UserCreationForm(request.POST)
        if form.is_valid():
            user = form.save()
            return redirect('posts:index')
    else:
        form = UserCreationForm()
    context = {
        'form': form,
    }
    return render(request, 'accounts/signup.html', context)
```

- `UserCreationForm`은 `forms.ModelForm`을 상속받았다.

### 회원가입

- 비밀번호 제공 및 확인
    - `UserCreationForm` 추가 column 정의
    - 저장 로직에서 일치하는지 확인
- 비밀번호 암호화 저장
    - `User.objects.create_user(username, email=None, password=None)`
    - `user.set_password(password)`



## 비밀번호

### 암호화

- 해시함수(SHA, Secure Hash Algorithm) 활용
    - PBKDF2(Password-Based Key Derivation Function)
        - 해시 함수의 컨테이너
        - 솔트를 적용한 후 해시 함수의 반복 횟수를 임의로 선택
    - SHA256
- 역산이 불가능(단방향)

> [Naver D2의 '안전한 패스워드 저장'](https://d2.naver.com/helloworld/318732) 참조



## Custom User

```python
# accounts/models.py

from django.db import models
from django.contrib.auth.models import AbstractUser

class User(AbstractUser):
    pass
```

```python
# settings.py

...

AUTH_USER_MODEL = 'accounts.User'
```