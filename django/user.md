# User Authentication

[Django Documentation](https://docs.djangoproject.com/en/3.0/topics/auth/default/)

## `accounts` 앱 생성

```bash
$ python manage.py startapp accounts
```

## 상속관계

> [Django GitHub](https://github.com/django/django/blob/master/django/contrib/auth/models.py) 참고

- `models.Model`
    - `AbstractBaseUser`: password
        - `AbstractUser`
            - `User`

- `django/contrib/auth/base_user.py`
    ```py
    class AbstractBaseUser(models.Model):
        password = models.CharField(_('password'), max_length=128)
        last_login = models.DateTimeField(_('last login'), blank=True, null=True)

        is_active = True

        ...
    ```

- `django/contrib/auth/models.py`

    ```py
    class AbstractUser(AbstractBaseUser, PermissionsMixin):
        """
        An abstract base class implementing a fully featured User model with
        admin-compliant permissions.
        Username and password are required. Other fields are optional.
        """
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
        first_name = models.CharField(_('first name'), max_length=150, blank=True)
        last_name = models.CharField(_('last name'), max_length=150, blank=True)
        email = models.EmailField(_('email address'), blank=True)
        is_staff = models.BooleanField(
            _('staff status'),
            default=False,
            help_text=_('Designates whether the user can log into this admin site.'),
        )
        is_active = models.BooleanField(
            _('active'),
            default=True,
            help_text=_(
                'Designates whether this user should be treated as active. '
                'Unselect this instead of deleting accounts.'
            ),
        )
        date_joined = models.DateTimeField(_('date joined'), default=timezone.now)

        objects = UserManager()

        EMAIL_FIELD = 'email'
        USERNAME_FIELD = 'username'
        REQUIRED_FIELDS = ['email']

        ...


    class User(AbstractUser):
        """
        Users within the Django authentication system are represented by this
        model.
        Username and password are required. Other fields are optional.
        """
        class Meta(AbstractUser.Meta):
            swappable = 'AUTH_USER_MODEL'
    ```

## signup: Create

```py
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

## 로그인

