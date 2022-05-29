# Django Form

## 기본 로직

1. Form 제공: `GET {URL}`

   1. `context`에 `form` 넘겨준다.

2. 양식에 맞춰 데이터 회수 => 처리: `POST {URL}`

   1. 양식\_데이터(`request.POST`)를 ModelForm에 넘긴다
      ```python
      form = ArticleForm(request.POST)
      ```
   2. 검증
      ```python
      if form.is_valid():
          form.save()
      ```

## User의 input 값을 받아 처리하기

> 두 페이지가 필요하다!
>
> `ping.html` 사용자 입력하는 페이지
>
> `pong.html` 입력을 받아 출력하는 페이지

1. 사용자 정보(`form`)
   - `boards/views.py`

     ```python
     def ping(request):
     	return render(request, 'boards/ping.html')
     ```

   - `templates/boards/ping.html`
     ```html
     <form action="/boards/ping/">
       <input type="text" name="msg" />
       <input type="submit" value="입력" />
     </form>
     ```
   ```

   ```
2. 정보 처리
   - `boards/views.py`
     ```python
     def pong(request):
         msg = request.GET.get('msg')
         return render(request, 'boards/pong.html', {'msg': msg})
     ```
   - `templates/boards/pong.html`
     ```html
     <p>출력: {{ msg }}</p>
     ```

## `/`

`action` 속성 시작 부분에...

- `/` 기호를 넣지 않으면 현재 페이지를 기준으로 요청을 보내고,
- `/` 기호를 넣게 되면 서버주소를 기준으로 요청을 보내게 됩니다.

## ModelForm

Model에서 쓰는 구조를 그대로 활용하기 위해서 사용

```python
# posts/forms.py
from django import forms
from .models import Post    # 내가 정의한 model

class PostForm(forms.ModelForm):
    class Meta:
        # 어떠한 모델인지 정의
        model = Post
        # 어떠한 필드를 담을 지
        fields = '__all__'
```

## User: 회원 가입 및 로그인 처리

> 비밀번호를 암호화해야 하기 때문에 일반 ModelForm은 쓸 수 없다.
> Django 내부 로직을 통해 이를 해결할 수 있다.

- `UserCreationForm`
- `AuthenticationForm`

### Signup: User Create

- Django Documentation

```python
# django/contrib/auth/forms.py

class UserCreationForm(forms.ModelForm):
    """
    A form that creates a user, with no privileges, from the given username and
    password.
    """
    error_messages = {
        'password_mismatch': _('The two password fields didn’t match.'),
    }
    password1 = forms.CharField(
        label=_("Password"),
        strip=False,
        widget=forms.PasswordInput(attrs={'autocomplete': 'new-password'}),
        help_text=password_validation.password_validators_help_text_html(),
    )
    password2 = forms.CharField(
        label=_("Password confirmation"),
        widget=forms.PasswordInput(attrs={'autocomplete': 'new-password'}),
        strip=False,
        help_text=_("Enter the same password as before, for verification."),
    )

    class Meta:
        model = User
        fields = ("username",)
        field_classes = {'username': UsernameField}

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if self._meta.model.USERNAME_FIELD in self.fields:
            self.fields[self._meta.model.USERNAME_FIELD].widget.attrs['autofocus'] = True

    def clean_password2(self):
        password1 = self.cleaned_data.get("password1")
        password2 = self.cleaned_data.get("password2")
        if password1 and password2 and password1 != password2:
            raise forms.ValidationError(
                self.error_messages['password_mismatch'],
                code='password_mismatch',
            )
        return password2

    def _post_clean(self):
        super()._post_clean()
        # Validate the password after self.instance is updated with form data
        # by super().
        password = self.cleaned_data.get('password2')
        if password:
            try:
                password_validation.validate_password(password, self.instance)
            except forms.ValidationError as error:
                self.add_error('password2', error)

    def save(self, commit=True):
        user = super().save(commit=False)
        user.set_password(self.cleaned_data["password1"])
        if commit:
            user.save()
        return user
```

### Login: Cookie & Session Create

- ArticleForm이나 UserCreateForm은 ModelForm을 상속받지만, AuthenticationForm은 그냥 Form을 상속받는다.
- 따라서 문법도 달라지게 된다.

- `request`가 중요하다.
  `AuthenticationForm(request, request.POST)`

```python
# django/contrib/auth/forms.py

class AuthenticationForm(forms.Form):
    """
    Base class for authenticating users. Extend this to get a form that accepts
    username/password logins.
    """
    username = UsernameField(widget=forms.TextInput(attrs={'autofocus': True}))
    password = forms.CharField(
        label=_("Password"),
        strip=False,
        widget=forms.PasswordInput(attrs={'autocomplete': 'current-password'}),
    )

    error_messages = {
        'invalid_login': _(
            "Please enter a correct %(username)s and password. Note that both "
            "fields may be case-sensitive."
        ),
        'inactive': _("This account is inactive."),
    }

    def __init__(self, request=None, *args, **kwargs):
        """
        The 'request' parameter is set for custom auth use by subclasses.
        The form data comes in via the standard 'data' kwarg.
        """
        self.request = request
        self.user_cache = None
        super().__init__(*args, **kwargs)

        # Set the max length and label for the "username" field.
        self.username_field = UserModel._meta.get_field(UserModel.USERNAME_FIELD)
        username_max_length = self.username_field.max_length or 254
        self.fields['username'].max_length = username_max_length
        self.fields['username'].widget.attrs['maxlength'] = username_max_length
        if self.fields['username'].label is None:
            self.fields['username'].label = capfirst(self.username_field.verbose_name)

    def clean(self):
        username = self.cleaned_data.get('username')
        password = self.cleaned_data.get('password')

        if username is not None and password:
            self.user_cache = authenticate(self.request, username=username, password=password)
            if self.user_cache is None:
                raise self.get_invalid_login_error()
            else:
                self.confirm_login_allowed(self.user_cache)

        return self.cleaned_data

    def confirm_login_allowed(self, user):
        """
        Controls whether the given User may log in. This is a policy setting,
        independent of end-user authentication. This default behavior is to
        allow login by active users, and reject login by inactive users.
        If the given user cannot log in, this method should raise a
        ``forms.ValidationError``.
        If the given user may log in, this method should return None.
        """
        if not user.is_active:
            raise forms.ValidationError(
                self.error_messages['inactive'],
                code='inactive',
            )

    def get_user(self):
        return self.user_cache

    def get_invalid_login_error(self):
        return forms.ValidationError(
            self.error_messages['invalid_login'],
            code='invalid_login',
            params={'username': self.username_field.verbose_name},
        )
```
