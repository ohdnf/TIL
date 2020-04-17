# Django Form

## User의 input 값을 받아 처리하기

> 두 페이지가 필요하다!

1. 사용자 정보(`form`)
    - `boards/views.py`
        ```py
        def ping(request):
        return render(request, 'boards/ping.html')
        ```

    - `templates/boards/ping.html`
        ```html
        <form action="/boards/ping/">
            <input type="text" name="msg">
            <input type="submit" value="입력">
        </form>

2. 정보 처리
    - `boards/views.py`
        ```py
        def pong(request):
        msg = request.GET.get('msg')
        context = {
            'msg': msg,
        }
        return render(request, 'boards/pong.html', context)
        ```
    
    - `templates/boards/pong.html`
        ```html
        <p>출력: {{ msg }}</p>
        ```

## `/`
`action` 속성에 시작부분에 /기호를 넣지 않으면 현재페이지를 기준으로 요청으로 보내게 되고 /기호를 넣게 되면 서버주소를 기준으로 요청을 보내게 됩니다.


## ModelForm

```py
# posts/forms.py
from django import forms
from .models import Post

class PostForm(forms.ModelForm):
    class Meta:
        # 어떠한 모델
        model = Post
        # 어떠한 필드
        fields = '__all__'
```