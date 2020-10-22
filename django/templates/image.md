# Django에서 Image 다루기

[Django Documentation](https://docs.djangoproject.com/en/2.1/howto/static-files/)

- STATIC vs. MEDIA
    - STATIC: CSS, JavaScripts, Images
    - MEDIA: User가 Upload한 file



## Image Upload

### Pillow library

- local 환경에서 작업 시 설치 필요
  
    ```shell
    $ pip install pillow
    ```

- `models.py`

    ```python
    from django.db import models
    class Article(models.Model):
        image = models.ImageField()
    ```

    > 새로운 컬럼으로 추가해 마이그레이션을 시도할 경우,
    > 이전에 만들어진 레코드들의 값이 없어 실패하게 된다.
    > 이 경우 NOT NULL 옵션을 NULLABLE하게 바꾸거나 DEFAULT 값을 설정해야 한다.
    
    ```python
    from django.db import models
    class Article(models.Model):
        image = models.ImageField(blank=True)   # NULLABLE
    ```

- `views.py`

    ```python
    @login_required
    def create(request):
        if request.method == 'POST':
            form = ArticleForm(request.POST, request.FILES)     # MEDIA 파일을 받기 위한 설정
            if form.is_valid():
                ...
    ```

- `form.html`

    ```html
    {% extends 'base.html' %}

    {% block content %}
    <!-- file이나 image를 받기 위해서 enctype 속성을 설정해야 함 -->
    <form action="" method="POST" enctype="multipart/form-data">
      {% csrf_token %}
      <div>
        {% form %}
        <!-- <label for="image">Choose image file to upload</label>
        <input type="file" id="image" name="image" multiple> -->
      </div>
      <div>
        <button class="btn btn-primary">Submit</button>
      </div>
    </form>
    {% endblock %}
    ```

- `detail.html`

    ```html
    {% extends 'base.html' %}

    {% block content %}
    <img src="{{ articles.image.url }}" alt="My image">
    {% endblock  %}
    ```
    
    > articles 객체에 있는 이미지의 URL을 가져와도 서버를 돌리면 깨져있는 것을 확인할 수 있다. 이미지가 올바르게 나오기 위해선 추가 설정이 필요하다.

### 추가 설정

- `settings.py`

    ```python
    # 미디어 파일을 저장하기 위한 루트 경로
    MEDIA_ROOT = os.path.join(BASE_DIR, 'media')
    # 미디어 파일을 불러오기 위한 루트 경로
    MEDIA_URL = '/media/'
    ```

- `urls.py`

    ```python
    from django.conf import settings
    from django.conf.urls.static import static

    urlpatterns = [
        ...
    ] + static(settings.STATIC_URL, document_root=settings.STATIC_ROOT)
    # 서버 안에 있는 STATIC 파일을 특정한 URL로 서빙할 수 있게 접근 가능 설정
    ```



## `django-imagekit`: 이미지를 잘라서 저장

[django-imagekit](https://github.com/matthewwithanm/django-imagekit)
[pilkit](https://github.com/matthewwithanm/pilkit)

```shell
pip install pilkit django-imagekit
```

### `models.py`

```python
from django.db import models
from imagekit.models import ImageSpecField
from imagekit.processors import ResizeToFill, ResizeToFit, Thumbnail
# ResizeToFill: 300*300으로 crop
# ResizeToFit: 가장 긴 곳(너비 또는 높이)을 300으로 맞추고, 비율에 맞춰서 crop
# Thumbnail:

class Profile(models.Model):
    # 원본 저장 + 활용 => ImageSpecField + source 설정
    avatar = models.ImageField(upload_to='avatars')
    # DB 저장 X, 호출하게 되면, 잘라서 표현
    avatar_thumbnail = ImageSpecField(source='avatar',
                                      processors=[ResizeToFill(100, 50)],
                                      format='JPEG',
                                      options={'quality': 60})

    # 원본을 잘라서 저장
    avatar_thumbnail = ProcessedImageField(upload_to='avatars',
                                           processors=[ResizeToFill(100, 50)],
                                           format='JPEG',
                                           options={'quality': 60})


profile = Profile.objects.all()[0]
print(profile.avatar_thumbnail.url)    # > /media/CACHE/images/982d5af84cddddfd0fbf70892b4431e4.jpg
print(profile.avatar_thumbnail.width)  # > 100
```