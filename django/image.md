# ImageField

image를 받기 위한 모델 필드

## Pillow library

```shell
$ pip install pillow
```


## Image Upload

### `models.py`

```py
from django.db import models
class Article(models.Model):
    image = models.ImageField()
```

### `views.py`

```py
request.FILES
```

### `form.html`

```html
<img src="{{ articles.image.url }}" alt="My image">
```


### `settings.py`

```py
# 미디어 파일을 저장하기 위한 루트 경로
MEDIA_ROOT = os.path.join(BASE_DIR, 'media')
MEDIA_URL = '/media/'
```

### `urls.py`

```py
from django.conf import settings
from django.conf.urls.static import static

urlpatterns = [

] + static(settings.STATIC_URL, document_root=settings.STATIC_ROOT)
```

## django-imagekit: 이미지를 잘라서 저장

[link](https://github.com/matthewwithanm/django-imagekit)

```shell
pip install pilkit django-imagekit
```

### `models.py`

```py
from django.db import models
from imagekit.models import ImageSpecField
from imagekit.processors import ResizeToFill

class Profile(models.Model):
    avatar = models.ImageField(upload_to='avatars')
    # 원본 저장,
    avatar_thumbnail = ImageSpecField(source='avatar',
                                      processors=[ResizeToFill(100, 50)],
                                      format='JPEG',
                                      options={'quality': 60})

profile = Profile.objects.all()[0]
print(profile.avatar_thumbnail.url)    # > /media/CACHE/images/982d5af84cddddfd0fbf70892b4431e4.jpg
print(profile.avatar_thumbnail.width)  # > 100
```