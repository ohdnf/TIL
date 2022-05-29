# Auto Documentation

## yasg

- API 관련 문서를 자동으로 생성
- models + views + urls

```shell
$ pip install yasg
```

```py
# settings.py

INSTALLED_APPS = [
    ...
    'drf_yasg',
]
```

```python
# urls.py
from django.urls import path
from . import views

# from rest_framework import permissions
from drf_yasg.views import get_schema_view
from drf_yasg import openapi


schema_view = get_schema_view(
   openapi.Info(
      title="Articles API",
      default_version='v1',
      description="게시판 API 서버입니다.",
    #   terms_of_service="https://www.google.com/policies/terms/",
    #   contact=openapi.Contact(email="contact@snippets.local"),
    #   license=openapi.License(name="BSD License"),
   ),
#   public=True,
#   permission_classes=(permissions.AllowAny,),
)

# urlpatterns = [
#   url(r'^swagger(?P<format>\.json|\.yaml)$', schema_view.without_ui(cache_timeout=0), name='schema-json'),
#   url(r'^swagger/$', schema_view.with_ui('swagger', cache_timeout=0), name='schema-swagger-ui'),
#   url(r'^redoc/$', schema_view.with_ui('redoc', cache_timeout=0), name='schema-redoc'),
# ]

urlpatterns = [
    ...
    path('swagger/', schema_view.with_ui('swagger')),
    path('redoc/', schema_view.with_ui('redoc')),
]
```
