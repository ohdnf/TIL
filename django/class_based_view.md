# Class-based Views

Django REST Framework에서는 API view를 작성할 때 function based view 대신 class-based view를 사용합니다. 자주 쓰이는 함수들을 재사용하고 이를 통해 코드를 [DRY](https://en.wikipedia.org/wiki/Don't_repeat_yourself)하게 유지할 수 있습니다.

```python
# Class-based Generic View
from django.views.generic import ListView, DetailView

class ArticleListView(ListView):
    model = Article

    # template_name = '모델명_list.html'
    # context_object_name = 'object_list'

class ArticleDetailView(ListView):
    model = Article
    
    def get(request):
        pass

    def post():
        pass
    
    def put():
        pass

    def delete():
        pass
```



## Reference

https://www.django-rest-framework.org/tutorial/3-class-based-views/