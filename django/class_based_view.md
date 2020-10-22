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