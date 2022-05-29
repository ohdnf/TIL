# One to Many Relationship

[Django Documentation](https://docs.djangoproject.com/en/3.0/topics/db/queries/#one-to-many-relationships)

> 1 has many N
> N belongs to 1

## 예시

```py
# models.py
import django.db import models

class Reporter(models.Model):
    username = models.CharField(max_length=20)

class Article(models.Model):
    title = models.CharField(max_length=100)
    conent = models.TextField()
    reporter = models.ForeignKey(Reporter, on_delete=models.CASCADE)
```

## Naming Convention

```py
reporter.article_set.all() # 자동으로 설정
article.reporter # related_name 옵션으로 직접 설정
```

```py

# models.py
class Reporter(models.Model):
    ...
    article = models.ForeignKey(
        Article, on_delete=models.CASCADE,
        related_name='reporter')

```

- 정의: 모델 단수형(ex. `.user`)
- 역참조: 모델\_set(ex. `.article_set`)

## `models.ForeignKey({모델명}, _on_delete_)`

```py
# models.py
class User(models.Model):
    username = models.CharField(max_length=10)

class Article(models.Model):
    title = models.CharField(max_length=100)
    content = models.TextField()
    user = models.ForeignKey(User, on_delete=models.CASCADE)

class Comment(models.Model):
    content = models.TextField()
    article = models.ForeignKey(Article, on_delete=models.CASCADE)
    user = models.ForeignKey(User, on_delete=models.CASCADE)

```

## ORM

```py
from onetomany.models import User, Article, Comment

# objects
u1 = User.objects.create(username='Kim')
u2 = User.objects.create(username='Lee')

a1 = Article.objects.create(title='1글', user=u1)
a2 = Article.objects.create(title='2글', user=u2)
a3 = Article.objects.create(title='3글', user=u2)
a4 = Article.objects.create(title='4글', user=u2)

c1 = Comment.objects.create(content='1글1댓', article=a1, user=u2)
c2 = Comment.objects.create(content='1글2댓', article=a1, user=u2)
c3 = Comment.objects.create(content='2글1댓', article=a2, user=u1)
c4 = Comment.objects.create(content='4글1댓', article=a4, user=u1)
c5 = Comment.objects.create(content='3글1댓', article=a3, user=u2)
c6 = Comment.objects.create(content='3글2댓', article=a3, user=u1)

```
