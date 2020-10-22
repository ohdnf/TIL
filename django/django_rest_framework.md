# Django REST Framework

## DRF로 CRUD하는 이유
0. Money
1. UX 안 좋으면 => User X => Money X
    - Data => Tells what human want
    - Mobile application(web)
    - churn(이탈율)
    - ~~Adobe Flash~~ ==> JavaScript
2. Template X, Data만 빠르게

==> javascript & frameworks 분리

## Web APIs for Django

- [django-rest-framework](https://www.django-rest-framework.org/)
- [GitHub](https://github.com/encode/django-rest-framework)

### Install

```shell
$ pip install djangorestframework
```

```python
# settings.py

INSTALLED_APP = [
    ...
    'rest_framework',
]
```

## Serializer

> 직렬화, `Object(언어, 데이터베이스) => String(JSON)`하는 것

- 포맷의 변환 (데이터 전송/이동)

dict => JSON (stringify)    # 직렬화
JSON => dict (parse)        # 역직렬화

> 다음 `serializers.py`와 `views.py`는 음악가/곡/평론 모델을 갖고있는 API 요청들을 간단히 구현

### `serializers.py`

```python
from rest_framework import serializers
from .models import Artist, Music, Comment

class ArtistSerializer(serializers.ModelSerializer):
    class Meta:
        model = Artist
        fields = ('id', 'name',)

class MusicSerializer(serializers.ModelSerializer):
    class Meta:
        model = Music
        fields = ('id', 'title')

class CommentSerializer(serializers.ModelSerializer):
    class Meta:
        model = Comment
        fields = ('id', 'content')

class ArtistDetailSerializer(ArtistSerializer):
    # column명 ==> serializer에서 어떻게 보일지
    musics = MusicSerializer(many=True)
    # source ==> Model의 property
    music_count = serializers.IntegerField(source='musics.count', read_only=True)

    class Meta(ArtistSerializer.Meta):
        fields = ArtistSerializer.Meta.fields + ('musics', 'music_count',)

class MusicDetailSerializer(MusicSerializer):
    comments = CommentSerializer(many=True)

    class Meta(MusicSerializer.Meta):
        fields = MusicSerializer.Meta.fields + ('comments',)
```

### `views.py`

```python
# views.py

from django.shortcuts import render, get_object_or_404

from rest_framework.response import Response
from rest_framework.decorators import api_view

from .models import Artist, Music, Comment
from .serializers import ArtistSerializer, MusicSerializer, CommentSerializer, ArtistDetailSerializer, MusicDetailSerializer
# Create your views here.

@api_view(['GET'])
def artists_list(request):
    artists = Artist.objects.all()
    serializer = ArtistSerializer(artists, many=True)
    return Response(serializer.data)

@api_view(['GET'])
def artists_detail(request, artist_pk):
    artist = get_object_or_404(Artist, pk=artist_pk)
    serializer = ArtistDetailSerializer(artist)
    return Response(serializer.data)

@api_view(['GET'])
def music_list(request):
    musics = Music.objects.all()
    serializer = MusicSerializer(musics, many=True)
    return Response(serializer.data)

@api_view(['GET'])
def music_detail(request, music_pk):
    music = get_object_or_404(Music, pk=music_pk)
    serializer = MusicDetailSerializer(music)
    return Response(serializer.data)

@api_view(['POST'])
def comment_create(request, music_pk):
    # music = get_object_or_404(Music, pk=music_pk)
    serializer = CommentSerializer(data=request.data)
    if serializer.is_valid(raise_exception=True):
        serializer.save(music_id=music_pk)
        return Response(serializer.data)
    return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)

@api_view(['PUT', 'DELETE'])
def comment_update_delete(request, comment_pk):
    comment = get_object_or_404(Comment, pk=comment_pk)
    if request.method == 'PUT':
        serializer = CommentSerializer(comment, data=request.data)
        if serializer.is_valid(raise_exception=True):
            serializer.save()
            # return Response(serializer.data)
            return Response({'message': '성공적으로 수정되었습니다.'})
        return Response(serializer.errors, status=status.HTTP_400_BAD_REQUEST)
    elif request.method == 'DELETE':
        comment.delete()
        # return Response(status=status.HTTP_204_NO_CONTENT)
        return Response({'message': '성공적으로 삭제되었습니다.'})

```

