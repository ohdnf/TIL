# DTL: Django Template Language

Templates의 html 파일은 Django Template Language로 구성

## 기본 문법

1. 출력: `{{}}`
    `{{ variable }}`
2. 속성 호출: `.`
    ```html
    {{ section.title }}
    {{ menu.0 }}
    ```
3. tag 문법
    ```html
    {% for menu in lunch %}
    <p>menu</p>
    {% endfor %}
4. 주석
    ```html
    {# Comment 작성할 수 있는 주석입니다. #}
    ```

## 반복문

```html
<ul>
{% for reply in replies %}
  <li>{{ reply }}</li>
{% empty %}
  <li>There is no reply</li>
{% endfor %}
</ul>
```

- `for loop`(반복문) 안에서 사용할 수 있는 Variables
| Variable | Description |
| -------- | ----------- |
| `{{ forloop.counter }}` | 1부터 시작하는 indexing |
| `{{ forloop.counter0 }}` | 0부터 시작하는 indexing |
| `{{ forloop.revcounter }}` | 뒤에서부터 시작하는 indexing(마지막 1) |
| `{{ forloop.revcounter0 }}` | 뒤에서부터 시작하는 indexing(마지막 0) |
| `{{ forloop.first }}` | 첫 번째 반복일 경우 `True` 반환 |
| `{{ forloop.last }}` | 마지막 반복일 경우 `True` 반환 |

## 조건문

```html
{% if athlete_list %}
    Number of athletes: {{ athlete_list|length }}
{% elif athlete_in_locker_room_list %}
    Athletes should be out of the locker room soon!
{% else %}
    No athletes.
{% endif %}
```
> `athlete_list`가 비어있지 않다면 운동선수 총원이, `athlete_list`가 비어있고 `athlete_in_locker_room_list`가 비어있지 않다면 'Athlete should be out of the locker room soon!'이라는 멘트가, 둘 다 비어있다면 'No athletes'라는 멘트가 나온다.

## Filters

### `default`

`value`가 `False`이거나 `empty`일 경우 `default`로 지정된 값을 반환

`{{ value|default:"nothing" }}`

### `length`

```html
{{ value|length }}
```

## `with`

[Django Documentation](https://docs.djangoproject.com/ko/3.0/ref/templates/builtins/#with)

```html
{% with article_like_users=article.like_users.all %}
  {% if request.user in article_like_users %}
  <a href="{% url 'articles:like' article.pk %}">
    <i class="fas fa-heart fa-lg" style="color: red;"></i>
  </a>
  {% else %}
  <a href="{% url 'articles:like' article.pk %}">
    <i class="fas fa-heart fa-lg" style="color: black;"></i>
  </a>
  {% endif %}
  <p>{{ article_like_users|length }}명이 좋아합니다.</p>
{% endwith %}
```