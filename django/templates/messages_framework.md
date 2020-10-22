# The messages framework

[Django documentation](https://docs.djangoproject.com/en/2.1/ref/contrib/messages/)

> one-time notification message(also known as "flash message")

> cookie- and session-based messaging

## How to use

- `views.py`
    - 기본 문법
    ```python
    from django.contrib import messages
    messages.add_message(request, messages.INFO, 'Hello world.')
    ```

    - shortcut methods for each tags
    ```
    messages.debug(request, '%s SQL statements were executed.' % count)
    messages.info(request, 'Three credits remain in your account.')
    messages.success(request, 'Profile details updated.')
    messages.warning(request, 'Your account expires in three days.')
    messages.error(request, 'Document deleted.')
    ```

- `base.html`
    ```html
    
    <!-- 기본 ul 태그로 작성 -->
    {% if messages %}
    <ul class="messages">
        {% for message in messages %}
        <li {% if message.tags %} class="{{ message.tags }}"{% endif %}>{{ message }}</li>
        {% endfor %}
    </ul>
    {% endif %}

    <!-- bootstrap 활용 -->
    {% if messages %}
    <div class="messages">
      {% for message in messages %}
      <div class="alert alert-{% if message.tags %}{{ message.tags }}{% endif %}" role="alert">
        {{ message }}
      </div>
      {% endfor %}
    </div>
    {% endif %}
    ```

## How it works?

- `settings.py`
    ```python
    TEMPLATES = [
        {
            'BACKEND': 'django.template.backends.django.DjangoTemplates',
            'DIRS': [
                os.path.join(BASE_DIR, 'templates'),
            ],
            'APP_DIRS': True,
            'OPTIONS': {
                'context_processors': [
                    'django.template.context_processors.debug',
                    # html 파일에서 {{ request }} 사용 가능
                    'django.template.context_processors.request',
                    'django.contrib.auth.context_processors.auth',
                    # html 파일에서 {{ message }} 사용 가능
                    'django.contrib.messages.context_processors.messages',
                ],
            },
        },
    ]
    ```