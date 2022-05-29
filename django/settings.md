# Django 환경 설정

## 개발 환경과 배포 환경 분리하기

> Django 프로젝트를 개발할 때 필요한 모듈(예. `django-debug-toolbar`)이 배포할 때는 필요없는 경우가 있다. 이런 모듈을 분리하여 관리할 수 있는 방법을 알아보자.

```
📁django_project
├─📁django_project
│    ├─...
│    ├─local_settings.py
│    ├─settings.py
│    └─...
├─📁posts
├─📁venv
├─.gitignore
├─manage.py
├─local_requirements.txt
└─requirements.txt
```

대략 다음과 같은 프로젝트 폴더/파일 구조를 가지고 있다고 가정한다.

`local_settings.py`에 개발 환경을 설정하고 필요한 모듈은 `local_requirements.txt`에 정리한다.

원격 저장소에 저장하기 전에 `.gitignore` 파일에 해당 파일들을 추적하지 않도록 추가해준다.

```bash
# .gitignore

...

### Django ###
*.log
*.pot
*.pyc
__pycache__/
local_settings.py
local_requirements.txt
db.sqlite3
db.sqlite3-journal
media

...
```

### 환경 설정(`settings`) 지정하기

Django 프로젝트를 실행할 때 다음과 같은 명령어를 통해 환경설정을 다르게 할 수 있다.([참고](https://docs.djangoproject.com/en/2.2/topics/settings/#envvar-DJANGO_SETTINGS_MODULE))

> Unix Bash shell

```shell
# 개발 환경에서 local_settings.py로 서버 실행
$ export DJANGO_SETTINGS_MODULE=django_project.local_settings
$ python manage.py runserver
```

> Windows Terminal

```powershell
> set DJANGO_SETTINGS_MODULE=django_project.local_settings
> python manage.py runserver
```

그냥 실행 옵션으로 설정할 수도 있다.

```shell
python manage.py runserver --settings=django_project.local_settings
```
