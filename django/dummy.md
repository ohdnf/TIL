# dummy 데이터 다루기

## fixture

```shell
# fixture 안에 더미데이터를 
python manage.py loaddata fixture폴더_내_json파일
# 앱에 있던 데이터를 dump.json이라는 파일로 저장하기
python manage.py dumpdata 앱이름 > dump.json
# 인덴트 넣어서 파일로 저장
python manage.py dumpdata musics --indent 2 > dump.json
```