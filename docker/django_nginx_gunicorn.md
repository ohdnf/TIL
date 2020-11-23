# Django 서버 Docker로 배포하기...



```shell
$ docker-compose down --rmi all -v --remove-orphans

$ ln -s /etc/nginx/sites-available/ypc /etc/nginx/sites-enable/ypc

```



## Nginx에 리액트 빌드한 파일들 올리기

> front 도커 이미지를 만들기
>
> docker-compose

`back` 브랜치와 `front` 브랜치를 각각 클론한다.



`nginx/conf.d` 안에 `default.conf`가 존재하므로 수정

conf를 바꾸고 나서 그냥 `docker-compose restart` 를 하면 실패 시 그냥 다운되어 버린다.

`nginx -s reload`를 하면 error를 캐치할 수 있다.



```shell
# build 프론트 파일을 도커 컨테이너 내부로 옮기는 명령어
docker cp ./build/. ${NGINX_컨테이너이름}:/var/www/html/

docker cp ./build/frontend/. 100moon1ta_nginx_1:/usr/share/nginx/html/
```



```yaml
# docker-compose.yml

command: gunicorn l00moon1ta.wsgi:application --bind 0.0.0.0:8000
```



### docker-compose 올리기

```shell
docker-compose up -d
```

### 컨테이너 안에서 bash 창 열기

```shell
docker exec -it nginx_typo /bin/bash
root@~~:# 
```





`nginx.conf`

```shell
server {
        listen 80;
        location / {
                root /var/www/html;
                try_files $uri $uri/ index.html;
        }
}
```

