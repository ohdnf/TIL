# NGINX Basics

## 핵심 파일, 명령어, 디렉토리 알아보기

### NGINX 파일과 디렉토리

#### `/etc/nginx/`

- Default configuration root for the NGINX server

#### `/etc/nginx/nginx.conf`

- Default configuration **entry point** used by the NGINX service

- Sets up global settings for worker process, tuning, logging, loading dynamic modules, and references to other NGINX configuration files

- Includes the top-level `http` block, which includes all configuration files in `/etc/nginx/conf.d`

  ```nginx
  # /etc/nginx/nginx.conf

  ...

  http {
      ...

      include /etc/nginx/conf.d/*.conf;
  }
  ```

#### `/etc/nginx/conf.d/`

- 기본 HTTP 서버 설정 파일들이 위치한 폴더
- 이 폴더의 `.conf` 확장자를 가지는 파일들은 `/etc/nginx/nginx.conf` 파일의 `http` 블록에 포함
- `include` 선언으로 설정을 구성하는 것이 설정 파일들을 간결하게 관리하는 가장 좋은 방법

#### `/var/log/nginx/`

- 기본 로그 폴더
- `access.log` 파일, `error.log` 파일 포함

### NGINX 명령어

#### `nginx -h`

- 도움말

#### `nginx -v`, `nginx -V`

- 버전 확인

#### `nginx -t`, `nginx -T`

- NGINX 설정 테스트

#### `nginx -s signal`

- `signal`
  - `stop`
    - NGINX 프로세스 정지
  - `quit`
    - 현재 요청을 처리하는 프로세스가 끝나면 NGINX 프로세스 정지
  - `reload`
    - 설정 다시 불러오기
  - `reopen`
    - log 파일 다시 열기

## Serving Static Content

> NGINX에서 정적 컨텐츠 제공하기
