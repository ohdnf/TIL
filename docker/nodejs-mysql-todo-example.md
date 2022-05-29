## Use bind mounts

### Start a dev-mode container

```sh
docker run -dp 3000:3000 -w /app -v "$(pwd):/app" node:12-alpine sh -c "apk add --no-cache python2 g++ make && yarn install && yarn run dev"
```

- `-dp 3000:3000` run in detached (background) mode and create a port mapping
- `-w /app` sets the "working directory" or the current directory that the command will run from
- `-v "$(pwd):/app"` bind mount the current directory(`$(pwd)`) from the host in the container into the `/app` directory
- `node:12-alpine` the image to use
- `sh -c "apk add --no-cache python2 g++ make && yarn install && yarn run dev"` start a shell using `sh` (alpine does not have `bash`) and run `yarn install` to install _all_ dependencies and then run `yarn run dev`. In the `package.json`, `dev` script starts `nodemon`

## Multi container apps

> each container should do one thing and do it well

### Container networking

- two ways to put a container on a network
  1. assign it at start
  2. connect an existing container

```sh
docker run -d --network todo-app --network-alias mysql --platform "linux/amd64" -v todo-mysql-data:/var/lib/mysql -e MYSQL_ROOT_PASSWORD=secret -e MYSQL_DATABASE=todos mysql:5.7
```

- `-v todo-mysql-data:/var/lib/mysql` docker automatically creates a named volume called todo-mysql-data. you can check where it mounted using `docker volume inspect todo-mysql-data`

### Debug networking issues

```sh
docker run -it --network todo-app nicolaka/netshoot
```

```sh
dig mysql

; <<>> DiG 9.16.27 <<>> mysql
;; global options: +cmd
;; Got answer:
;; ->>HEADER<<- opcode: QUERY, status: NOERROR, id: 27149
;; flags: qr rd ra; QUERY: 1, ANSWER: 1, AUTHORITY: 0, ADDITIONAL: 0

;; QUESTION SECTION:
;mysql.				IN	A

;; ANSWER SECTION:
mysql.			600	IN	A	172.18.0.2

;; Query time: 0 msec
;; SERVER: 127.0.0.11#53(127.0.0.11)
;; WHEN: Mon Apr 11 01:51:55 UTC 2022
;; MSG SIZE  rcvd: 44
```

- `--network-alias mysql` command option makes docker resolve host name `mysql` to `172.18.0.2`

### Run your app with MySQL

```sh
docker run -dp 3000:3000 -w /app -v "$(pwd):/app" --network todo-app -e MYSQL_HOST=mysql -e MYSQL_USER=root -e MYSQL_PASSWORD=secret -e MYSQL_DB=todos node:12-alpine sh -c "apk add --no-cache python2 g++ make && yarn install && yarn run dev"
```

## Use Docker Compose

- help define and share multi-container applications

```yml
services:
  app:
    image: node:12-alpine
    command: sh -c "apk add --no-cache python2 g++ make && yarn install && yarn run dev"
    ports:
      - 3000:3000
    working_dir: /app
    volumes:
      - ./:/app
    environment:
      MYSQL_HOST: mysql
      MYSQL_USER: root
      MYSQL_PASSWORD: secret
      MYSQL_DB: todos
  mysql:
    image: mysql:5.7
    volumes:
      - todo-mysql-data:/var/lib/mysql
    environment:
      MYSQL_ROOT_PASSWORD: secret
      MYSQL_DATABASE: todos

volumes:
  todo-mysql-data:
```

- Docker Compose automatically creates a network specifically for the application stack

```sh
docker-compose up
```

```sh
# stop containers and remove network
docker-compose down
# remove volumes too
docker-compose down --volumes
```
