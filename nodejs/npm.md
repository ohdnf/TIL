# npm

node package manager

## `npm install`

```shell
# package를 ./node_modules에 설치하고 ./package.json의 dependencies 속성에 package 정보 저장
# production 빌드 시 해당 package를 포함
npm install --save

# package를 ./node_modules에 설치하고 ./package.json의 devDependencies 속성에 package 정보 저장
# production 빌드 시 해당 package를 미포함
npm install --save-dev
# 줄여서
npm i -D

# 전역 설치 ==> 명령어로 사용하기 위해
# npm이 설치되어 있는 폴더(C:\Users\사용자이름\AppData\Roaming\npm)에 설치
npm install --global
# 줄여서
npm i -g
```

> [npm-install documentation 참고](https://docs.npmjs.com/cli/install)
