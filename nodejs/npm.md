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
```

> [npm-install documentation 참고](https://docs.npmjs.com/cli/install)