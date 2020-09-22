const fs = require('fs').promises;  // fs 모듈은 프로미스 형식으로 바꿔주는 방법을 사용

// 파일 읽기
fs.readFile('./test.txt')   // 경로는 node 명령어 실행하는 콘솔 기준
  .then((data) => {
    console.log(data);
    console.log(data.toString());
  })
  .catch((err) => {
    console.error(err);
  });
