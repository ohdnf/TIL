const fs = require('fs').promises;

// 파일 만들기
fs.writeFile('./test.txt', '글을 입력합니다.')
  .then(() => {
    return fs.readFile('./test.txt');
  })
  .then((data) => {
    console.log(data.toString());
  })
  .catch((err) => {
    console.error(err);
  });
