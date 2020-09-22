/**
 * uncaughtException은 처리하지 못한 모든 에러를 잡아내지만
 * 이벤트 발생 후 다음 동작이 제대로 동작하는지 보증하지 않는다.
 * 단순히 에러 내용을 기록하는 용도로 사용하고
 * process.exit()으로 프로세스 종료하는 것이 좋다.
 */

process.on('uncaughtException', (err) => {
  console.error('예기치 못한 에러', err);
});

setInterval(() => {
  throw new Error('서버를 고장내주마!');
}, 1000);

setTimeout(() => {
  console.log('실행됩니다');
}, 2000);