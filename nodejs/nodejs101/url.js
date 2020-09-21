const url = require('url');

const { URL } = url;
const myURL = new URL('https://programmers.co.kr/job?_=1600690432262&job_position%5Bmin_career%5D=0&job_position%5Btags%5D%5B%5D=Python&page=1');
// WHATWG 방식의 url
// username, password, origin, searchParams 속성이 존재
console.log('new URL():', myURL);
console.log('url.format():', url.format(myURL));  // 분해되었던 url 객체를 다시 원래 상태로 조립
console.log('------------------------------------------------------------');
// node 방식의 url
const parsedURL = url.parse('https://programmers.co.kr/job?_=1600690432262&job_position%5Bmin_career%5D=0&job_position%5Btags%5D%5B%5D=Python&page=1');
// 주소를 분해
// username과 password 대신 auth 속성이, searchParams 대신 query가 존재
console.log('url.parse():', parsedURL);
console.log('url.format():', url.format(parsedURL));
