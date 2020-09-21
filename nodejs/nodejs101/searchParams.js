const { URL } = require('url');

const myURL = new URL('https://programmers.co.kr/job?_=1600690432262&job_position%5Bmin_career%5D=0&job_position%5Btags%5D%5B%5D=Python&page=1');
console.log('searchParams:', myURL.searchParams);
console.log('searchParams.keys():', myURL.searchParams.keys());
console.log('searchParams.values():', myURL.searchParams.values());
console.log('searchParams.toString():', myURL.searchParams.toString());
myURL.search = myURL.searchParams.toString();