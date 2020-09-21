const path = require('path')

const string = __filename

// 경로 구분자 출력
console.log('path.sep:', path.sep)
// path.sep: \

// 환경 변수의 구분자
console.log('path.delimiter:', path.delimiter)
// path.delimiter: ;

console.log('----------------------------------------')

// 파일이 위치한 폴더 경로
console.log('path.dirname():', path.dirname(string))
// path.dirname(): C:\Users\multicampus\Workspace\TIL\nodejs

// 파일의 확장자
console.log('path.extname():', path.extname(string))
// path.extname(): .js

// 확장자를 포함한 파일의 이름을 표시
// 두 번째 인자에 확장명을 넣으면 파일의 이름만 표시
console.log('path.basename():', path.basename(string))
// path.basename(): path.js
console.log('path.basename - extname:', path.basename(string, path.extname(string)))
// path.basename - extname: path

console.log('----------------------------------------')

// 파일 경로를 root, dir, base, ext, name으로 분리
console.log('path.parse():', path.parse(string))
/*
path.parse(): {
  root: 'C:\\',
  dir: 'C:\\Users\\multicampus\\Workspace\\TIL\\nodejs',
  base: 'path.js',
  ext: '.js',
  name: 'path'
}
*/

// 객체를 파일 경로로 합침
console.log('path.format():', path.format({
  dir: 'C:\\Users\\multicampus',
  name: 'path',
  ext: '.js'
}))
// path.format(): C:\Users\multicampus\path.js

// \나 /를 실수로 여러 번 사용했거나 혼용했을 때 정상적인 경로로 변환
console.log('path.normalize():', path.normalize("C://Users\\mulitcampus\\Workspace\\TIL\\nodejs\\path.js"))
// path.normalize(): C:\Users\mulitcampus\Workspace\TIL\nodejs\path.js

console.log('----------------------------------------')

// 파일의 경로가 절대경로(true)인지 상대경로(false)인지를 true나 false로 반환
console.log('path.isAbsolute(C:\\):', path.isAbsolute('C:\\'))  // true
console.log('path.isAbsolute(./home):', path.isAbsolute('./home'))  // false

console.log('----------------------------------------')

// 첫 번째 인자로 넣은 경로에서 두 번째 인자로 넣은 경로로 가는 방법을 반환
console.log('path.relative():', path.relative('C:\\Users\\multicampus\\Workspace\\TIL\\nodejs\\path.js', 'C:\\'))
// path.relative(): ..\..\..\..\..\..

// 여러 인수를 넣으면 하나의 경로로 합침
console.log('path.join():', path.join(__dirname, '..', '..', '/boilerplate', '.', '/server'))
// path.join(): C:\Users\multicampus\Workspace\boilerplate\server

// `/`를 만나면 절대경로로 인식해 앞의 경로를 무시하고 처리
console.log('path.resolve():', path.resolve(__dirname, '..', '/django', '.', '/database'))
// path.resolve(): C:\database