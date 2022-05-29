# 수제 모듈 로더

```js
const fs = require('fs');

function loadModule(filename, module, require) {
  var wrappedSrc =
    '(function(module, exports, require) {' +
    fs.readFileSync(filename, 'utf8') +
    '})(module, module.exports, require);';
  eval(wrappedSrc);
}

var require = function (moduleName) {
  console.log('Require invoked for module: ' + moduleName);
  var id = require.resolve(moduleName); // resolve full path of module, id
  // check cache if module loaded in the past
  if (require.cache[id]) {
    return require.cache[id].exports;
  }
  // set up the module metadata for the first load
  var module = {
    exports: {}, // export any public API
    id: id,
  };
  // update the cache
  require.cache[id] = module;
  // load the module
  loadModule(id, module, require); // 5
  // return exported variables
  return module.exports; // 6
};
require.cache = {};
require.resolve = function (moduleName) {
  /* resolve a full module id from the moduleName */
};
```

1. 모듈명을 인자로 받아 전체 경로를 이행
2. 기존 캐시에 저장된 적이 있다면 즉시 반환
3. 저장된 적이 없다면, 첫 저장을 위한 환경 구축. `exports` 속성을 지닌 `module` 객체는 public API를 해당 속성에 추가
4. `module` 객체를 캐싱
5. `loadModule` 함수로 파일에 있는 모듈 코드를 불러옴. `module` 객체와 `require` 함수에 대한 참조를 해당 모듈에 인자로 전달. 해당 모듈은 `module.exports` 객체를 조작하거나 대체함으로써 public API를 내보냄
6. `module.exports`가 호출자에게 반환됨
