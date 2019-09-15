# 문자열 탐색

## 문자열 시작이나 끝부분과 일치하는지 검사
`str.startswith()`나 `str.endswith()` 사용

* 예시
```
import os
filenames = os.listdir('.')
[filename for filename in filenames if filenames.endswith(('.py', '.txt', '.md'))]
```
* 찾고 싶은 문자열을 배열 구조로 입력하고 싶다면 tuple 형태(ex. `()`)로 던져줘야한다.