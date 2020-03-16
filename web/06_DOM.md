# Document Object Model

## DOM
- 문서의 구조화된 표현을 제공하여, DOM 구조에 접근할 수 있는 방법을 제공
- 문서 구조, 스타일, 내용 등을 변경할 수 있도록 도우며, 구조화된 노드와 오브젝트로 문서를 표현
- 주요 객체
    - `window`
    - `document`
    - `navigator`, `location`, `history`, `screen`

### `window`
- 전역 객체
- 다양한 함수, 이름공간, 객체 등이 포함

### DOM 접근
- 단일 Node
    - `document.getElementByID(id)`
    - (중요) `document.querySelector(selector)`
- HTMLCollection(live)
    - `document.getElementsByTagName(class)`
    - `document.getElementsByClassName(class)`
- NodeList(non-live)
    - (중요) `document.querySelectorAll(selector)`

- HTMLCollection은 모두 live collection이며, 활용 시 주의를 요한다.
- NodeList는 경우에 따라 live collection일 수 있다.

### DOM 조작
