# HTML5의 이해

## 1. HTML5의 탄생과 의미

### HTML이란

- Hyper Text Markup Language의 약자
- 웹용 콘텐츠의 구조를 지정하는 컴퓨터 언어
- HTML은 웹서버에 저장되며 클라이언트 웹 브라우저에 읽혀지고 해석되어 화면에 보여진다.

### HTML의 탄생과 발전

- 1990년 World Wide Web과 함께 탄생
- 웹의 발전으로 웹 브라우저의 중요성 확대
- 'Netscape Navigator'와 'Microsoft Internet Explorer'의 시장 점유율 전쟁으로 HTML과 CSS의 비표준 심화
- 새로운 웹 브라우저가 등장하면서 웹의 표준화 논의
- 웹 표준을 개발하고 논의하며 제정하는 W3C(World Wide Web Consortium) 등장
- 1997년 W3C HTML3.2 권고 - 시장 요구에 의해 비표준 HTML 포함
- 1999년 HTML 4.01 권고 - 시장 요구에 의해 다양한 버전 등장
- 2000년 XHTML 권고 - XML의 엄격함을 주요 내용으로 한 웹 표준
- W3C는 XHTML2 개발 추진하다 돌연 HTML5로 방향 전환

### HTML5의 탄생

- 기존 HTML의 한계

    - 웹 기반 사업과 기술의 발전을 못 따라간 HTML
    - 동적인 웹을 위한 표준화된 기술을 요구
    - W3C의 XHTML2로의 발전 예고

- WHATWG(Web Hypertext Application Technology Working Group) 탄생

    - 시장 요구에 부응하지 못하는 W3C에 실망한 웹 관련 업체와 단체가 자체적으로 만든 워킹 그룹
    - 웹 기술과 시장의 요구를 분석하여 HTML5 명세서 작업 착수

- W3C의 HTML5 수용

    - 2008년 W3C HTML5 초안 발표
    - 2009년 XHTML2 개발 중단
    - 2014년 XHTML5 권고 예정

- HTML5의 특징

    - 기존 HTML과의 호환성 유지
    - 실용적 설계: 느슨한 문법, 효율적인 추가 요소, 안전한 보안
    - 표현과 내용의 완벽한 분리
    - 플러그인 없이 각종 미디어 처리 및 동적인 작동: 캔버스
    - 최신 웹 기술 수용: 지오로케이션, 웹소켓, 웹스토리지, 웹워커 등


## 2. HTML5 시작하기

### Doctype 지정하기

- HTML은 여러 버전이 존재하므로 Doctype을 명시해야 한다.
- 기존 Doctype은 매우 길고 복잡한 DTD를 명시해야 했다.
- HTML5의 실용성 원칙으로 인해 짧아졌다.
- `<!DOCTYPE html>`

### HTML 작성 규칙

- HTML의 마크업 명령은 요소라 부른다.
- HTML은 대소문자르 구분하지 않는다.
- 요소는 콘텐츠와 구분을 위해서 꺽쇠로 둘러싼다.
    - 태그 `<p>`, `<a>`, `<div>`
- 시작태그와 마침태그로 요소의 범위를 지정한다.
    - `<p>이것은 단락입니다.</p>`
- 마침태그가 없이 단독으로 사용되는 요소도 있다.
    - `<br>`, `<img>`, `<meta>` 등
- 요소의 속성은 `속성명="속성값"` 형식으로 기술한다.
    - `<img src="img/logo.jpg" alt="Company Logo">

### 이벤트 API

```html
<!DOCTYPE html>
<html>
    <head>
    </head>
    <body>
    </body>
</html>
```

- `<html>`은 HTML 코드 전체를 감싼다.
- HTML은 `<head>`와 `<body>` 부분으로 나뉜다.
- `<head>`는 메타데이터와 스크립, CSS 등이 위치한다.
- `<body>` 부분은 콘텐츠가 담기는 곳으로 웹 브라우저에 표시된다.

## 3. HEAD 설정

### 타이틀 지정

- HTML 파일의 제목으로 웹 브라우저 타이틀에 나타난다.
- `<title>웹 페이지 제목</title>`

### 문자 인코딩

- 사용하는 텍스트 에디터의 문자 인코딩과 HTML의 문자 인코딩이 동일해야 웹 브라우저에서 올바르게 표시된다.
- 유니코드인 UTF-8로 지정한다.

### 메타데이터

- 메타데이터를 기술하면 웹 검색에 유리하다.
- HTML에 대한 정보를 기록할 수 있다.
- 메타데이터 기술 방법

    ```html
    <meta name="description" content="HTML5와 Javascript 학습콘텐츠">
    <meta name="keywords" content="HTML5, CSS, JavaScript">
    <meta name="author" content="Jupyo Hong">
    <meta name="copyright" content="(c)2020 Jupyo Hong">
    <meta name="reply-to" content="jupyohong7@gmail.com">
    <meta name="date" content="2020-03-09T13:19:30+09:00">
    ```

## 4. 외부 파일 연결

### HTML과 함께 사용되는 CSS와 자바스크립트는 다른 파일로 분리함이 원칙이다.

### 외부 CSS 파일 연결

- `<link rel="styleshee" href="css/style.css">`

### 외부 자바스크립트 파일 연결

- `<script src="js/script.js"></script>`

