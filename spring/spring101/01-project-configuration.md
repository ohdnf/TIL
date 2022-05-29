# 프로젝트 환경설정

## 프로젝트 생성: [spring initializr](https://start.spring.io/)

스프링 부트 기반 스프링 프로젝트를 생성해주는 사이트

### Project

필요한 라이브러리를 땡겨오고, 빌드하는 라이프사이클을 관리해주는 툴

- `Maven Project`
  - 레거시, 과거 프로젝트
- `Gradle Project`
  - 요즘 추세
  - 스프링 라이브러리 관리까지

### Spring Boot 버전

- SNAPSHOT: 현재 만들고 있는 버전
- M1: 아직 정식 릴리즈되지 않은 버전

### Project Metadata

- `Group`: 기업 도메인(거꾸로)
- `Artifact`: 결과물(프로젝트명)

### Dependencies(중요)

내가 스프링 부트 기반으로 프로젝트를 시작할 때 **어떤 라이브러리를 쓸 것인지**

- Spring Web
- Thymeleaf: HTML을 만들어주는 Template Engine

---

## 라이브러리 살펴보기

### 스프링 부트 라이브러리

- `spring-boot-starter-web`
  - `spring-boot-starter-tomcat`: WAS 라이브러리(웹 서버)
  - `spring-webmvc`: 스프링 웹 MVC
- `spring-boot-starter-thymeleaf`: 템플릿 엔진(View)
- `spring-boot-starter`(공통): spring boot + spring core + logging
  - `spring-boot`
    - `spring-core`
  - `spring-boot-starter-logging`
    - `logback`: log를 어떤 구현체로 출력할건지 결정해줌
    - `slf4j`: 인터페이스

### 테스트 라이브러리

- `spring-boot-starter-test`
  - `junit`: 5 version으로 넘어가는 추세
  - `mockito`: 목 라이브러리
  - `assertj`: 테스트 코드를 좀 더 편하게 작성하게 도와주는 라이브러리
  - `spring-test`: junit과 스프링 통합 테스트 지원

---

## View 환경설정

### Welcome Page 만들기

#### `resources/static/index.html`

```html
<!DOCTYPE html>
<html>
  <head>
    <title>Hello</title>
    <meta http-equiv="Content-Type" content="text/html; charset=UTF-8" />
  </head>
  <body>
    Hello
    <a href="/hello">hello</a>
  </body>
</html>
```

- 스프링 부트가 제공하는 Welcome Page 기능 - `static/index.html`을 올려두면 Welcome page 기능을 제공한다. - [공식 문서](https://docs.spring.io/spring-boot/docs/2.3.1.RELEASE/reference/html/spring-bootfeatures.
  html#boot-features-spring-mvc-welcome-page) 참고

### thymeleaf 템플릿 엔진

> 템플릿 엔진? HTML 코드에 Loop를 넣거나 변수를 활용할 수 있게 해줌

```java
// java/hello.hellospring/controller/HelloController.java

package hello.hellospring.controller;

import org.springframework.stereotype.Controller;
import org.springframework.ui.Model;
import org.springframework.web.bind.annotation.GetMapping;

// 컨트롤러는 annotation을 적어줘야 함
@Controller
public class HelloController {

    // HTTP Method annotation
    @GetMapping("hello")
    public String hello(Model model) {
        model.addAttribute("data", "hello!");
        // resources/templates 내 hello.html을 렌더링
        return "hello";
    }
}

```

```html
<!-- resources/templates/hello.html -->
<!DOCTYPE html>
<!-- Thymeleaf 템플릿 엔진 선언 -->
<html xmlns:th="http://www.thymeleaf.org">
  <head>
    <title>Hello</title>
    <meta http-equiv="Content-Type" content="text/html; charset=UTF-8" />
  </head>
  <body>
    <p th:text="'안녕하세요. ' + ${data}">안녕하세요. 손님</p>
  </body>
</html>
```

- 동작 확인

  - 실행: `http://localhost:8080/hello`

- 컨트롤러에서 `return`값으로 문자를 반환하면 `viewResolver`가 화면을 찾아서 처리한다.
  - 스프링 부트 템플릿엔진 기본 `viewName` 매핑
  - `resources:templates/` + `{ViewName}` + `.html`

> 참고: `spring-boot-devtools` 라이브러리를 추가하면 `html` 파일을 컴파일만 해주고 서버 재시작 없이 View 파일 변경이 가능하다.
> IntelliJ 컴파일 방법: build -> Recompile

## Build하고 실행하기

1. 빌드 명령어 실행
   - Mac OS: `./gradlew build`
   - Windows OS: IntelliJ 터미널(프로젝트 실행 경로)에서 `gradle build` 입력한 상태에서 `Ctrl + Enter`
2. `cd build/libs`
3. `java -jar hello-spring-0.0.1-SNAPSHOT.jar` 명령어로 실행
