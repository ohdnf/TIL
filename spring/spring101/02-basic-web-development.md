# 스프링 웹 개발 기초

## 정적 컨텐츠

- 프로그래밍 불가능(관련 Controller가 없음)
- `resources/static/` 안에 있는 `hello-static.html` 파일을 그대로 렌더링

```html
<!DOCTYPE html>
<html>
  <head>
    <title>static content</title>
    <meta http-equiv="Content-Type" content="text/html; charset=UTF-8" />
  </head>
  <body>
    정적 컨텐츠 입니다.
  </body>
</html>
```

- `http://localhost:8080/hello-static.html`로 실행

---

## MVC와 템플릿 엔진

- MVC: Model, View, Controller

### Controller

```java
@Controller
public class HelloController {

    ...

    @GetMapping("hello-mvc")
    public String helloMvc(@RequestParam("name") String name, Model model) {
        // http://localhost:8080/hello-mvc?name=spring
        // ?name=spring 과 같이 인자를 넘겨줘야함
        model.addAttribute("name", name);
        return "hello-template";
    }
}

```

### View

```html
<!-- resources/template/hello-template.html -->
<html xmlns:th="http://www.thymeleaf.org">
  <body>
    <p th:text="'hello ' + ${name}">hello! empty</p>
  </body>
</html>
```

- 실행: `http://localhost:8080/hello-mvc?name=spring`

---

## API

- `@RequestBody`를 사용
  - HTTP의 BODY에 문자 내용을 직접 반환
  - `viewResolver` 대신 `HttpMessageConverter`가 동작
  - 기본 문자처리: `StringHttpMessageConverter`
  - 기본 객체처리: `MappingJackson2HttpMessageConverter`
  - byte 처리 등 기타 여러 HttpMessageConverter가 기본으로 등록되어 있음

> 참고: 클라이언트의 HTTP Accept Header와 서버의 Controller 반환 타입 정보를 조합해서 `HttpMessageConverter`가 선택됨

```java
@Controller
public class HelloController {

    ...

    @GetMapping("hello-string")
    @ResponseBody
    public String helloString(@RequestParam("name") String name) {
        // 응답 body에 직접 name을 넣어줌
        return "hello" + name;
    }

    @GetMapping("hello-api")
    @ResponseBody
    public Hello helloApi(@RequestParam("name") String name) {
        Hello hello = new Hello();
        hello.setName(name);
        return hello;   // 기본 JSON 객체를 반환
    }

    static class Hello {
        private String name;

        public String getName() {
            return name;
        }

        public void setName(String name) {
            this.name = name;
        }
    }
}
```
