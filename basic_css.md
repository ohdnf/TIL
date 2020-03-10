# Cascading Style Sheets

## CSS 선언 방식

- style 적용 우선순위

    1. inline
    2. internal == external (나중에 선언된 속성이 반영)

- `inline`

    - 가장 우선적으로 반영
    - 해당 태그에 직접 지정
    - 적용되면 다른 방식은 무시됨

    ```html
    <!DOCTYPE html>
    <html>
        <head>
            <meta charset="utf-8"/>
            <title>Hello</title>
        </head>
        <body>
            <span style="color:red;">Hello, World!</span>
        </body>
    </html>
    ```

- `internal`

    - `<head>`안에 `<style>` 태그로 지정
    - 구조와 스타일이 섞이게 되므로 유지보수가 어려움
    - CSS파일을 관리하지 않아 브라우저가 서버에 CSS파일을 부르기 위해 별도의 요청을 보낼 필요가 없음

    ```html
    <!DOCTYPE html>
    <html>
        <head>
            <meta charset="utf-8"/>
            <title>Hello</title>
            <style>
            span {
                color: red;
            }
            </style>
        </head>
        <body>
            <span>Hello, World!</span>
        </body>
    </html>
    ```

- `external`

    - 외부파일(`.css`)로 지정
    - CSS 코드가 아주 짧지 않다면 가급적 이 방식으로 구현하는 것이 좋음
    - 현업에서는 여러 개의 CSS 파일을 분리하고 이를 합쳐서 서비스에 사용하기도 함


    `main.css` 파일
    ```css
    /*main.css*/
    span {
        color: red;
    }
    ```

    `home.html` 파일
    ```html
    <!DOCTYPE html>
    <html>
        <head>
            <meta charset="utf-8"/>
            <title>Hello</title>
            <link rel="stylesheet" type="text/css" href="main.css" />
        </head>
        <body>
            <span>Hello, World!</span>
        </body>
    </html>
    ```