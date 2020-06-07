# HyperText Transfer Protocol

## Stateless & Connectless

## Methods(Verbs)

- `GET`
    - 특정 리소스의 표시
    - `<a>`, `<form>`, 브라우저에서 주소창을 보내는 요청 등
    - URL을 활용(쿼리스트링)하여 데이터를 전송함
        - 예: `https://www.naver.com/?q=나이키`
        - 크기 제약 및 보안 이슈가 있음

- `POST`
    - 특정 리소스에 제출(서버의 상태 변화)
    - 보통 HTML Form을 통해 서버에 전송, 서버의 데이터 변경이 발생
    - HTTP 요청 메시지의 body에 데이터를 전송함

## Status Code

> 2XX대: Successful responses

- `200 OK`
    The request has succeeded. The meaning of the success depends on the HTTP method:
    - `GET`: The resource has been fetched and is transmitted in the message body.
    - `HEAD`: The entity headers are in the message body.
    - `PUT` or `POST`: The resource describing the result of the action is transmitted in the message body.
    - `TRACE`: The message body contains the request message as received by the server

> 3XX대: Redirection messages

- `301 Moved Permanently`
    The URL of the requested resource has been changed permanently. The new URL is given in the response.

- `302 Found`
    This response code means that the URI of requested resource has been changed temporarily. Further changes in the URI might be made in the future. Therefore, this same URI should be used by the client in future requests.

> 4XX대: Client error responses

- `400 Bad Request`
    The server could not understand the request due to invalid syntax.

- `401 Unauthorized`
    Although the HTTP standard specifies "unauthorized", semantically this response means "unauthenticated". That is, the client must authenticate itself to get the requested response.

- `403 Forbidden`
    The client does not have access rights to the content; that is, it is unauthorized, so the server is refusing to give the requested resource. Unlike 401, the client's identity is known to the server.

- `404 Not Found`
    The server can not find the requested resource. In the browser, this means the URL is not recognized. In an API, this can also mean that the endpoint is valid but the resource itself does not exist. Servers may also send this response instead of 403 to hide the existence of a resource from an unauthorized client. This response code is probably the most famous one due to its frequent occurrence on the web.

- `405 Method Not Allowed`
    The request method is known by the server but has been disabled and cannot be used. For example, an API may forbid DELETE-ing a resource. The two mandatory methods, GET and HEAD, must never be disabled and should not return this error code.

> 5XX대: Server error responses
- `500 Internal Server Error`
    The server has encountered a situation it doesn't know how to handle.

### 참고

- [MDN web docs: HTTP response status codes](https://developer.mozilla.org/en-US/docs/Web/HTTP/Status)
- [Tutorials Point: HTTP - Status Codes](https://www.tutorialspoint.com/http/http_status_codes.htm)

## Cookie