# ngrok

> _ngrok allows you to expose a web server running on your local machine to the internet. Just tell ngrok what port your web server is listening on._ 
>
> 로컬에서 돌리는 웹 서버를 외부에서 접속할 수 있게 해주는 프록시 서버



### `cmd`에서 돌리기

> http 서버를 local의 5000포트로 터널을 열어준다.

```shell
$ ngrok http 5000
```

```shell
ngrok by @inconshreveable                                                                               (Ctrl+C to quit)

Session Status                online
Session Expires               7 hours, 59 minutes
Version                       2.3.35
Region                        United States (us)
Web Interface                 http://127.0.0.1:4040
Forwarding                    http://79e3cf2e.ngrok.io -> http://localhost:5000
Forwarding                    https://79e3cf2e.ngrok.io -> http://localhost:5000

Connections                   ttl     opn     rt1     rt5     p50     p90
                            0       0       0.00    0.00    0.00    0.00
```


