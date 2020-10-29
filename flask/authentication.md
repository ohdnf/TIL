# Authentication: 인증

## 사용자 비밀번호 암호화

### `hashlib` 모듈

Python에서 제공하는 단방향 해시 함수 모듈

```python
import hashlib
m = hashlib.sha256()
m.update(b'test password')
m.hexdigest()
print(m)
```

### 단방향 해시 암호 알고리즘 해킹 방법과 방어

단방향 해시 암호 알고리즘은 복호화할 수 없어 원본 값을 구할 수 없다고 생각되지만 그렇지 않다.

#### 공격: Rainbow attack

미리 해시값들을 계산해 놓은 테이블인 `rainbow table`을 생성한 다음 해시 함수 값을 역추적해서 본래 값을 찾아내는 해킹 방법

> 해시 함수는 원래 패스워드를 저장하기 위해 설계된 것이 아니라 짧은 시간 안에 데이터를 검색하기 위해 설계된 것
> 따라서 공격자는 매우 빠른 속도로 임의의 문자열의 값을 해시화해서 해킹할 대상의 해시값과 비교할 수 있다.
> MD5 단방향 해시 알고리즘을 사용한 경우 일반적으로 1초당 56억 개의 해시 값을 대입할 수 있다.

#### 방어1: Salting

실제 비밀번호에 추가적으로 랜덤 데이터를 더해서 해시 값을 계산하는 방법으로 `rainbow attack`처럼 미리 해시 값을 계산하여 해킹하는 rainbow attack을 무효화

#### 방어2: Key stretching

- 단방향 해시 값을 계산한 후 그 해시 값을 또 해시하는 것을 여러 번 반복하는 방법
- 컴퓨터 성능이 더 향상되더라도 키 스트레칭 횟수를 추가하여 보완 가능



### `bcrypt` 모듈

salting과 key stretching을 구현한 해시 함수 라이브러리

```python
import bcrypt
a = bcrypt.hashpw(b'secret password', bcrypt.gensalt())
print(a)
b = bcrypt.hashpw(b'secret password', bcrypt.gensalt()).hex()
print(b)
```



## access token

- 사용자가 로그인에 성공한 후에
- 사용자의 로그인 여부를 확인하기 위해
- 백엔드 API 서버에서 프론트엔드에 전송하는 데이터



### JWT

- JSON Web Token
- access token을 생성하는 방법 중 하나



#### 구조

```
header.payload.signature
```

##### header

토큰 타입, 해시 알고리즘을 지정

```json
{
    "typ": "JWT",
    "alg": "HS256"
}
```

##### payload

실제 전송할 데이터

```json
{
    "user_id": 2,
    "exp": 1539517391
}
```

##### signature

JWT의 원본 확인 용도

```
Base64URL
```





### `PyJWT` 모듈

```python
import jwt
data_to_encode = {"some": "payload"}
encryption_secret = 'secrete'
algorithm = 'HS256'
encoded = jwt.encode(data_to_encode, encryption_secret, algorithm=algorithm)
print(encoded) # b'eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJzb21lIjoicGF5bG9hZCJ9.j4hydZvraNFUqUHpXw0hYBN9qTRzbm9-yS9h5skNht0'
decoded = jwt.decode(encoded, encryption_secret, algorithms=[algorithm])
print(decoded) # {'some': 'payload'}
```