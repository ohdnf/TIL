# Authentication: 인증

## `hashlib` 모듈

```python
import hashlib
m = hashlib.sha256()
m.update(b'test password')
m.hexdigest()
print(m)
```


## 단방향 해시 암호 알고리즘 해킹 방법과 방어

### rainbow attack

- 미리 해시값들을 계산해 놓은 테이블인 `rainbow table`을 생성한 다음 해시 함수 값을 역추적해서 본래 값을 찾아내는 방법

### Salting

- 실제 비밀번호에 추가적으로 랜덤 데이터를 더해서 해시 값을 계산하는 방법으로 `rainbow attack`처럼 미리 해시 값을 계산하여 해킹하는 공격들을 무효화

### Key stretching

- 단방향 해시 값을 계산한 후 그 해시 값을 또 해시하는 것을 여러 번 반복하는 방법

- 컴퓨터 성능이 더 향상되더라도 키 스트레칭 횟수를 추가하여 보완 가능



## `bcrypt` 모듈

```python
import bcrypt
a = bcrypt.hashpw(b'secret password', bcrypt.gensalt())
print(a)
b = bcrypt.hashpw(b'secret password', bcrypt.gensalt()).hex()
print(b)
```



## `PyJWT` 모듈

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