# 우아한 Redis

[YouTube 우아한 테크 세미나](https://youtu.be/mPB2CZiAkKM) 191121 by 강대명님

## Redis 소개

- In-Memory Data Structure Store
- Open Source(BSD 3 License)
- Support data structrues
  - Strings, set, sorted-set, hashes, list
  - Hyperloglog, bitmap, gespatial index
  - Stream
- Only 1 Committer

> ### Cache
>
> 나중 요청을 위해 결과를 미리 저장해두었다가 빠르게 서비스 해주는 것
>
> ex. Dynamic Programming => Factorial의 계산
>
> 10! = 10 _ 9 _ ...
>
> 21! = 21 _ 20 _ ... _ 10 _ 9 \* ...
>
> ...
>
> 20880! = ...?
>
> 20881! = 20881 \* 20880!
>
> #### CPU Cache 사진 넣기
>
> ![]()
>
> #### 캐시를 사용하는 이유
>
> 전체 요청의 80%는 20%의 사용자가 요청한다!(파레토의 법칙)
>
> #### Cache 구조
>
> ##### Look aside Cache
>
> ##### Write Back Cache
>
> 어느 것이 더 효율적일까?
>
> INSERT 1줄 \* 500번
>
> vs
>
> INSERT 500줄 \* 1번
>
> - 단점
>   - 메모리 저장 -> 장애가 발생하면 날아감
>   - log를 DB에 저장해서 보완할 수 있음

## 왜 Collection이 중요한가?

> Memcached는 Collection을 제공하지 않는다.

- 개발의 편의성
- 개발의 난이도
- 외부의 Collection을 잘 이용하는 것으로 여러가지 개발 시간을 단축시키고 문제를 줄여줄 수 있기 때문에 Collection이 중요

## Redis Collections

## Redis 운영

## Redis 데이터 분산

## Redis Failover
