# Index

> 지정한 컬럼들을 기준으로 메모리 영역에 일종의 목차를 생성하는 것

- `INSERT`, `UPDATE`, `DELETE`의 성능을 희생시키는 대신, `SELECT`의 성능을 향상
- `UPDATE`, `DELETE` 쿼리 자체는 느릴 수 있지만 인덱스가 존재할 때 `UPDATE`, `DELETE`를 하기 위해 해당 데이터를 조회(`SELECT`)하는 것은 빠르다(다른 개념)
- 일반적으로 `B-tree` 인덱스를 사용한다.
  - B-tree: 인덱스 키(인덱스로 만들 테이블의 컬럼 값)와 이 키에 해당하는 컬럼 값을 가진 테이블의 로우가 저장된 주소 값으로 구성

## Refernce

https://jojoldu.tistory.com/243

https://coding-factory.tistory.com/419

http://www.mysqlkorea.com/sub.html?mcode=manual&scode=01&m_no=21712&cat1=7&cat2=219&cat3=251&lang=k