# GROUP BY

## [입양 시각 구하기(2)](https://programmers.co.kr/learn/courses/30/lessons/59413)

> 레코드에 0시부터 23시까지 모든 시간 중 해당하는 시간이 없어도 count 컬럼을 0으로 출력하는 것이 관건

- `UNION`과 `LEFT JOIN` 활용

```sql
SELECT h1.hour, IFNULL(outs.count, 0) AS count
FROM (SELECT 0 AS hour
UNION SELECT 1 UNION SELECT 2 UNION SELECT 3 UNION SELECT 4 UNION SELECT 5
UNION SELECT 6 UNION SELECT 7 UNION SELECT 8 UNION SELECT 9 UNION SELECT 10
UNION SELECT 11 UNION SELECT 12 UNION SELECT 13 UNION SELECT 14 UNION SELECT 15
UNION SELECT 16 UNION SELECT 17 UNION SELECT 18 UNION SELECT 19 UNION SELECT 20
UNION SELECT 21 UNION SELECT 22 UNION SELECT 23) h1
LEFT JOIN (
SELECT HOUR(datetime) AS 'hour',
COUNT(*) AS 'count' FROM animal_outs
GROUP BY hour) AS outs ON h1.hour = outs.hour;
```

- 변수 설정: `@변수명 := 값`

```sql
set @HOUR = -1;

select (@hour := @hour +1) as HOUR, ( select count(*)
                                      from animal_outs
                                      where hour(datetime) = @hour ) as COUNT
from animal_outs
where @hour < 23;
```

