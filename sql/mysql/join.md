# JOIN

## [없어진 기록 찾기](https://programmers.co.kr/learn/courses/30/lessons/59042)

```sql

-- 코드를 입력하세요
SELECT animal_id, name 
    FROM animal_outs AS outs 
    WHERE animal_id NOT IN (
        SELECT animal_id 
        FROM animal_ins AS ins
        )
    ORDER BY outs.animal_id;