# Knapsack 알고리즘

## 배낭 문제에 동적 계획법 적용하기

> n개의 물건, 각 물건 i의 무게 wi, 가치 vi, 배낭의 용량 W일 때, 배낭에 담을 수 있는 물건의 최대 가치를 찾기
> 단, 배낭의 물건의 무게 합이 W를 초과하지 말아야 한다.



## 완전 검색

1. 완전 검색으로 물건들의 집합 S에 대한 모든 부분집합 구함
2. 부분집합에 포함된 물건들의 총 무게가 배낭 무게 W를 초과하는 집합들은 버리고, 나머지 집합에서 총 가치가 가장 큰 집합 선택
3. 부분집합을 생성하는 상태공간 트리를 탐색하는 방법으로 모든 후보해 탐색

```python
# W: 남은 배낭의 무게, k: 배낭에 넣을 물건(1...n), 방문하는 노드의 높이
# curValue: 현재까지 담은 물건의 총 가치, maxValue: 최대 가치
# n: 물건의 개수

def knapsack(W, k, curValue):
    global maxValue
    if W >= 0:
        if k > n:
            if maxValue < curValue:
                maxValue = curValue
        else:
            knapsack(W - weight[k], k+1, curValue + value[k])
            knapsack(W, k+1, curValue)
```



## 부분 문제 정의

두 개의 부분 문제(case1: n번째 물건을 담는 경우, case2: n번째 물건 담지 않는 경우)의 최적해를 사용해서 구함


`K[n][W]`: 배낭 무게 W를 1부터 n까지의 물건으로 채우는 최대 가치. 물건 1부터 i까지만 고려하고, (임시) 배낭 용량이 w일 때의 최대 가치
`K[n-1][W-wn]`: 배낭 무게 W-wn을 1부터 n-1까지 물건으로 채우는 최대 가치(case1)
`K[n-1][W]`: 배낭 무게 W를 1부터 n-1까지 물건으로 채우는 최대 가치(case2)

최적해 = max{ case1의 최대 가치 + vn, case2의 최대 가치 }
==> `K[n][W] = max(K[n-1][W-wn]+vn, K[n-1][W])`



## 재귀 알고리즘에 메모이제이션 적용

```python
# K[i][W]: 부분 문제의 해(최대 가치)를 저장하기 위한 리스트, -1로 초기화
# i: 배낭에 넣을 물건을 나타내는 값(1...n), W: 배낭의 무게
# n: 물건의 개수

def knapsack(i, W):
    # Memoization
    if K[i][W] != -1:
        return K[i][W]
    
    # 담을 물건이 없거나 배낭 용량이 0인 경우
    if i == 0 or W == 0:
        K[i][W] = 0
        return K[i][W]
    else:
        # n번째 물건을 담는 경우, 배낭 무게 W-wn을 1부터 n-1까지 물건으로 채우는 최대 가치
        case1 = 0
        if W >= weight[i]:
            case1 = knapsack(i-1, W-weight[i]) + value[i]
        
        # n번째 물건 담지 않는 경우, 배낭 무게 W를 1부터 n-1까지 물건으로 채우는 최대 가치
        case2 = knapsack(i-1, W)

        K[i][W] = max(case1, case2)
        return K[i][W]
```



## 동적 계획법 적용

동적 계획법을 적용해서 상향식으로 배낭 문제의 최적해를 구하려면 **부분 문제의 의존성**을 고려해야 함
i행의 값들을 구하기 위해서 **i-1행의 값이 미리 계산되어 저장**되어 있어야 함

i가 0인 공집합부터 시작해서 i값을 증가시켜가면서 구해 나감
상향식 문제 해결 --> 필요한 부분 문제가 어떤 것인지 알기 어려워 **모든 부분 문제들에 대한 해를 구해 나가야 함**
리스트를 행 우선으로 탐색하는 순서로 테이블을 채워나가면 의존성에 위배되지 않음(0번 물건)

```python
# W: 배낭의 무게
# i: 배낭의 넣을 물건 번호(1...n)
# n: 물건의 개수

def knapsack():
    for i in range(n+1):
        K[i][0] = 0
    for w in range(W+1):
        K[0][w] = 0
    
    for i in range(1, n+1):
        for w in range(1, W+1):
            if weight[i] > w:   # i번 물건을 담을 수 없는 경우
                K[i][w] = K[i-1][w]
            else:
                K[i][w] = max(K[i-1][w-weight[i]]+value[i], K[i-1][w])
    return K[n][W]
```



## 1차원 배열로 해결하기

```python
NUMBER, MAX_WEIGHT = map(int, input().split())    # 물건 개수, 가방 용량
stuff = dict()
for _ in range(NUMBER):
    weight, value = map(int, input().split())   # 물건의 무게, 가치
    stuff[weight] = value

dp = [0] * (MAX_WEIGHT+1)

for weight, value in stuff.items():
    for k in range(weight, MAX_WEIGHT+1):
        dp[k] = max(dp[k-weight]+value, dp[k])

print(dp[MAX_WEIGHT])
```



## 동전 거슬러 주기

```python
# n: 동전 종류 수
# m: 거슬러 줄 금액

n, m = map(int, input().split())
coin = list(map(int, input().split()))
# 1~m원을 거슬러 줄 최소 동전의 개수를 dp 배열로 생성
dp = [float('inf')] * (m+1) # 동전을 거슬러 줄 방법을 모두 무한대로 초기화
dp[0] = 0   # 0원을 거슬러 줄 방법은 0으로 초기화

for c in coin:
    for money in range(c, m+1):
        dp[money] = min(dp[money-c]+1, dp[money])

print(dp[m])    # m원을 거슬러 줄 최소 동전의 개수