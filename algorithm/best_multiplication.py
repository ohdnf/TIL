def solution(number):
    """
    어떤 수를 두 수 이상의 합으로 나타내는 방법 중 해당 수들의 곱이 가장 큰 값 찾기
    예시> 8
    1+1+1+1+1+1+1+1
    2+2+2
    1+2+2+3
    2+3+3
    ...
    방법들 중 2+3+3 일 때 2*3*3 = 18로 그 합이 가장 크다.
    """
    dp = {1: 1, 2: 1, 3: 2, 4: 4, 5: 6, 6: 9}
    if number >= 7:
        for num in range(7, number+1):
            dp[num] = max(dp[num-2]*2, dp[num-3]*3)
            # dp[num] = max((num-2)*2, dp[num-2]*2, (num-3)*3, dp[num-3]*3)
    return dp[number]

if __name__ == "__main__":
    for n in range(1, 101):
        res = solution(n)
        print(f'{n} ==> {res}')
