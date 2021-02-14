from typing import List

def count_lis(seq: List) -> int:
    dp = [0] * (len(seq) + 1)   # seq[i]를 마지막 값으로 가지는 가장 긴 LIS의 길이
    seq.insert(0, -1)           # dp와 길이를 맞추기 위해 맨 앞에 -1 추가

    for i in range(1, len(seq)):
        max_length = 0
        for j in range(0, i):
            if seq[j] < seq[i]:
                dp[i] = max(dp[i], dp[j] + 1)
        print(i, dp)
    return max(dp)

def my_lis(seq: List) -> List:
    # dp = [0] * (len(seq) + 1)   
    # seq.insert(0, 0)            
    candidates = [[seq[0]]]

    for element in seq:
        temp = []
        for candidate in candidates:
            if candidate[-1] < element:
                temp.append(candidate + [element])  # +는 새로운 배열 객체를 만든다
            else:
                if len(candidate) == 1 or candidate[-2] < element:
                    candidate.pop()
                    candidate.append(element)
        candidates.extend(temp)
        print(f'element {element} has been processed: ', candidates)
    
    candidates.sort(key=len)
    return candidates[-1]

import math

def lis(seq: List) -> List:
    M = [0] * (len(seq) + 1)    # M[j]: 길이가 j인 증가부분수열의 마지막 원소의 seq 인덱스 k
    P = [0] * len(seq)          # P[k]: 증가부분수열의 마지막 원소인 seq[k]의 바로 앞 원소의 인덱스

    L = 0   # 지금까지 찾아낸 최장증가수열(LIS)의 길이
    for i in range(len(seq)):
        # Binary search for the largest positive j <= L
        # such that seq[M[j]] <= seq[i]
        # seq[i]가 어떤 증가부분수열의 마지막 값이 되기 위해서는
        # seq[i]가 추가되기 전 증가부분수열의 마지막 값(seq[M[j]])이 seq[i]보다 작은 값이어야 한다.
        lo = 1
        hi = L
        # print(f'start binary-searching between 0 ~ {L}')
        while lo <= hi:
            mid = math.ceil((lo+hi)/2)
            # print(f'lo = {lo}, hi = {hi}, mid = {mid}')
            if seq[M[mid]] < seq[i]:
                lo = mid + 1
                # print(f'seq[M[{mid}]] < seq[{i}] => {seq[M[mid]]} < {seq[i]}')
                # print(f'lo = {lo}')
            else:
                hi = mid - 1
                # print(f'seq[{M[mid]}] > seq[{i}] => {seq[M[mid]]} > {seq[i]}')
                # print(f'hi = {hi}')
        
        # print('finish binary-searching...')
        # After searching, lo is 1 greater than the
        # length of the longest prefix of seq[i]
        # 이분 탐색 후 seq[i]는 seq[M[j]] 중
        # seq[i]보다 작은 원소의 앞에 위치
        new_L = lo
        # print(f'new_L = {new_L}')

        # The predecessor of seq[i] is the last index of
        # the subsequence of length (new_L - 1)
        # seq[i]의 predecessor(P[i])는 바로 이전 크기의 부분수열의 마지막 원소
        # new_L 길이의 최장증가수열의 마지막 원소의 seq 인덱스는 i
        P[i] = M[new_L - 1]
        M[new_L] = i
        # print(f'P[{i}] = M[{new_L - 1}] = {P[i]}')
        # print(f'M[{new_L}] = {i}')

        if new_L > L:
            # If we found a subsequence longer than any we've
            # found yet, update L
            # seq[i]를 마지막 값으로 가지는 증가부분수열의 길이가 
            # 지금까지 찾아낸 증가부분수열의 길이 중 가장 길 경우 업데이트
            L = new_L
            # print(f'new_L > L => L = {new_L}')
        
        # print('P', P)
        # print('M', M)
        # print('L', L)
        # 현재까지 알아낸 최장증가수열
        # temp = [0] * L
        # k = M[L]
        # for i in range(L-1, -1, -1):
        #     temp[i] = seq[k]
        #     k = P[k]
        # print('current LIS:', temp)
        # print()
    
    # Reconstruct the longest increasing subsequence
    LIS = [0] * L
    k = M[L]
    for i in range(L-1, -1, -1):
        LIS[i] = seq[k]
        k = P[k]
    
    return LIS

if __name__ == "__main__":
    print(lis([3, 5, 7, 9, 2, 1, 4, 8]), [3, 5, 7, 8])
    print(count_lis([3, 5, 7, 9, 2, 1, 4, 8]))
    print(lis([10, 20, 40, 25, 20, 50, 30, 70, 85]), [10, 20, 25, 30, 70, 85])
    print(count_lis([10, 20, 40, 25, 20, 50, 30, 70, 85]))
    print(lis([0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15]), [0, 2, 6, 9, 11, 15])
    print(count_lis([0, 8, 4, 12, 2, 10, 6, 14, 1, 9, 5, 13, 3, 11, 7, 15]))
