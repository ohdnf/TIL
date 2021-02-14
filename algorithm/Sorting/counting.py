def counting(arr):
    counts = [0] * (max(arr) + 1)
    for num in arr:
        counts[num] += 1
    # print('1st counts:', counts)
    for idx in range(1, len(counts)):
        counts[idx] += counts[idx-1]
    # print('2nd counts:', counts)
    result = [0] * len(arr)
    for num in arr[::-1]:
        temp = counts[num] - 1
        result[temp] = num
        counts[num] -= 1
        # print(result)
    return result


a = [0, 4, 1, 3, 1, 2, 4, 1]
print(a)
res = counting(a)
# print('-'*10)
print(res)
