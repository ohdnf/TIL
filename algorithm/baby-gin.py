# input data --> whitespace separated
# arr = list(map(int, input().split()))

numbers = list(input())
cnt = [0] * 10

for x in numbers:
    cnt[int(x)] += 1

run = triplet = 0
for i in range(8):
    if cnt[i] > 0 and cnt[i+1] > 0 and cnt[i+2] > 0:
        run += 1
        for j in range(3):
            cnt[i+j] -= 1


print(cnt)
