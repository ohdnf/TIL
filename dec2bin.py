# 십진수를 이진수로
def dec2bin(num):

    answer = 0

    if num % 2 == 1:
        answer += 1
        num -= 1

    while num > 1:
        temp = 1
        quotient = 0
        while num >= temp:
            temp *= 2
            quotient += 1

        num = num - temp // 2
        answer = answer + 10 ** (quotient - 1)

    return answer
