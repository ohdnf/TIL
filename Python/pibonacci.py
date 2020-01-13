def pibonacci(n):
    seq = [1, 1]
    idx = 2
    while n > idx:
        seq.append(seq[idx - 2] + seq[idx - 1])
        idx += 1
    return seq


if __name__ == '__main__':
    n = int(input())
    print(pibonacci(n))