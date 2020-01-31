# 조합
def combi(a, r):
    if len(a)==r:
        return [a]
    elif r==0:
        return [[]]
    else:
        return combi(a[1:], r) + [ [a[0]] + x for x in combi(a[1:], r-1) ]


if __name__ == '__main__':
    # print(combi(['a', 'b', 'c', 'd'], 4))
    print(combi(['a', 'b', 'c', 'd', 'e'], 3))
    # print(combi(['a', 'b', 'c', 'c', 'd'], 3))
    # print(combi(['a', 'b', 'c', 'd'], 2))
    # print(combi(['a', 'b', 'c', 'd'], 1))
    # print(combi(['a', 'b', 'c', 'd'], 0))
    