def foo():
    arr += [3]        # 지역변수
    # arr.append(3)   # 전역변수
    print(arr)

if __name__ == '__main__':
    arr = [1, 2]
    foo()
    print(arr)