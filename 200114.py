def multiply(*args):
    try:
        result = 1
        for val in args:
            result *= val
        return result
    except:
        print("에러발생")


if __name__ == '__main__':
    #res = multiply(1, 2, '4', 3)
    #print(res)

    for item in dict(zip(list('abcdef'), list(range(6)))).items():
        print("{0}: {1}".format(item[0], item[1]))