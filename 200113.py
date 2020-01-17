# 리스트 출력
# 1부터 100까지 원소 사이에 ', '(쉼표 공백) 넣어서 출력하기
l = [num for num in range(1, 100) if num % 2 == 1]
print(*l, sep=', ')                 # *(리스트)는 리스트를 unpack해준다
# print(", ".join(map(str, l)))     # .join()은 리스트를 ', '로 연결시켜주며, 이때 원소가 str 형식이어야 한다

# 십진수를 이진수로
num = int(input())

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

print(answer)


# 가변형 매개변수 사용하기
# 가장 마지막 매개변수로 지정해야함
def calc_sum(precision, *params):
    if precision == 0:
        total = 0
    elif 0 < precision < 1:
        total = 0.0
    for val in params:
        total += val
    return a + b + total


# 키워드 언팩 연산자(**)
def use_keyword_arg_unpacking(**params):
    for k in params.keys():
        print("{0}: {1}".format(k, params[k]))

print("use_keyword_arg_unpacking()의 호출")
use_keyword_arg_unpacking(a=1, b=2, c=3)


# 기본 값을 가지는 매개변수는 일반 매개변수 앞에 위치할 수 없다
# 이름이 같을 경우 지역변수 다음에 전역변수에 접근하므로 전역변수가 접근하지 못할 경우 발생 가능
# 함수 내에서 전역변수에 접근하려면 변수명 앞에 global 선언을 해주면 된다.

# 람다식
# lambda 매개변수: 반환값
def calc(operator_fn, x, y):
    return operator_fn(x, y)

ret_val = calc(lambda x, y: x + y, 10, 5)
print(ret_val)

ret_val = calc(lambda x, y: x - y, 10, 5)
print(ret_val)


# 클로저
# 중첩함수 자체를 반환값으로 사용해 정보 은닉 구현
def outer_func():
    id = 0

    def inner_func():
        nonlocal id
        id += 1
        return id

    return inner_func       # 함수 호출이 아닌 함수에 대한 참조를 반환

make_id = outer_func()