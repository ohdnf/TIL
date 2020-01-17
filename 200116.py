# set = set()
# {}는 딕셔너리의 리터럴이기 때문에 빈 set 객체는 set()으로 표현
# 중복된 값을 단일 항목으로 저장해주는 배열 객체

set1 = {1, 2, 3, 4, 5}
set2 = {3, 4, 5, 6, 7}

# 교집합
set1 & set2 == set1.intersection(set2)
# 합집합
set1 | set2 == set1.union(set2)
# 차집합
set1 - set2 == set1.difference(set2)

# 항목 추가
set1.add(6)
set2.update({8, 9, 10})
# 항목 제거
set1.remove(1)
set2.pop()
set3 = set2 - set1
set3.clear()
# 항목 확인
# set1이 set2를 전부 포함하는 집합인지 확인
set1.issuperset(set2)
# set1이 set2에 전부 포함되는 집합인지 확인
set1.issubset(set2)


# dict = dict() = {}
# 인덱스X / 중복X / 순서X
heroes1 = dict(홍길동=20, 이순신=45, 강감찬=35)
tuple1 = (("홍길동", 20), ("이순신", 45), ("강감찬", 35))
list1 = [["홍길동", 20], ["이순신", 45], ["강감찬", 35]]
heroes2 = dict(tuple1)
heroes3 = dict(list1)

# 항목 추가
heroes1["을지문덕"] = 40
heroes2.update({"신사임당": 50, "유관순": 16})
# 항목 변경
heroes3["강감찬"] = 38
heroes3.update({"이순신": 48, "홍길동": 25,})
# 항목 제거
del heroes1["강감찬"]
heroes1.pop("이순신")
heroes1.clear()

# 접근
print("heroes2.items(): {0}, type(heroes2.items()): {1}".format(heroes2.items(), type(heroes2.items())))
print("heroes2.keys(): {0}, type(heroes2.keys()): {1}".format(heroes2.keys(), type(heroes2.keys())))
print("heroes2.values(): {0}, type(heroes2.values()): {1}".format(heroes2.values(), type(heroes2.values())))

for key in heroes2:
    print("heroes2[{0}]: {1}".format(key, heroes2[key]))

# 내포
heroes = {item for item in heroes2.items()}
print(f"heroes = {type(heroes)}")
heroes = {item[0]: item[1] for item in heroes2.items()}
print(heroes)


# 문자열
# 찾기
data_str = "abcdefghidefabc"
print(data_str.find("abc"))
print(data_str.rfind("abc"))
# 삽입
comma_space = ", "
output = comma_space.join(data_str)
print("{0}: {1}".format(type(output), output))
# 대소문자 변경
data_str = "better tomorrow"
data_str_cap = data_str.capitalize()
print("'{0}'".format(data_str_cap))
data_str_low = data_str.lower()
print("'{0}'".format(data_str_low))
data_str_upp = data_str.upper()
print("'{0}'".format(data_str_upp))
# 변경
hello = "Hello, Python!"
hell0 = hello.replace(" ", "")  # 공백 제거