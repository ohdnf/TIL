# result = []
# temp = []
# for i in range(2, 10):
#     del temp[:]     # <<< ???
#     for j in range(1, 10):
#         temp.append(i * j)

#     result.append(temp)

# print(result)

# data = list(map(int, input().split(',')))
# print(data)
# print(tuple(data))

# products = {k: v for k, v in sorted(products.items(), key=lambda item: item[1], reverse=True)}
# for e in products:
#     print("{0}: {1}".format(e, products[e]))

# beer = {'하이트': 2000, '카스': 2100, '칭따오': 2500, '하이네켄': 4000, '버드와이저': 500}

# new_beer = {item[0]: round(item[1] * 1.05, 1) for item in beer.items()}

# for i, item in enumerate(beer):
#     print(i, item)

# for item in beer.items():
#     print(item)


# L=[]
# A=[L.append(1)  if x<2 else L.append(L[x-1]+L[x-2]) for x in range(0,10)]
# print(L)

# data = input()
# while data != '':
#     print(">> {0}".format(data.upper()))
#     data = input()

# words = input().split(" ")
# words = sorted(set(words))
# print(words.join(","))


# class Student:
#     def __init__(self):
#         self.__name = ''

#     @property
#     def name(self):
#         return self.__name

#     @name.setter
#     def name(self, name):
#         self.__name = name

# class GraduateStudent(Student):
#     def __init__(self):
#         super().__init__()
#         self.__major = ''

#     @property
#     def major(self):
#         return self.__major
    
#     @major.setter
#     def major(self, major):
#         self.__major = major


# if __name__ == '__main__':
#     student = Student()
#     student.name = '홍길동'
#     graduateStudent = GraduateStudent()
#     graduateStudent.name = '이순신'
#     graduateStudent.major = '컴퓨터'
#     print(student.name)
#     print(graduateStudent.name)
#     print(graduateStudent.major)

# T = int(input())
# # 여러개의 테스트 케이스가 주어지므로, 각각을 처리합니다.
# for test_case in range(1, T + 1):
#     N, M = map(int, input().split())
#     area = []
#     for _ in range(N):
#         area.append(list(map(int, input().split())))
#     max_kill = 0
#     for row in range(N-M-1):
#         for col in range(N-M-1):
#             kill = sum([sum(area[r][col:col+M]) for r in range(row, row+M)])
#             if kill > max_kill:
#                 max_kill = kill
#     print('#{0} {1}'.format(test_case, max_kill))

# def ssafy(name, location='서울'):
#     print(f'{name}의 지역은 {location}입니다.')

# if __name__ == '__main__':
#     # ssafy(location='대전', name='철수')
#     # ssafy('길동', location='광주')
#     ssafy(name='허준', '구미')