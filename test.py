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


class Student:
    def __init__(self):
        self.__name = ''

    @property
    def name(self):
        return self.__name

    @name.setter
    def name(self, name):
        self.__name = name

class GraduateStudent(Student):
    def __init__(self):
        super().__init__()
        self.__major = ''

    @property
    def major(self):
        return self.__major
    
    @major.setter
    def major(self, major):
        self.__major = major


if __name__ == '__main__':
    student = Student()
    student.name = '홍길동'
    graduateStudent = GraduateStudent()
    graduateStudent.name = '이순신'
    graduateStudent.major = '컴퓨터'
    print(student.name)
    print(graduateStudent.name)
    print(graduateStudent.major)