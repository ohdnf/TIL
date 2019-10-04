# path

### 디렉토리에 있는 파일들 이름 불러오기

1. join 사용
해당 디렉토리의 파일명만 저장
```
import os

path = "C:/Users/user/Documents/Study/TIL/Python"
filelist = os.listdir()
filelist_py = [file for file in filelist if file.endswith(".py")]
for file in filelist:
    print(file)
for file in filelist_py:
    print(file)
```


2. glob 사용
디렉토리 경로까지 붙여서 전부 저장
```
import glob

path = "C:/Users/user/Documents/Study/TIL/Python"
filelist = glob.glob(path)
for file in filelist:
    print(file)
```



# join
디렉토리와 파일 이름을 연결해주는 함수
```
import os

def search(dirname):
    filenames = os.listdir(dirname)
    for filename in filenames:
        full_filename = os.path.join(dirname, filename)
        print (full_filename)

search("c:/")
```



# walk
시작 디렉토리부터 하위 모든 디렉토리를 차례대로 방문
```
import os

for (path, dir, files) in os.walk("c:/"):
    for filename in files:
        ext = os.path.splitext(filename)[-1]
        if ext == '.py':
            print("%s/%s" % (path, filename))
```