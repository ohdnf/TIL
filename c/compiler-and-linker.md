# 컴파일러와 링커

```c
#include <stdio.h>

int main()
{
    // printf('Hello, world!');
    // 작은 따옴표는 문자열을 담지 못한다.
    printf("Hello, world!");
    return 0;
}
```

> Visual Studio에서 `Ctrl + F5`를 누르면 디버깅 없이 실행

### 폴더 구조

```
Solution/
	Debug/
		.exe
	Project/
		Debug/
			.obj
		.c
```

CMD에서 컴파일하고 실행하기

```shell
# -o 이름 옵션
$ gcc compile_this.c -o to_this_name.o
# -c 컴파일만 하기(obj 파일을 볼 수 있음)
$ gcc only_compile.c -c
$ gcc link_this_object.o -o exe_from_obj.exe
```
