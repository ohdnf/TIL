## 입력

### `scanf()` 함수

> Microsoft Visual Studio 쓸 때
> `#define _CRT_SECURE_NO_WARNINGS`를 전처리기 설정에 추가하면 에러 없어짐

```c
#include <stdio.h>

int main()
{
    int i = 0;
    scanf("%d", &i);	// ampersand
    
    printf("Value is %d", i);
    
    return 0;
}
```

#### `&`

- 변수 앞에 붙어서 변수의 주소를 넘겨주는 역할
- `scanf()` 함수의 인자(argument)로 `&i`라는 `i` 변수의 주소값을 넘겨준다.



## 출력

### `printf()` 함수

> standard input/output 라이브러리를 include 해야 사용할 수 있다.

```c
#include <stdio.h>

int main()
{
    printf("The truth is... \nI am Ironman.");
    // backslash == escape sequence
    return 0;
}
```

#### 변수를 넣어 출력하기

```c
#include <stdio.h>

int main()
{
    int x = 1, y = 4, z;
    
    z = x + y;
    
    printf("%i + %i = %d", x, y, z);

    return 0;
}
```

### 출력형식 지정자

- `%d` decimal 정수
- `%c` character 문자
- `%f` float 실수
- `%s` string 문자열(null 문자를 만날 때까지 출력)
- `%x` hexadecimal 16진수
- `%#x` 출력 문자열 앞에 `0x`를 붙여준다
- `%#X` `0X`를 붙여준다.
- `%%` % 문자 자체 출력(`\%`와 동일)



## 입출력 함수 만들어보기

> 원(KRW)을 입력받아 달러(USD)로 바꾸기

```c
#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main()
{
    float won = 0.0f;
    float dollar = 0.0f;
    
    printf("Input KRW: ");
    scanf("%f", &won);
    
    dollar = won * 0.00092f;	// 0.00092는 원래 double이므로 float형으로 변환
    printf("KRW %f = USD %f", won, dollar);
    
    return 0;
}
```

