# 입출력



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

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <inttypes.h>

int main()
{
    /* multiple inputs with blank separators */
    int i;
    float f;
    char str[30];
    scanf("%d %f %s", &i, &f, str);
    printf("%d %f %s\n", i, f, str);

    /* character */
    char c;
    scanf("%c", &c);
    printf("%i\n", c);

    /* unsigned as signed */
    unsigned i;
    scanf("%i", &i);    // signed인 i로 받기 때문에 음수도 가능
    printf("%i\n", i);

    /* unsigned as unsigned */
    unsigned i;
    scanf("%u", &i);
    printf("%u\n", i);

    /* floating point numbers */
    // l for double for %f %e %g ...
    double d = 0.0;
    scanf("%lf", &d);
    printf("%f\n", d);

    /* width */
    char str[30];
    scanf("%5s", str);  // 문자열 다섯 글자까지 입력받음
    printf("%s\n", str);

    /* h modifier */
    char i;
    scanf("%hhd", &i);
    printf("%i\n", i);

    /* integer with characters */
    int i;
    scanf("%i", &i);    // 123a 123a456
    printf("%i\n", i);

    /* j modifier */
    intmax_t i;
    scanf("%ji", &i);
    printf("%ji", i);

    /* regular characters */
    int a, b;
    scanf("%d,%d", &a, &b);
    printf("%d %d\n", a, b);

    /* char receives blank */
    int a, b;
    char c;
    scanf("%d%c%d", &a, &c, &b);
    printf("%d|%c|%d", a, c, b);

    /* return value of scanf() */
    int a, b;
    int i = scanf("%d%d", &a, &b);
    printf("%d", i);

    /* *modifier for printf() */
    int i = 123;
    int width = 5;
    printf("Input width: ");
    scanf("%d", &width);
    printf("%*d\n", width, i);

    /* *modifier for scanf() */
    int i;
    scanf("%*d%*d%d", &i);  // 입력은 세 개 받지만 세번째 입력을 i에 대입
    printf("Your third input = %d", i);

    return 0;
}
```





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

### 변수를 넣어 출력하기

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

### 자주 사용하는 변환 지정자

- `%d` decimal 정수
- `%c` character 문자 하나
  - 문자 하나는 single quote로 감싸줘야 한다(`'A'` (O) / `"A"` (X))
- `%f` float 실수
- `%s` string 문자열(null 문자를 만날 때까지 출력)
- `%x` hexadecimal 16진수
- `%#x` 출력 문자열 앞에 `0x`를 붙여준다
- `%#X` `0X`를 붙여준다.
- `%%` % 문자 자체 출력(`\%`와 동일)

### 변환 지정자

> Conversion specifier. 옵션 수식어와 함께 붙어 형식 지정자가 된다.

```c
printf(제어-문자열, 아이템1, 아이템2, ...);
아이템은 변수, 상수, 표현식 등이 될 수 있다.

int a = 2;
printf("%d + %d = %d", 1, a, 1 + a);
```

#### 형식 지정자의 수식어

> 변환 지정자 앞에 붙는 출력 옵션
>
> [참고](http://www.cplusplus.com/reference/cstdio/printf/)

```c
// %[flags][width][.precision][length]specifier
    
printf("%+10.5hi", 256);
printf("%010.3f", 123.45678);
```

> 참고
> `printf()`에서는 부동소수점 데이터가 들어오면 `float`, `double` 상관 없이 모두 `double`로 변환되어, 8 bytes가 된다.

```c
#include <stdio.h>

int main()
{
    float n1 = 3.14;	// 4 bytes
    double n2 = 1.234;	// 8 bytes
    int n3 = 1024;		// 4 bytes
    
    printf("%f %f %d\n\n", n1, n2, n3);
    
    // 데이터 사이즈가 맞지 않거나, 데이터 사이즈는 맞지만 타입이 맞지 않는 예
    printf("%d %d %d\n\n", n1, n2, n3);
    printf("%lld %lld %d\n\n", n1, n2, n3);
    printf("%f %d %d\n\n", n1, n2, n3);
    printf("%f %lld %d\n\n", n1, n2, n3);
    
    return 0;
}
```



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



> 문자열 입출력하기

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
	//char fruit_name;	// stores only one character
	char fruit_name[40];	// stores only one character
	printf("What is your favorite fruit?\n");

	//scanf("%c", &fruit_name);
	scanf("%s", fruit_name);	// 배열의 이름은 주소를 나타내기 때문에 &를 붙이지 않는다.

	//printf("You like %c!\n", fruit_name);
	printf("You like %s!\n", fruit_name);

	return 0;
}
```

