# 자료형



## 목차
- [sizeof](#sizeof)
- [정수와 실수](#정수와-실수)
	- [정수 자료형](#정수-자료형)
	- [Overflow of Integer](#overflow-of-integer)
	- [고정 너비 정수](#고정-너비-정수)
	- [부동소수점 수](#부동소수점-수)
	- [부동소수점 형](#부동소수점-형)
- [문자형](#문자형)
- [불리언형](#불리언형)
- [복소수형](#복소수형)



## sizeof

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>

struct MyStruct
{
	int i;
	float f;
};

int main()
{
	/* 1. sizeof basic types */

	int a = 0;
	unsigned int int_size1 = sizeof a;
	unsigned int int_size2 = sizeof(int);
	unsigned int int_size3 = sizeof(a);

	// 이식하기 좋은 데이터형(portable type)
	size_t int_size4 = sizeof(a);
	size_t float_size = sizeof(float);

	printf("Size of int type is %u bytes.\n", int_size1);
	printf("Size of int type is %zu bytes.\n", int_size4);
	printf("Size of float type is %zu bytes.\n", float_size);

	/* 2. sizeof arrays */

	int int_arr[30];	// int_arr[0] = 1024;
	int* int_ptr = NULL;
	int_ptr = (int*)malloc(sizeof(int) * 30);	// int_ptr[0] = 1024;

	printf("Size of array = %zu bytes\n", sizeof(int_arr));
	printf("Size of pointer = %zu bytes\n", sizeof(int_ptr));

	/* 3. sizeof character array */

	char c = 'a';
	char string[10];	// maximum 9 character + '\0' (null character)

	size_t char_size = sizeof(char);
	size_t str_size = sizeof(string);

	printf("Size of char type is %zu bytes.\n", char_size);
	printf("Size of string type is %zu bytes.\n", str_size);

	/* 4. sizeof structure */

	printf("%zu\n", sizeof(struct MyStruct));

	return 0;
}
```






## 정수와 실수

### 정수 자료형

> unsigned
> 부호 없는 양수만을 표현
>
> signed
> 부호 있는 음수까지 표현, 맨 앞 한 비트를 부호 표현에 사용(0이면 양수 1이면 음수), 음수는 2의 보수로 표현

| 정수 자료형                         | 최소 크기(Byte) | 값의 범위                                              | 형식 지정자(`%`) |
| ----------------------------------- | --------------- | ------------------------------------------------------ | ---------------- |
| (signed) char                       | 1               | -128 ~ 127                                             | hhd 또는 c       |
| unsigned char                       | 1               | 0 ~ 255                                                | hhu 또는 c       |
| (signed) short (int)                | 2               | -32,768 ~ 32,767                                       | hd               |
| unsigned short int                  | 2               | 0 ~ 65,535                                             | hu               |
| signed (int)<br />또는 (signed) int | 2 또는 4        | -32,768 ~ 32,767<br />-2,147,483,648 ~ 2,147,483,647   | d 또는 i         |
| unsigned (int)                      | 2 또는 4        | 0 ~ 65,535<br />0 ~ 4,294,967,295                      | u                |
| long (int)                          | 4               | -2,147,483,648 ~ 2,147,483,647                         | ld               |
| unsigned long (int)                 | 4               | 0 ~ 4,294,967,295                                      | lu               |
| long long (int)                     | 8               | -9,223,372,036,854,775,808 ~ 9,223,372,036,854,775,807 | lld              |
| unsigned long long (int)            | 8               | 0 ~ 18,446,744,073,709,551,615                         | llu              |

- 괄호 안은 생략 가능
- 현대 컴파일러의 `int`는 보통 4 byte
- 형식 지정자(Format specifier)를 잘 못 사용하면 overflow가 발생할 수 있다.

```c
#include <stdio.h>

int main()
{
    char c = 65;
    short s = 200;
    unsigned int ui = 3000000000U;
    long l = 65537L;
    long long ll = 12345678901234LL;

    printf("char = %hhd, %d, %c\n", c, c, c);
    printf("short = %hhd, %hd, %d\n", s, s, s);
    printf("unsigned int = %u, %d\n", ui, ui);
    printf("long = %ld, %hd\n", l, l);
    printf("long long = %lld, %ld\n", ll, ll);

    return 0;
}
```

결과

```shell
char = 65, 65, A
short = -56, 200, 200
unsigned int = 3000000000, -1294967296
long = 65537, 1
long long = 12345678901234, 1942892530
```



### Overflow of Integer

#### `sizeof()`로 데이터 크기 확인

```c
#include <stdio.h>

int main()
{
    printf("%u bytes\n", sizeof(unsigned int));		// 4 bytes --> 32 bits
    printf("%u bytes\n", sizeof(signed int));		// 4 bytes

    return 0;
}
```



#### signed vs. unsigned

```c
#include <stdio.h>
#include <limits.h>

int main()
{
    // unsigned int i = 0b11111111111111111111111111111111;
    unsigned int u_max = UINT_MAX;
    unsigned int u_min = 0;
    signed int i_max = INT_MAX;
    signed int i_min = INT_MIN;

    printf("max of unsigned = %u\n", u_max);
    printf("min of unsigned = %u\n", u_min);
    printf("max of signed = %u\n", i_max);
    printf("min of signed = %u\n", i_min);

    return 0;
}
```

결과

```shell
max of unsigned = 4294967295
min of unsigned = 0
max of signed = 2147483647
min of signed = 2147483648
```



#### Overflow 확인

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <limits.h>
#include <stdlib.h>

int main()
{
    // unsigned int u_max = UINT_MAX;
    unsigned int u_max = UINT_MAX + 1;
    // unsigned int u_max = 0 - 1;

    // i to binary representation
    char buffer[33];
    _itoa(u_max, buffer, 2);

    // print decimal and binary
    printf("decimal: %u\n", u_max);
    printf("binary: %s\n", buffer);

    return 0;
}
```

1. `unsigned int u_max = UINT_MAX`

   ```shell
   decimal: 4294967295
   binary: 11111111111111111111111111111111
   ```

2. `unsigned int u_max = UINT_MAX + 1`

   ```shell
   decimal: 0
   binary: 0
   ```

3. `unsigned int u_max = 0 - 1`

   ```shell
   decimal: 4294967295
   binary: 11111111111111111111111111111111
   ```



### 고정 너비 정수

> C 언어는 각 자료형의 최소 메모리 크기를 규정하고, 시스템에 따라 크기가 더 커질 수 있다.
> Why? 해당 컴퓨터 아키텍처에서 최선이라고 생각하는 int의 크기가 다르기 때문
> int = 4 byte -> 다른 시스템에서 2 byte라면 overflow -> 이식성이 낮아지는 문제 발생
>
> 멀티/크로스 플랫폼 관련 실무를 할 때 모든 아키텍처에서 자료형의 메모리 크기를 고정시킬 필요가 있기 때문에 고정 너비 정수를 정의한다.(C99 이후)

```c
#include <stdio.h>
//#include <stdint.h>	// also included in inttypes.h
#include <inttypes.h>

int main()
{
    int i;
    int32_t i32;		// 32 bit integer
    int_least8_t i8;	// smallest 8 bit
    int_fast8_t f8;		// fastest minimum
    intmax_t imax;		// biggest signed integers
    uintmax_t uimax;	// biggest unsigned integers
    
    i32 = 1004;
    printf("me32 = %d\n", i32);
    printf("me32 = %" "d" "\n", i32);		// formatting 문자열 띄어쓰기
    printf("me32 = %" PRId32 "\n", i32);	// 매크로로 교체
}
```



### 부동소수점 수

>  실수 표현에는 unsigned가 없다.

```
부호		지수		   분수
sign	exponent	fraction
+		  -1		0.3141592	=> 0.3141592 * 10^-1
+		   1		0.3141592	=> 0.3141592 * 10^1
+		   2		0.3141592	=> 0.3141592 * 10^2
```



#### 32bit single precision

```c
float a = 1.234f;
```

| sign  | exponent | fraction |
| ----- | -------- | -------- |
| 1 bit | 8 bits   | 23 bits  |



#### 64bit double precision

```c
double pi = 3.141592;
```

| sign  | exponent | fraction |
| ----- | -------- | -------- |
| 1 bit | 11 bits  | 52 bits  |



### 부동소수점 형

> 천문학적인 수를 다루기 위한 과학적 표기법

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>

int main()
{
	printf("%u\n", sizeof(float));			// 4
	printf("%u\n", sizeof(double));			// 8
	printf("%u\n", sizeof(long double));	// 8

	float f = 123.456f;
	double d = 123.456;

	float f2 = 123.456;		// truncation 발생 가능
	double d2 = 123.456f;	// double이 더 크기 때문에 경고 없음

	int i = 3;
	float f3 = 3.f;			// 3.0f 등 실수형을 나타내주는 것이 좋다
	double d3 = 3.;			// 3.0

	float f4 = 1.234e10f;

	float f5 = 0x1.1p1;		// e 대신에 p 사용
	double d5 = 1.0625e0;

	printf("%f %F %e %E\n", f, f, f, f);	// 123.456001 123.456001 1.234560e+02 1.234560E+02
	printf("%f %F %e %E\n", d, d, d, d);	// 123.456000 123.456000 1.234560e+02 1.234560E+02
	printf("%a %A\n", f5, f5);		// 0x1.1000000000000p+1 0X1.1000000000000P+1
	printf("%a %A\n", d5, d5);		// 0x1.1000000000000p+0 0X1.1000000000000P+0

	return 0;
}
```



#### 한계

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <float.h>

int main()
{
	// round-off error 1
	float a, b;
	a = 1.0e20f + 1.0f;
	b = a - 1.0e20f;

	printf("%f\n", b);	// 0.0000	범위가 크게 다른 두 수를 연산하면 1이 무시됨

	// round-off error 2
	float a = 0.0f;
	a = a + 0.01f;	// 백 번 반복하면 1이 나와야 한다.
	// a = a + 0.01f; 했다 치고
	printf("%f\n", a);	// 0.999999가 나온다.
	/*
	* 부동소수점은 0.01을 정확히 표현하기 어렵기 때문에 이런 문제가 발생
	*/
    
    // overflow
	float max = 3.402823466e+38F
	printf("%f\n", max);	// 340282346638528859811704183484516925440.000000
	max = max * 100.0f;
	printf("%f\n", max);	// inf

	// underflow
	float f = 1.401298464e-45F;
	printf("%e\n", f);			// 1.401298e-45
	f = f / 2.0f;				// subnormal 발생
	printf("%e\n", f);			// 0.000000e+00

	return 0;
}
```



## 문자형

ASCII

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
	char c = 'A';
	char d = '*';

	// printf("%c %hhd\n", c, c);	// A 65
	// printf("%c %hhd\n", d, d);	// * 42

	// printf("%c \n", c + 1);		// B

	// char a = '\a';
	// printf("%c", a);		// 띵동
	// printf("\07");
	// printf("\x7");

	float salary;
	printf("$______\b\b\b\b\b\b");
	scanf("%f", &salary);
}
```



### 문자열이 메모리에 저장되는 구조

```
숫자 하나	1
숫자의 배열	0 1 2 3 4 5 6 7 8 9
문자 하나	'a'
문자의 배열	'H' 'e' 'l' 'l' 'o' '\0' ? ? ? ?
```

> `printf()` 함수가 문자열 출력시 `\0`을 만나면 그 다음은 무시하고 출력을 종료하게 된다.

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
	int a = 1;
	int int_arr[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };

	printf("%i %i %i\n", int_arr[0], int_arr[1], int_arr[9]);
	// printf("%i\n", int_arr[10]);

	char c = 'a';
	char str1[10] = "Hello";
	char str2[10] = { 'H', 'i' };

	printf("%c\n", c);
	printf("%s\n", str1);
	printf("%s\n", str2);

	printf("%hhi %hhi %hhi %hhi %hhi \n",
		str2[0], str2[1], str2[2], str2[3], str2[4]);

	//char str3[10] = "Hello, World";		// array size is not enough
	char str3[20] = "Hello, \0World";
	printf("%s\n", str3);		// Hello, 
    printf("%c\n", str3[10]);	// r
    printf("%c\n", str3[11]);	// l

	return 0;
}
```



### `strlen()`

> 문자열의 길이를 반환하는  함수
> `\0` 등을 제외하고 사람에게 의미 있는 문자열 길이만을 측정한다.

```c
#include <stdio.h>
#include <string.h>

int main()
{
    char str1[100] = "Hello";
    char str2[] = "Hello";		// 6 글자에 맞춰서 공간 할당
    char str3[100] = "\0";
    char str4[100] = "\n";

    printf("%zu %zu\n", sizeof(str1), strlen(str1));    // 100 5
    printf("%zu %zu\n", sizeof(str2), strlen(str2));    // 6 5
    printf("%zu %zu\n", sizeof(str3), strlen(str3));    // 100 0
    printf("%zu %zu\n", sizeof(str4), strlen(str4));    // 100 1

    /* Extra */
    char* str5 = (char*)malloc(sizeof(char) * 100);
    str5[0] = 'H'; str5[1] = 'e'; str5[2] = 'l'; str5[3] = 'l'; str5[4] = 'o';
    str5[5] = '\0';
    printf("%zu %zu\n", sizeof(str5), strlen(str5));    // 4 5

    return 0;
}
```





## 불리언형

> 고전적인 C 언어 자료형에는 불리언 형이 없었다.
> `_Bool`이 추가되었다가 최근에 `stdbool.h` 헤더가 추가되어 `bool`로 쓸 수 있게 되었다.
> 특별히 출력 형식지정자가 없다.

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdbool.h>

int main()
{
	printf("%u\n", sizeof(_Bool));	// 1 byte

	_Bool b1;
	b1 = 0;		// false
	b1 = 1;		// true

	printf("%d\n", b1);

	bool b2, b3;
	b2 = true;
	b3 = false;

	printf("%d %d\n", b2, b3);

	return 0;
}
```



## 복소수형

> Visual Studio와 GCC 컴파일러가 각각 복소수를 지원하는 방식이 다르다.

### VS

```c
#include <stdio.h>
#include <complex.h>

int main()
{
    _Dcomplex z;
    z._Val[0] = 1.0;
    z._Val[1] = 1.0;
    
    return 0;
}
```



### GCC

```c
#include <stdio.h>
#include <complex.h>

int main()
{
    //double _Imaginary i = 3;
    double _Complex z = 1 + 2*I;
    
    z = 1 / z;
    
    printf("1 / (1.0 + 2.0i) = %.1f%+.1fi\n", creal(z), cimag(z));
    
    return 0;
}
```



