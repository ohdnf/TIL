# 정수와 실수



## 정수 자료형

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



## Overflow of Integer

### `sizeof()`로 데이터 크기 확인

```c
#include <stdio.h>

int main()
{
    printf("%u bytes\n", sizeof(unsigned int));		// 4 bytes --> 32 bits
    printf("%u bytes\n", sizeof(signed int));		// 4 bytes

    return 0;
}
```



### signed vs. unsigned

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



### Overflow 확인

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



## 부동소수점 수

>  실수 표현에는 unsigned가 없다.

```
부호		지수		   분수
sign	exponent	fraction
+		  -1		0.3141592	=> 0.3141592 * 10^-1
+		   1		0.3141592	=> 0.3141592 * 10^1
+		   2		0.3141592	=> 0.3141592 * 10^2
```



### 32bit single precision

```c
float a = 1.234f;
```

| sign  | exponent | fraction |
| ----- | -------- | -------- |
| 1 bit | 8 bits   | 23 bits  |



### 64bit double precision

```c
double pi = 3.141592;
```

| sign  | exponent | fraction |
| ----- | -------- | -------- |
| 1 bit | 11 bits  | 52 bits  |


