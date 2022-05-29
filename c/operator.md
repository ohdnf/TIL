# 연산자

> 연산자 operator
> 피연산자 operand

```c
int i;
i = 1024;
// 1024 = I;
i = i + 1;
```

> **L-value(object locator value)**
> 메모리를 차지하고 있는 특정 데이터 객체
>
> **R-value(value of an expression)**
> 수정 가능한 L-value에게 대입될 수는 있지만 자기 자신은 L-value가 될 수 없는 것들
>
> `a`, `b`, `c`는 수정 가능한 L-value
> `TWO`는 수정 불가능한 L-value(여기서 `=`는 대입이 아니라 초기화)
>
> `42`는 R-value
> `(a + b)`는 R-value(프로그램이 계산하는 임시 값, 끝나면 사라짐)

```c
const int TWO = 2;
int a;
int b;
int c;

a = 42;
b = a;
c = TWO * (a + b);
```

## 더하기/빼기 연산자

```c
#include <stdio.h>

int main()
{
	printf("%d\n", 1 + 2);

    int income, salary, bonus;

    income = salary = bonus = 100;	// triple assignment

    salary = 100;
    bonus = 30;

    income = salary + bonus;	// l-value vs r-value
    // salary(l-value) + bonus(l-value) 의 결과값은 임시로 저장되는 r-value

    int takehome, tax;
    tax = 20;
    takehome = income - tax;

    int a, b;
    a = -7;
    b = -a;
    b = +a;

    1.0f + 2;

	return 0;
}
```

## 곱하기 연산자

> 원금과 목표금액, 이자율이 주어졌을 때 원금을 복리로 저축하여 목표 금액이 되는 데까지 걸리는 시간 계산

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
    double seed_money, target_money, annual_interest;

    printf("Input seed money: ");
    scanf("%lf", &seed_money);

    printf("Input target money: ");
    scanf("%lf", &target_money);

    printf("Input annual interest(%%): ");
    scanf("%lf", &annual_interest);

    double fund = seed_money;
    int year_count = 0;

    while (fund < target_money)
    {
        // fund = fund * (1 + 0.01 * annual_interest);
        fund += fund * annual_interest / 100;
        // year_count = year_count + 1;
        // year_count += 1;
        year_count++;
    }

    printf("It takes %d years\n", year_count);

    return 0;
}
```

## 나누기 연산자

```c
#include <stdio.h>

int main()
{
	// Truncating towrad zero (C99)
    print("-7 / 2 = %d\n", -7 / 2);
    // -3.5가 아닌 -3 출력

    // Floating divisions
    printf("9.0 / 4 = %f\n", 9.0 / 4);
    // 다른 형끼리 연산을 못하지만
    // 컴파일러가 형변환을 하여 계산
	return 0;
}
```

## 나머지 연산자

> 초를 입력 받으면 시간, 분, 초로 계산

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
    int seconds = 0, minutes = 0, hours = 0;

    printf("Input seconds: ");
    scanf("%d", &seconds);

    while (seconds >= 0)
    {
        hours = seconds / 3600;
        seconds %= 3600;
        minutes = seconds / 60;
        seconds %= 60;

        printf("%d hours, %d minutes, %d seconds\n", hours, minutes, seconds);

        printf("Input seconds: ");
        scanf("%d", &seconds);
    }

    return 0;
}
```

> 피연산자가 음수이면 나머지도 음수

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
    int div, mod;

    div = 11 / 5;	// 2
    mod = 11 % 5;	// 1
    printf("div = %d, mod = %d\n", div, mod);

    div = 11 / -5;	// -2
    mod = 11 % -5;	// 1
    printf("div = %d, mod = %d\n", div, mod);

    div = -11 / 5;	// 2
    mod = -11 % 5;	// -1
    printf("div = %d, mod = %d\n", div, mod);

    div = -11 / -5;	// -2
    mod = -11 % -5;	// -1
    printf("div = %d, mod = %d\n", div, mod);


    return 0;
}
```

## 증가/감소 연산자

> L-value에 대해서만 가능

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
    /*int a = 0;
    a++;
    printf("%d\n", a);

    ++a;
    printf("%d\n", a);*/

    //int count = 0;
    //while (count++ < 10)	// ++count or count++
    //{
    //    printf("%d ", count);
    //}

    int i = 1, j = 1;
    int i_post, pre_j;

    i_post = i++;   // i_post에 i값(=1)을 대입시키고 i값을 1 증가(=2)시킨다.
    pre_j = ++j;

    printf("%d %d\n", i, j);
    printf("%d %d\n", i_post, pre_j);

    return 0;
}
```

## 표현식(Expression)과 문장(Statement)

### 표현식

| 표현식                 | 값                          |
| ---------------------- | --------------------------- |
| 4                      | 4                           |
| -6                     | -6                          |
| 4 + 21                 | 25                          |
| q = 5 \* 2             | 10                          |
| 3 + (c = 1 + 2)        | 6                           |
| 2 > 1                  | 1                           |
| 2 < 1                  | 0                           |
| a                      | a의 값                      |
| x = ++q % 3            | x와 q의 값에 의해 결정      |
| q > 3                  | q의 값에 의해 결정          |
| a \* \*b / c + d) / 20 | a, b, c, d의 값에 의해 결정 |

### 표현식

```c
int x, y, apples;	// declaration statement
apples = 3;			// assignment statement
;					// null statement
7;
1 + 2;
x = 4;
++x;
x = 1 + (y = 5);	// y = 5 is subexpression

while (x++ < 10)	// while statement(structured statement)
    y = x + y;

int i = 0;
while (i < 10)
    // {} block is a compound statement
{
    i++;
    printf("%d\n", i)
}

printf("%d\n", y);	// function statement
return 0;			// return statement
```

### Side Effects and Sequence Points

> Sequence point는 모든 식의 계산(평가)이 완료되는 시점을 말한다. 앞의 문장이 계산되는 구분점이다. `;`이 있는 경우 대부분 Sequence point다. `;`이 없어도 `while`문처럼 조건이 계산되면 Sequence point를 만난다고 볼 수 있다.

```c
x = 4;					// main intent is evaluating expressions
y = 1 + x++;

while (x++ < 10)		// ;이 없지만 sequence point를 만난다고 가정
    printf("%d\n", x);	// (x++ < 10) is a full expression

y = (4 + x++) + (6 + x++);	// Not (4 + x++) nor (6 + x++) is a full expression
```

## 자료형 변환

```c
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>

int main()
{
    /* promotion in assignment */
    short s = 64;
    int i = s;

    float f = 3.14f;
    double d = f;

    /* demotion in assignment */
    d = 1.25;
    f = 1.25;
    f = 1.123;

    /* ranking of types in operations */
    // Ref: Integer conversion rank

    /* automatic promotion of function arguments */
    // 1. Functions without prototypes
    // 2. Variadic functions

    /* casting operators */
    // 명시적 형변환을 해주는 것이 바람직하다.
    d = (double)3.14f;
    i = 1.6 + 1.7;				// i = 3
    i = (int)1.6 + (int)1.7;	// i = 2

    char c;
    //int i;
    //float f;
    f = i = c = 'A';	// 65
    printf("%c %d %f\n", c, i, f);
    c = c + 2;	//	'C', 67
    i = f + 2 * c;	// 65.0f + 2 * 67
    printf("%c %d %f\n", c, i, f);	// 199
    c = 1106;
    // demolition
    // 1106 = 0b10001010010, 0b01010010 = 1106 % 256 = 82 = 'R'
    printf("%c\n", c);	// 82, 'R'
    c = 83.99;
    printf("%c\n", c);	// 83, 'S'

    return 0;
}
```
