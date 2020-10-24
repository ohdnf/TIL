# 기초 문법



## 목차

- [기본 자료형](#기본-자료형)
- [형 변환](#형-변환)
- [Formatting](#Formatting)
  - [특수 문자](#특수-문자)
  - [서식 문자](#서식-문자)
- [연산자](#연산자)
- 



## 기본 자료형

### 기본 자료형

데이터가 변수에 직접 저장

```java
int i = 10;
```

#### 정수형

- `byte` 1 byte
- `char` 2 byte
- `short` 2 byte
- `int` 4 byte
- `long` 8 byte

#### 실수형

- `float` 4 byte
- `double` 8 byte

#### 논리형

- `boolean` 1 byte



### 객체 자료형

객체 메모리 주소가 변수에 저장

- C 계열에선 **포인터**
- Java에선 **레퍼런스**

```java
String str = "ABC";

```



## 형 변환

### 묵시적 형 변환

작은 공간의 메모리에서 큰 공간의 메모리로 이동

```java
byte by = 10;
int in = by;
System.out.println("in = " + in);
// in = 10
```



### 명시적 형 변환

큰 공간의 메모리에서 작은 공간의 메모리로 이동

```java
int iVar = 100;
byte bVar = (byte)iVar;
System.out.println("bVar = " + bVar);
// bVar = 100

iVar = 123456;
bVar = (byte)iVar;
System.out.println("bVar = " + bVar);
// bVar = 64
```

> 명시적 형 변환은 데이터가 누실될 수 있다.
>
> 따라서 기본적으로 여유롭게 큰 공간의 메모리를 가진 자료형을 선택한다.



## Formatting



### 특수 문자

| 문자    | 기능         |
| ------- | ------------ |
| `\t`    | 탭           |
| `\n`    | 줄 바꿈      |
| `\'`    | 작은 따옴표  |
| `\"`    | 큰 따옴표    |
| `\\`    | 역슬래시     |
| `//`    | 한 줄 주석   |
| `/* */` | 여러 줄 주석 |



### 서식 문자

> `System.out.prinf("서식문자", 자료);`
>
> println과 달리 printf를 사용한다.



#### 서식

| 문자 | 형식   |
| ---- | ------ |
| `%d` | 10진수 |
| `%o` | 8진수  |
| `%x` | 16진수 |
| `%c` | 문자   |
| `%s` | 문자열 |
| `%f` | 실수   |

예시

```java
float todayTemp = 12.34;
System.out.printf("오늘의 기온은 %f도입니다.", todayTemp);

// 진수 바꾸기
int num = 30;
System.out.printf("10진수로 표현: %d", num);	// 30
System.out.printf("8진수로 표현: %o", num);		// 36
System.out.printf("16진수로 표현: %x", num);	// 1E
```



#### `printf` 출력 형식

```java
System.out.printf("%d", 123);		// 123
System.out.printf("%d", 1234);		// 1234
System.out.printf("%d", 12345);		// 12345

// 출력 문자열 길이를 5로 두고 오른쪽으로 정렬해 출력
System.out.printf("%5d", 123);		//   123
System.out.printf("%5d", 1234);		//  1234
System.out.printf("%5d", 12345);	// 12345

// 소수점 자리 정하기
System.out.printf("%f", 1.23);		// 1.230000
System.out.printf("%.0f", 1.23);	// 1
System.out.printf("%.1f", 1.23);	// 1.2
System.out.printf("%.2f", 1.23);	// 1.23
System.out.printf("%.3f", 1.23);	// 1.234

// 소수점 밑이 잘린다면 반올림
System.out.printf("%.1f", 1.23);	// 1.2
System.out.printf("%.1f", 1.25);	// 1.3
```



## 연산자



### 삼항 연산자

```java
// condition ? true : false

int a = 10;
int b = 20;
(a > b) ? 100 : 200		// 200
```



### 증감 연산자

전위 연산자와 후위 연산자의 차이를 구분하여 알도록 하자.

```java
int x = 10;
System.out.println("Current x's value is " + (x));	// 10
System.out.println("print(++x) = " + (++x));		// 11
System.out.println("after ++x = " + (x));			// 11

x = 10;
System.out.println("Reset int x as " + (x));		// 10
System.out.println("print(x++) = " + (x++));		// 10
System.out.println("after x++ = " + (x));			// 11
```



### 논리 연산자

#### AND

```java
boolean cond1 = false;
boolean cond2 = true;

cond1 && cond2	// false
```

#### OR

```java
boolean cond1 = false;
boolean cond2 = true;

cond1 || cond2	// true
```

#### NOT

```java
boolean cond = false;

!cond	// true
```



### 비트 연산자

데이터를 비트 단위로 환산하여 연산을 수행, 다른 연산자보다 연산 속도가 향상

```java
int x = 2;	// 0010
int y = 3;	// 0011

x & y	// 0010	= 2
x | y	// 0011	= 3
x ^ y	// 0001	= 1
```

