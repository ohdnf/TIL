# 기초 문법

## 목차

- [기본 자료형](#기본-자료형)
- [형 변환](#형-변환)
- [Formatting](#Formatting)
  - [특수 문자](#특수-문자)
  - [서식 문자](#서식-문자)
- [연산자](#연산자)
- [배열](#배열)
- [배열과 메모리](#배열과-메모리)
- [조건문](#조건문)
- [반복문](#반복문)

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

### 단항 연산자 & 이항 연산자

```java
// 단항 연산자
+x, -y, !z

// 이항 연산자
int x = 1;
int y = 2;
```

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

## 배열

### 배열이란

> **인덱스를 이용**해서 **자료형이 같은 데이터**를 관리하는 것

### 배열 선언 및 초기화

> 변수와 마찬가지로 선언과 초기화 과정을 거쳐 사용

#### 배열 선언 후 초기화

`데이터의 자료형[] 배열이름 = new 자료형[배열의 크기]`

```java
// 배열 선언 후 초기화
int[] arr1 = new int[5];
arr1[0] = 100;
arr1[1] = 200;
arr1[2] = 300;
arr1[3] = 400;
arr1[4] = 500;
```

#### 선언과 초기화를 동시에

```java
int[] arr2 = {10, 20, 30, 40, 50};
```

> Java에서 배열의 크기는 한 번 정해지면 바뀔 수 없다!

#### 사용자의 입력값으로 배열 초기화

```java
String[] team = new String[5];
Scanner scanner = new Scanner(System.in);

System.out.printf("첫 번째 팀원 이름: ");
team[0] = scanner.next();

System.out.printf("두 번째 팀원 이름: ");
team[1] = scanner.next();

System.out.printf("세 번째 팀원 이름: ");
team[2] = scanner.next();

System.out.printf("네 번째 팀원 이름: ");
team[3] = scanner.next();

System.out.printf("다섯 번째 팀원 이름: ");
team[4] = scanner.next();

scanner.close();
```

## 배열과 메모리

### 배열의 메모리 크기

> 배열을 구성하는 데이터의 자료형에 따라 배열의 메모리 크기가 결정

```java
int [] arr = new int[3];

// int형 = 4 byte
// arr의 크기 = 3 * 4 = 12 byte
```

### 배열 변수: 배열을 가리키는 배열이름

> 기본 자료형 변수와 달리 배열 변수는 배열의 주소를 담고 있다.

```java
// 배열 초기화
int[] arr1 = {10, 20, 30, 40, 50};
int[] arr2 = null;
int[] arr3 = null;

// 배열 길이
System.out.println("arr1.length: " + arr1.length);

// 배열 요소 출력
System.out.println("arr1: " + Arrays.toString(arr1));

// 배열 요소 복사
System.out.println("arr3: " + Arrays.toString(arr3));
arr3 = Arrays.copyOf(arr1, arr1.length);
System.out.println("arr3: " + Arrays.toString(arr3));

// 배열 참조(reference)
arr2 = arr1;
System.out.println("arr1: ": arr1);
System.out.println("arr2: ": arr2);
System.out.println("arr3: ": arr3);
```

### 다차원 배열

> 3차원 이상은 보통 사용하지 않는다.

```java
// 다차원 배열(예. 2차원 배열)
int[][] matrix = new int[3][2];	// 3행 2열
matrix[0][0] = 10;
matrix[0][1] = 20;
matrix[1][0] = 30;
matrix[1][1] = 40;
matrix[2][0] = 50;
matrix[2][1] = 60;

System.out.println("matrix[0]: " + Arrays.toString(matrix[0]));
System.out.println("matrix[1]: " + Arrays.toString(matrix[1]));
System.out.println("matrix[2]: " + Arrays.toString(matrix[2]));
```

## 조건문

### 조건문이란

### if문

> 양자택일

```java
// if (조건식)
if (condition) {
    // 조건이 참이라면 실행
}

// if (조건식) else
if (condition) {
    // 조건이 참이라면 실행
} else {
    // 조건이 거짓이라면 실행
}

// if (조건식) else if (조건식)
if (condition1) {
    // 조건1이 참이라면 실행
} else if (condition2) {
    // 조건1이 거짓이고 조건2가 참이라면 실행
} else {
    // 위 모든 조건이 거짓이라면 실행
}
```

### switch문

> 다자택일
> 비교대상이 되는 결과값과 선택사항이 많을 경우 주로 사용

```java
System.out.print("점수를 입력하세요: ");
Scanner inputNum = new Scanner(System.in);
int score = inputNum.nextInt();

switch (score) {
    case 100:
    case 90:
        System.out.println("수");
        break;

    case 80:
        System.out.println("우");
        break;

    case 70:
        System.out.println("미");
        break;

    default:
        System.out.printf("입력한 점수: %s", score);
        break;
}

inputNum.close();	// inputNum 자원 회수
```

## 반복문

> 프로그램 진행을 특정 조건에 따라 반복적으로 진행

### for문

```java
for (int i = 0; i < 10; i++) {
    System.out.println(i);
}
```

### while문

```java
int i = 0;
while (i < 10) {
    System.out.printf("%d ", i);
    i++;
}
```

### do ~ while문

> 조건과 상관없이 최초 한 번 무조건 `{ ... }` 안의 코드를 실행

```java
do {
    System.out.println("무조건 한 번은 실행")
} while (false);
```
