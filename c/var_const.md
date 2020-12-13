# 변수와 상수

## 변수

> 데이터를 저장하기 위해 프로그램에 의해 이름을 할당받은 메모리 공간

```c
int angel = 1004;
```

- `int` 자료형
- `angel` 변수(variable)
- `1004` 리터럴 상수(literal constant)



## 상수

> 변수와 마찬가지로 데이터를 저장할 수 있는 메모리 공간
>
> 변수와 다른 점은 프로그램이 실행되는 동안 **상수에 저장된 데이터는 변경할 수 없다.**

```c
const int angel = 1004;
```

- `const` 한정자, 제한자(qualifier)
- `int` 자료형
- `angel` 기호적 상수(symbolic constant)
- `1004` 리터럴 상수(literal constant)

### 리터럴 상수 vs 심볼릭 상수

#### 리터럴 상수

- 변수와 달리 데이터가 저장된 메모리 공간을 가리키는 이름을 가지지 않음
- 타입에 따라 정수형(`123`), 실수형(`3.14`), 문자형(`'a'`) 등으로 구분

#### 심볼릭 상수

- 반드시 선언과 동시에 초기화되어야 함
- `const` 키워드를 사용하거나, 매크로를 이용해 선언
- 이름은 대문자로 선언하는 것이 관례

> 컴파일하면서 전처리기 지시자 `#define` 다음에 오는 상수를 그 다음에 오는 값으로 바꿔준다.

```c
#include <stdio.h>
#define PI 3.141592f
#define AI_NAME "Jarvis"

int main()
{
    flaot radius, area, circum;
    
    printf("I'm %s.\nPlease, input radius...\n", AI_NAME);
    scanf("%f", &radius);
    
    area = PI * radius * radius;
    circum = 2.0 * PI * radius;
    
    printf("Area = %f\n", area);
    printf("Circumference = %f\n", circum);
}
```

> 또는 변수명 앞에 `const` 를 붙여서 선언할 수 있다.

```c
#include <stdio.h>
// #define PI 3.141592f
#define AI_NAME "Jarvis"

int main()
{
    const float pi = 3.141592f;
    flaot radius, area, circum;
    
    printf("I'm %s.\nPlease, input radius...\n", AI_NAME);
    scanf("%f", &radius);
    
    area = PI * radius * radius;
    circum = 2.0 * PI * radius;
    
    printf("Area = %f\n", area);
    printf("Circumference = %f\n", circum);
}
```

