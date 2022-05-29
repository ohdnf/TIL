# 함수

## 함수 만들기

> 함수 선언부와 정의부를 나눈다.

```c
void say(void);		// function declaration, prototype

int main()
{
    say();

    return 0;
}

void say(void)		// function definition
{
    printf("Hello, World!\n");

    // return;
}
```

## 인수(arguments)와 매개변수(parameters)

### Arguments vs. Parameters

- actual argument, actual parameter -> argument (values)

  - 함수에 실제로 들어가는 값

- formal argument, formal parameter -> parameter (variables)
  - 함수에서 쓰일 값을 정의한 변수

###
