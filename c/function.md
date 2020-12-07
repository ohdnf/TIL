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

