# 반복문

초기 C 언어에서는 `goto`문을 활용해 반복문을 구현

```c
#include <stdio.h>

int main()
{
	int n = 1;

label:
	printf("%d\n", n);
	n = n + 1;

	if (n == 10) goto out;

	goto label;

out:
	return 0;
}
```

```c
#include <stdio.h>

int main()
{
	int n = 1;

    while (n < 11)
    {
        printf("%d\n", n);
        n = n + 1;	// 종료 조건을 충족시킬 코드가 필요하다.
    }

	return 0;
}
```
