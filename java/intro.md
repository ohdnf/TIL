# Java

## MainClass.java

```java
package test;

public class MainClass {
	// main 치고 Ctrl+Space ==> main method 자동 완성
	// 프로그램을 실행할 때 가장 먼저 실행하는 출발점
	public static void main(String[] args) {
		System.out.println("Hello World");
	}
}
```

- Java에서 파일 하나를 Class라고 지칭한다.
- `public static main(String[] args) {}` 메소드는 프로그램을 실행시켰을 때 가장 먼저 실행되는 곳이다.


## Encapsulation

- Access Modifier
    1. `private`
        - 자신만 사용
    2. (`default`)
        - 같은 package 안에서만 접근 허용
    3. `protected`
        - 상속 관계도 접근 허용
    4. `public`
        - 모두 접근 가능


## Method

### Overloading

- method signature
    - method 이름, parameter 타입, 갯수
    - method signature가 다르면 별개의 method
- 같은 이름으로 method signature가 다른 여러 method를 구현하는 것이 Overloading이다

### Overriding

- 상위 class의 method를 하위 class에서 **재정의**하는 것(`protected`, `public`)

### Interface

- method만 기술
- Java 추상화의 상징
- abstract method(구현되지 않은 body는 실제 Interface를 구현하는 곳에서 작성) + default method(default는 키워드, body가 존재)
- class에 기능/역할 부여
- 서로 다른 class들은 동일한 interface를 구현함으로써 기능/역할의 공통성을 가짐
- `public class Child implements Interface Parent`
- interface는 **다중** 구현이 가능함

## Collection

Java 객체를 여러 개 담고 관리하는 다양한 Container
- `java.util.Collection interface`와 목적에 따른 sub-interface들이 있다.
    - List/Set/Queue/Map

        | interface | 구현 class | 설명 |
        | --------- | ---------- | ---- |
        | List | LinkedList | 순서(O) |
        |  | Stack | 중복(O) |
        |  | ArrayList |  |
        | Set | HashSet | 순서(X) |
        |  | TreeSet | 중복(X) |
        | Queue | LinkedList | 순서(O) |
        |  | PriorityQueue | 중복(O) |
        | Map | HashTable | Key, Value 쌍, 순서(X) |
        |  | HashMap | Key 중복(X), Value 중복(O) |
        |  | TreeMap |  |

- 목적에 맞는 interface를 구현한 Collection Class를 app에서 사용
- 넣기, 꺼내기, 들여다보기, 삭제하기, 순환하기 등의 method 제공

## 예외 처리

- Error vs. Exception
- RuntimeException vs. Other Exception
- try-catch-finally vs. throws
- throw new XXXException();
- AutoCloseable Interface
- User defined Exception
