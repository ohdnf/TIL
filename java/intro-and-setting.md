# Java

## 환경변수 설정

`javac.exe`, `java.exe`를 다른 디렉토리에서도 실행할 수 있도록 하기 위해 환경변수(Path)에 JDK의 `bin` 경로를 등록한다.

- `javac.exe`: 컴파일러
- `java.exe`: JVM 구동 명령

> 시스템 환경 변수 편집 > `JAVA_HOME` 환경변수 추가

### Path 환경변수 확인

```shell
$ java -version
```

## Java 컴파일러와 JVM

### Java 소스 작성부터 프로그램 실행까지의 순서

1. `Test.java` 소스 작성
   ```java
   public class Test {
       public static void main(String[] args) {
           System.out.println("Hello World");
       }
   }
   ```
2. `javac.exe`가 컴파일
   ```shell
   $ javac.exe Test.java
   ```
3. `xxx.class`라는 바이트 코드 파일 생성
   ```shell
   $ ls
   Test.class Test.java
   ```
4. `java.exe`로 JVM 구동
5. LINK(메모리 로딩/실행 준비/실행 결정/초기화): 기계어 > 실행
   ```shell
   $ java xxx
   Hello World
   ```

## Garbage Collector

프로그램 실행에 필요한 메모리를 Garbage Collector가 자동으로 관리한다.

| C계열 프로그램                            | Java 프로그램                                       |
| ----------------------------------------- | --------------------------------------------------- |
| 개발자가 직접 메모리 관리                 | 개발자가 메모리에 접근할 수 없음                    |
| 메모리 누수 발생 시 타 프로그램 동작 멈춤 | Garbage Collector가 불필요한 메모리 회수해서 최적화 |

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

    | interface | 구현 class    | 설명                       |
    | --------- | ------------- | -------------------------- |
    | List      | LinkedList    | 순서(O)                    |
    |           | Stack         | 중복(O)                    |
    |           | ArrayList     |                            |
    | Set       | HashSet       | 순서(X)                    |
    |           | TreeSet       | 중복(X)                    |
    | Queue     | LinkedList    | 순서(O)                    |
    |           | PriorityQueue | 중복(O)                    |
    | Map       | HashTable     | Key, Value 쌍, 순서(X)     |
    |           | HashMap       | Key 중복(X), Value 중복(O) |
    |           | TreeMap       |                            |

- 목적에 맞는 interface를 구현한 Collection Class를 app에서 사용
- 넣기, 꺼내기, 들여다보기, 삭제하기, 순환하기 등의 method 제공

## 예외 처리

- Error vs. Exception
- RuntimeException vs. Other Exception
- try-catch-finally vs. throws
- throw new XXXException();
- AutoCloseable Interface
- User defined Exception
