# Java 객체

## 객체 지향 프로그래밍이란

### 객체

세상에 존재하는 모든 것을 뜻하며, 프로그래밍에서 **속성**과 **기능**을 가지는 프로그램 단위

### 클래스

객체를 생성하기 위한 **틀**로 모든 객체는 클래스로부터 생성

#### 클래스의 구성요소

- 속성(멤버 변수)
- 기능(메서드)

## 클래스 제작과 객체 생성

### 클래스 제작

```java
// 클래스 이름은 대문자로 시작한다.
public class TeslaCar {
    // 속성(멤버 변수)
    public String color;
    public String model;
    public int price;

    // 생성자
    public TeslaCar() {
        /*
        클래스 이름과 동일
        객체가 생성될 때 가장 먼저 호출되는 곳
        생성에 필요한 것들을 이곳에 정의
        생성자를 작성하지 않으면 컴파일 시 자동으로 빈 생성자 호출
        */
        System.out.println("Tesla model constructor");
    }

    // 메서드(기능)
    public void run() {
        // 호출했을 때 수행해야 할 기능 구현
        System.out.println("--run--");
    }

    public void stop() {
        System.out.println("--stop--");
    }
}
```

### 클래스로부터 `new`를 이용해서 객체를 생성

```java
TeslaCar myRoadster = new TeslaCar();
myRoadster.color = "Red";
myRoadster.model = "Roadster";
myRoadster.price = 106000;

myRoadster.run();
myRoadster.stop();
```

## 메서드

### 메서드 선언과 호출

> 메서드도 변수와 같이 선언 및 정의 후 필요시에 호출해서 사용

```java
// 메서드 선언부
// 접근자 반환형 이름()
public void getInfo() {
    // 메서드 정의부
    System.out.println('i = ', i);
}
```

- 메서드 이름은 동사형, camelCase로 작성

### 매개변수

```java
public void setInfo(int i, boolean b, double d, char c, String s) {
    System.out.println('i = ', i);
    System.out.println('b = ', b);
    System.out.println('d = ', d);
    System.out.println('c = ', c);
    System.out.println('s = ', s);
}
```

### 중복 메서드

> Overloading은 이름은 동일하지만 매개변수를 달리 하는 메서드를 만드는 것

```java
public void getInfo() {
    // 기본
}

public void getInfo(int x, int y) {
    // 매개변수의 개수가 다르다
}

public void getInfo(String s1, String s2) {
    // 매개변수의 타입이 다르다
}
```

### 접근자

> 메서드를 호출할 때 접근자에 따라서 호출이 불가할 수 있다

```java
// public 메서드
public void getInfo() {
	// 외부에서 호출 가능
}

// private 메서드
private void getInfo() {
    // 외부에서 호출 불가
    // 클래스 내부에서만 사용
}
```

## 객체와 메모리

### 메모리에서 객체 생성(동적 생성)

> 객체는 메모리에서 동적으로 생성되며, 객체가 더 이상 필요없게 되면 GC(Garbage Collector)에 의해서 제거

### 레퍼런스

> 생성한 객체의 주소를 변수에 저장하는 것을 레퍼런스라고 한다.

```java
ObjectClass obj1 = new ObjectClass();
ObjectClass obj2 = new ObjectClass();
ObjectClass obj3 = new ObjectClass();

System.out.println("obj1 ----> " + obj1);
System.out.println("obj2 ----> " + obj2);
System.out.println("obj3 ----> " + obj3);

/*
결과
obj1 ----> lec14Pjt001.ObjectClass@7d6f77cc
obj2 ----> lec14Pjt001.ObjectClass@5aaa6d82
obj3 ----> lec14Pjt001.ObjectClass@73a28541
*/
```

### 자료형이 같아도 다른 객체

```java
if(obj1 == obj2) {
    System.out.println("obj1 == obj2");
} else {
    System.out.println("obj1 != obj2");
}

if(obj2 == obj3) {
    System.out.println("obj2 == obj3");
} else {
    System.out.println("obj2 != obj3");
}

if(obj1 == obj3) {
    System.out.println("obj1 == obj3");
} else {
    System.out.println("obj1 != obj3");
}

/*
obj1 != obj2
obj2 != obj3
obj1 != obj3
*/
```

### `null`과 `NullPointException`

> 레퍼런스에 `null`이 저장되면 객체와 연결이 끊기며, 더 이상 객체를 이용할 수 없다.
> 연결이 끊긴 객체는 GC에 의해 제거된다. 레퍼런스는 재사용할 수 있다.

```java
System.out.println("obj1 ----> " + obj1);
obj1.getInfo();

// obj1 ----> lec14Pjt001.ObjectClass@7d6f77cc
// -- getInfo method --


obj1 = null;
System.out.println("obj1 ----> " + obj1);
obj1.getInfo();

// Exception in thread "main" ObjectClass
// constructorjava.lang.NullPointerException
//	at lec14Pjt001.MainClass.main(MainClass.java:44)
// obj1 ----> null
```

## 생성자와 소멸자 그리고 `this`

### 디폴트 생성자

> 객체가 생성될 때 가장 먼저 호출되는 생성자로, 개발자가 명시하지 않아도 컴파일 시점에 자동 생성된다.

### 사용자 정의 생성자

> 디폴트 생성자 외 특정 목적에 의해서 개발자가 만든 생성자로, 매개변수에 차이가 있다.

```java
public ObjectEx() {

    System.out.println("Default constructor");

}

public ObjectEx(int i) {

    System.out.println("Custom constructor");
    num = i;

}

public ObjectEx(String s, int i[]) {

    System.out.println("UserDefined constructor");
    str = s;
    nums = i;

}
```

### 소멸자

> 객체가 GC에 의해서 메모리에서 제거될 때 `finalize()` 메서드가 호출된다.

#### 클래스 내부 정의부

```java
@Override
protected void finalize() throws Throwable {

    System.out.println(" -- finalize() method --");

    super.finalize();
}
```

#### 소멸자 호출

```java
ObjectEx obj4;

obj4 = newObjectEx();	// 첫 번째 생성
obj4 = newObjectEx();

System.gc();	// 첫 번째 생성된 객체를 소멸
```

- `System.gc();`를 사용한다고 해서 GC가 바로 작동하는 것이 아니라, 가급적 빨리 작동하도록 요청하는 것이다.
- Java는 기본적으로 메모리를 개발자가 직접 관리하지 않으므로 일반적으로 `System.gc();`를 사용하는 경우는 드물다.

### `this` 키워드

```java
public class ObjectEx {

	int x;	// 전역 x
	int y;	// 전역 y

    public ObjectEx(int x, int y) {
        // this를 사용해서 전역변수에 매개변수 들어온 값을 대입
        this.x = x;
        this.y = y;
    }
}
```

## 패키지와 `static`

> class 파일을 효율적으로 관리하기 위한 방법과 객체 간 속성 또는 메서드를 공유하는 방법

### 패키지(`package`)

> 클래스를 폴더 형식으로 관리하는 것

#### 패키지 이름 결정 요령

- 패키지가 속해 있는 클래스가 최대한 다른 클래스와 중복되는 것을 방지
- 패키지 이름은 일반적으로 도메인을 거꾸로 이용
- 개발 중 패키지 이름과 구조는 변경 가능
- 패키지 이름만 보고 속성과 기능을 예측할 수 있도록 작성

```
📁src
├─📁com.java.dailyJournal
│   └─DailyJournal.java
├─📁com.java.employee
│   ├─Fulltime.java
│   └─Parttime.java
├─📁com.java.main
│   └─Main.java
├─📁com.java.pay
│   └─Payment.java
└─📁com.java.util
    └─Util.java
```

### `import`

> 다른 패키지에 있는 클래스를 사용하기 위해서는 `import` 키워드를 이용한다.

```java
package com.java.pay;

import com.java.dailyJournal.DailyJournal;
import com.java.employee.*;

public class Payment {
    DailyJournal journal = new DailyJournal();
    Fulltime fulltime = new Fulltime();
    Parttime parttime = new Parttime();
}
```

### `static`

> 클래스의 속성과 메서드에 `static` 키워드를 사용하면 생성된 객체 간 속성과 메서드를 공유할 수 있다.

```java
public class InstanceCounter {

   private static int numInstances = 0;

   protected static int getCount() {
      return numInstances;
   }

   private static void addInstance() {
      numInstances++;
   }

   InstanceCounter() {
      InstanceCounter.addInstance();
   }

   public static void main(String[] arguments) {
      System.out.println("Starting with " + InstanceCounter.getCount() + " instances");

      for (int i = 0; i < 500; ++i) {
         new InstanceCounter();
      }
      System.out.println("Created " + InstanceCounter.getCount() + " instances");
   }
}

/*
결과
Started with 0 instances
Created 500 instances
*/
```

## 데이터 은닉

> 객체가 가지고 있는 데이터를 외부로부터 변질되지 않게 보호하는 방법

### 멤버변수의 `private` 설정

> 속성은 주로 `private`로 설정해서 외부에서 접근하는 것을 막는다.

```java
public class InstanceCounter {

	private int sturdyAttr = 0;
    public int vulnerableAttr = 0;

    public static void main(String[] arguments) {

    }
}
```

### `setter`, `getter`

> 외부에서 속성에 접근할 수 있도록 설정하는 메서드

```java
public class InstanceCounter {

	private int sturdyAttr = 0;
    public int vulnerableAttr = 0;

    protected int getSturdyAttr() {
        return sturdyAttr;
   }

    protected void setSturdyAttr(int arg) {
        this.sturdyAttr = arg;
    }
}
```
