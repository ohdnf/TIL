# 디자인 패턴



## OOP 원칙: SOLID

- Single Responsibility Principle

  > 하나의 클래스는 하나의 역할만 해야 한다.

- Open / Closed Principle

  > 확장(상속)에는 열려있고, 수정에는 닫혀있어야 한다.

- Liskov Substitution Principle

  > 자식이 부모의 자리에 항상 교체될 수 있어야 한다.

- Interface Segregation Principle

  > 인터페이스가 잘 분리되어서, 클래스가 꼭 필요한 인터페이스만 구현하도록 해야 한다.

- Dependency Inversion Principle

  > 상위 모듈이 하위 모듈에 의존하면 안 된다.
  > 둘 다 추상화에 의존하며, 추상화는 세부 사항에 의존하면 안 된다.



## GoF(Gang of Four) Design Pattern

객체 지향 개발을 위한 설계 Best Practices를 모았더니 세 가지 분류(생성, 구조, 행위), 총 23개 패턴으로 정의됨



### 생성 패턴(Creational Pattern): 객체의 생성 방식 결정

- Class-creational patterns, Object-creational patterns
- DB Connections를 관리하는 Instance를 하나만 만들 수 있도록 제한하여 불필요한 연결을 막는다.



### 구조 패턴(Structural Pattern): 객체 간의 관계를 조직

- 2개의 인터페이스가 서로 호환이 되지 않을 때, 둘을 연결해주기 위해서 새로운 클래스를 만들어서 연결시킬 수 있도록 한다.



### 행위 패턴(Behavioral Pattern): 객체의 행위를 조직, 관리, 연합

- 하위 클래스에서 구현해야 하는 함수 및 알고리즘을 미리 선언하여, 상속 시 필수로 구현하도록 한다.



## 디자인 패턴의 이점

- 개념화와 다이어그램(UML)으로 표현이 쉽고 해결법을 재사용 가능
- 개발자 간 커뮤니케이션 용이
- 확장성, 재사용성, 유지보수성이 좋은 소프트웨어 설계



## 디자인 패턴 사용시 주의사항

> "If you only have a hammer, you tend to see every problem as a nail"
>
> Abraham Maslow