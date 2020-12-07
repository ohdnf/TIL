# 객체지향 프로그래밍이란?

> 이 질문에 대해 누군가는 "데이터와 함수의 조합"이라고 답할 수 있다. 또는 이 질문에 흔히 "실제 세계를 모델링하는 새로운 방법"이라고들 답한다.
> ...
> 또는 Object-Oriented(OO)가 캡슐화(Encapsulation), 상속(Inheritance), 다형성(Polymorphism), 이 세 가지 개념을 가진 것이라고 말한다.
> ...
> 하지만 소프트웨어 아키텍트 관점에서 정답은 명백하다. OO란 **다형성을 이용하여 전체 시스템의 모든 소스 코드 의존성에 대한 절대적인 제어 권한을 획득할 수 있는 능력**이다. OO를 사용하면 아키텍트는 플러그인 아키텍처를 구성할 수 있고, 이를 통해 고수준의 정책을 포함하는 모듈은 저수준의 세부사항을 포함하는 모듈에 대해 독립성을 보장할 수 있다. 저수준의 세부사항은 중요도가 낮은 플러그인 모듈로 만들 수 있고, 고수준의 정책을 포함하는 모듈과는 독립적으로 개발하고 배포할 수 있다.
>
> 로버트 C. 마틴, 『클린 아키텍처』에서



라고 면접장에서 답할 수는 없으니까...

- 프로그래밍 패러다임 중 하나
- 현실 세계의 사물들을 객체라고 보는 것처럼 프로그램을 구성하고 로직으로 프로그래밍하는 것
- 대규모 소프트웨어 개발에 많이 사용된다.
- 개발과 유지보수가 편하고 추상적이기 떄문에 직관적인 코드 분석을 가능하게 한다.

라고 말하며 시작해보자!



## 클래스와 객체

### 클래스

속성(attribute; property, state)과 행동(actions; capability, method)을 가진 같은 종류의 무언가를 추상적으로 정의한 것

### 객체

다른 객체 인스턴스와는 다른 별도의 상태를 가진 어떤 클래스의 특정 인스턴스



## 캡슐화(Encapsulation)

- 객체의 속성(data fields)과 행위(methods)를 하나로 묶고,
- 실제 구현 내용 일부를 외부로부터 감추어 은닉한다.



## 상속(Inheritance)과 다형성(Polymorphism)

> 상속은 어떤 클래스에서 더 특화된 버전의 클래스를 위한 행동을 제공할 수 있게 한다.
>
> 다형성은 하나의 메소드나 클래스가 존재할 때, 이것들이 다양한 방법으로 동작하는 것을 의미한다. 오버로딩이나 오버라이딩을 통해서 상황에 따라 적당한 구현을 가능하게 해준다.

### Overloading

> 같은 함수 이름을 가지고 있으나 매개변수, 리턴타입 등의 특징이 다르게 생성하는 것

### Overriding

> 서브클래스(자식 클래스)가 자신의 슈퍼클래스(부모 클래스)에 의해 이미 제공된 메소드를 특정한 형태로 구현하는 것

매개변수의 타입과 갯수가 같다.

```python
class Thought(object):
    def __init__(self):
        pass
    def message(self):
        print "I feel like I am diagonally parked in a parallel universe."

class Advice(Thought):
    def __init__(self):
        super(Advice, self).__init__()
    def message(self):
        print "Warning: Dates in calendar are closer than they appear"
        super(Advice, self).message()
```



## 객체지향적 설계 원칙: SOLID

### SRP(Single Responsibility Principle)

> **단일 책임 원칙**
> 모든 클래스는 하나의 책임만 가지며, 클래스는 그 책임을 완전히 캡슐화해야 한다.

- 책임이란 *변경하려는 이유*
- 단 하나의 책임을 갖는다는 것은 변경하는 이유가 단 하나여야 한다는 것
- 한 클래스를 한 관심사에 집중하도록 유지하는 것이 중요한 이유는, 이것이 클래스를 더욱 튼튼하게 만들기 때문이다.

### OCP(Open/Closed Principle)

> **개방-폐쇄 원칙**
> 소프트웨어 개체(클래스, 모듈, 함수 등)는 확장에 열려 있어야 하고, 수정에 대해서는 닫혀 있어야 한다.

- *확장에 대해 열려있다*는 것은 모듈의 동작을 확장할 수 있다, 즉 모듈이 하는 일을 변경할 수 있다는 것을 의미
- *수정에 대해 닫혀있다*는 것은 모듈의 소스 코드나 바이너리 코드를 수정하지 않아도 모듈의 기능을 확장하거나 변경할 수있다는 것을 의미
- 기능을 추가하거나 변경해야 할 때 이미 제대로 동작하고 있던 원래 코드를 변경하지 않아도 되는 것
- 모듈을 수정해야 할 때 그 모듈을 이용하는 다른 모듈들도 고쳐야 한다면 개발 작업이 쉽지 않다.

### LSP(Liskov Substitution Priciple)

> **리스코프 치환 원칙**
> 프로그램의 객체는 프로그램의 정확성을 깨뜨리지 않으면서 하위 타입의 인스턴스로 바꿀 수 있어야 한다.
> 즉, 컴퓨터 프로그램에서 자료형 S가 자료형 T의 하위형이라면 필요한 프로그램의 속성(정확성, 수행하는 업무 등)의 변경 없이 자료형 T의 객체를 자료형 S의 객체로 교체(치환)할 수 있어야 한다.

#### 전형적인 위반(Circle-ellipse problem)

- 너비와 높이의 `getter` 및 `setter` 메서드를 가진 직사각형 클래스로부터 정사각형 클래스를 파생하는 경우 LSP를 위반하게 된다.
- 직사각형을 다루는 문맥에서 정사각형 객체가 사용되는 경우, 정사각형 객체의 `setter` 메서드는 직사각형의 `setter` 메서드와 다르기 때문에(정사각형과 직사각형의 정의를 생각해보자. 직사각형은 너비와 높이가 독립적이지만 정사각형은 그렇지 않다.) LSP를 위반하게 된다.
- 정사각형과 직사각형이 `getter` 메서드만 가진다면(즉, 불변 객체라면) LSP 위반은 발생하지 않는다.

### ISP(Interface Segregation Principle)

> **인터페이스 분리 원칙**
> 클라이언트가 자신이 이용하지 않는 메서드에 의존하지 않아야 한다. 즉, 클라이언트에게 필요없는 메서드를 가지고 있는 인터페이스라면 분리해야 한다. 따라서 인터페이스는 그 인터페이스를 사용하는 클라이언트를 기준으로 분리해야 한다.

- 특정 클라이언트를 위한 인터페이스 여러 개가 범용 인터페이스 하나보다 낫다.
- 인터페이스가 잘 분리되어서, 클래스가 꼭 필요한 인터페이스만 구현하도록 해야 한다.

### DIP(Dependency Inversion Principle)

> **의존관계 역전 원칙**
> 프로그래머는 "추상화에 의존해야지, 구체화에 의존하면 안된다."

- 소프트웨어 모듈들을 분리하는 특정 형식을 지칭
- 상위 계층(정책 결정)이 하위 계층(세부 사항)에 의존하는 전통적인 의존관계를 역전시킴으로써 상위 계층이 하위 계층의 구현으로부터 독립되게 한다.
- 내용
  1. 상위 모듈은 하위 모듈에 의존해서는 안 된다. 상위 모듈과 하위 모듈 모두 추상화에 의존해야 한다.
  2. 추상화는 세부 사항에 의존해서는 안 된다. 세부 사항이 추상화에 의존해야 한다.



## 장점

> 강한 응집력(Strong Cohesion)과 약한 결합력(Weak Coupling)을 지향

- 클래스에 하나의 문제 해결을 위한 데이터를 모아 놓아 응집력을 강화
- 클래스 간에 독립적으로 디자인함으로써 결합력을 약하게 함



## JavaScript에서 OOP

> JavaScript는 기본적으로 Prototype-based Programming Language다.



### 생성자와 new

생성자(constructor)는 객체를 만드는 역할을 하는 함수

```javascript
function Person(name) {			// 생성자
    this.name = name;
    this.introduce = function() {
	    return 'My name is ' + this.name;
    }
}	
let p = new Person('ohdnf');	// 생성자에 new를 붙이면 객체를 생성함
console.log(p.introduce());
```



### 상속

```javascript
function Person(name) {
    this.name = name;
}
// 생성자 안에 prototype이라는 객체 property가 존재
Person.prototype.name = null;
Person.prototype.introduce = function() {
    return 'My name is ' + this.name;
}

function Programmer(name) {
    this.name = name;
}
Programmer.prototype = new Person();	// Programmer에 Person을 상속시킴
Programmer.prototype.say = function() {
    return "Hello, world!";
}

let hongPro = new Programmer('ohdnf');
console.log(hongPro.introduce());
console.log(hongPro.say());
```



### prototype

> 객체와 객체를 연결하는 역할

```javascript
// prototype chain

function Ultra(){}
Ultra.prototype.ultraProp = true;
 
function Super(){}
Super.prototype = new Ultra();
 
function Sub(){}
Sub.prototype = new Super();
 
var o = new Sub();
console.log(o.ultraProp);
```



## 참고

- 로버트 C. 마틴, *클린 아키텍처*, 인사이트
- 존 몽건 외, *프로그래밍 면접 이렇게 준비한다*, 한빛미디어
- [생활코딩](https://opentutorials.org/course/743/6584)
- 위키백과, 객체 지향 프로그래밍
- 위키백과, SOLID