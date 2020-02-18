# CS50 by Kang

**강동주** 선생님

5개월 동안 함께할 Mento (- -)(_ _)



## Samsung SW 역량 Test A형 유형 조지기: 네 가지만 떠올려라

(IM은 소위 Index Manipulation...!)

삼성 시험의 경우 `import`를 하면 연산 메모리 안에 모듈을 다 넣어놓기 때문에 채점 시 불리할 수 있다. 기본으로 푸는 연습할 것(`from collections import deque` 같은 경우도 그냥 `list`로 구현하기)

1. Simulation(단순 구현)

2. Brute Force(완전 탐색): Exhaustive Enumeration 포함



> 1, 2번까지는 구현해서 test case까지 돌려보고 나서 3, 4번으로 생각



3. BFS

4. **DP** + Greedy

    무식하게 많이 풀어보는 수 밖에 없다

    재귀 + 메모(Memorization)



## 다른 코테는...?

**자료구조** 및 CS 기본지식 필요!


## Computer Programming Language

### 1. 각 단어 분석

* Computer: 계산/저장하는 기계
* Programming: 명령, 순서
* Language

=> 컴퓨터에게 뭔가를 시킬 때 쓰는 말



### 2. 컴퓨터 프로그래밍 언어의 변천 과정

0, 1 => 어셈블리어 => C언어 => __Python__



### 3. 3형식

```html
저장(=)
조건(if)
반복(while)
```



## 저장
- 어떻게(=)
    - 할당 연산자
- 무엇을(데이터 타입 == 자료형)
    - 숫자
    - 글자
    - boolean
- 어디에(변수 or 컨테이너)
    - 변수
    - 시퀀스형 자료
        - 'string'
        - [list]
        - (tuple)
        - range()
    - 비시퀀스형 자료
        - {set}
        - {dict:}



## 식(Expression) & 문(Statement)
- 조건식(Expression)
    - 값(Value) & 연산자(Operator)
    - 연산자를 통한 값의 평가(Evaluate)가 가능
    - 변수에 할당(바인딩) 가능
- 문(Statement)
    - 식(Expression)으로 구성
    - 콜론(:) 포함
    - 변수 할당 or 바인딩 불가



## 절대법칙

1. Web Browser는 Chrome이다.

2. 교과서는 공식 문서(Documentation)다.

3. User는 또라이다.



## Algorithms

Program that solves a problem

==> Machine way of thinking

### ~~추상화~~, 요약

    1. Abstraction
    2. Reduction

### 패러다임(접근방식)

- brute force

- dynamic programming

- greedy (knapsack)



## 자료구조

### 선형

### 비선형



## OOP(Object-oriented Programming)

프로그래밍을 할 때 인간이 이해하기 쉽게 주어(S)와 서술부(V, predicate)로 추상화, 논리구조를 만드는 것.



## Error

### Error handling

1. Syntactic Error

    `Syntax`: 구문, 문법, 규칙을 나타내는 기본 단위
    확실한 오류 ==> 실행종료

### (Semantic) Debugging

의미상 다른 부분을 고쳐나가는 것

2. Static Semantic Error

    구문적으로 오류가 없으며 (Syntatically valid) 의미를 내포하려 하지만 의미상 오류가 존재

3. Semantic Error

    Syntatically valid하고 Static Semantic Error도 없는 상태이지만 의미가 의도한 의미와 다를 가능성이 존재