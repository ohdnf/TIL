# 스프링 빈과 의존관계

## 스프링 빈(Bean)

- Spring IoC(Inversion of Control) 컨테이너가 관리하는 Java 객체
- `new` 연산자로 생성된 객체는 빈이 아님
- `ApplicationContext.getBean()`으로 얻어질 수 있는 객체
- 즉, `ApplicationContext`가 만들어서 그 안에 담고 있는 객체

## 스프링 빈을 등록하는 두 가지 방법

### 1. 컴포넌트 스캔과 자동 의존관계 설정

- 생성자에 `@Autowired`가 있으면 스프링이 객체 생성 시점에 연관된 객체를 스프링 컨테이너에서 찾아서 넣어준다(Dependency Injection, 의존성 주입).
- 생성자가 1개만 있으면 `@Autowired`는 생략할 수 있다.
- [참고](https://atoz-develop.tistory.com/entry/Spring-%EC%8A%A4%ED%94%84%EB%A7%81-%EB%B9%88Bean%EC%9D%98-%EA%B0%9C%EB%85%90%EA%B3%BC-%EC%83%9D%EC%84%B1-%EC%9B%90%EB%A6%AC)

```java
// MemberController.java

@Controller
public class MemberController {

    private final MemberService memberService;

    @Autowired
    public MemberController(MemberService memberService) {
        this.memberService = memberService;
    }
}
```

- 하지만, 현재 `memberService`는 순수한 Java Class이므로 스프링 컨테이너가 `memberService`를 알 수 있는 방법(컴포넌트 스캔)이 없다.
- 따라서 컴포넌트 스캔이 가능하도록 빈을 등록해야 한다.

```java
// MemberService.java

@Service
public class MemberService {

    private final MemberRepository memberRepository;

    @Autowired
    public MemberService(MemberRepository memberRepository) {
        this.memberRepository = memberRepository;
    }

    ...
```

- `memberRepository`를 DI했으니 Repository 구현체(`MemoryMemberRepository`)를 빈으로 등록하자.

```java
// MemoryMemberRepository.java


@Repository
public class MemoryMemberRepository implements MemberRepository{

    private static Map<Long, Member> store = new HashMap<>();
    private static Long sequence = 0L;

    ...
```

> `helloController`는 스프링이 제공하는 컨트롤러여서 스프링 빈으로 자동 등록된다. `@Controller`가 있으면 자동 등록된다.

#### 컴포넌트 스캔 원리

- `@Component` Annotation이 있으면 스프링 빈으로 자동 등록된다.
- `@Controller`, `@Service`, `@Repository`는 모두 내부에 `@Component`를 포함하고 있어 스프링 빈으로 자동 등록된다.

> 스프링은 스프링 컨테이너에 스프링 빈을 등록할 때, 기본으로 싱글톤으로 등록한다(유일하게 하나만 등록해서 공유한다). 따라서 같은 스프링 빈이면 모두 같은 인스턴스다. 싱글톤이 아닌 설정이 가능하지만, 특별한 경우를 제외하면 대부분 싱글톤으로 사용한다.

---

### 2. 자바 코드로 직접 스프링 빈 등록하기
