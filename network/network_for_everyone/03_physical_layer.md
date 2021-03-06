# 물리 계층: 데이터를 전기 신호로 변환하기

## 물리 계층의 역할과 랜 카드의 구조

### 아날로그 vs 디지털

데이터는 0과 1로 이루어진 비트열인데 이를 네트워크를 통해 전달하기 위해선 전기 신호, 즉 **아날로그 신호**로 변환되어야 한다.

### 랜 카드

0과 1의 정보(디지털 신호)를 전기 신호로 변환, 또는 그 반대로 변환을 하는 장비



## 케이블의 종류와 구조

전송 매체란 로 종류가 크게 유선과 무선으로 나뉜다.

### 트위스트 페어 케이블(twisted pair cable)

유선 **전송 매체**(데이터가 흐르는 물리적인 선로)의 한 종류로, 일반적으로 랜 케이블(LAN cable)이라고 한다. 양쪽 끝에 RJ-45라고 부르는 커넥터가 붙어 있어 네트워크 기기 연결에 사용한다.

#### UTP 케이블(Unshielded Twist Pair cable)

구리 선 여덟 개를 두 개씩 꼬아 만든 네 쌍의 전선으로 _실드_로 보호되어 있지 않다.

> 실드(Shield)
>
> 금속 호일이나 금속의 매듭과 같은 것으로 외부에서 발생하는 노이즈(noise)를 막는 역할 담당

#### STP 케이블(Shielded Twist Pair cable)

두 개씩 꼬아 만든 선을 실드로 보호한 케이블



### 다이렉트 케이블 & 크로스 케이블

<img src="https://qph.fs.quoracdn.net/main-qimg-21eca23a0370403e14f4bde9cb10194a" alt="다이렉트 케이블과 크로스 케이블" />

#### 다이렉트 케이블

구리 선 여덟 개를 같은 순서로 커넥터에 연결한 케이블. **컴퓨터와 스위치를 연결**할 때 사용

#### 크로스 케이블

1-2번 선을 3-6번에 연결한 케이블. **컴퓨터 간에 직접 랜 케이블로 연결**할 때 사용

> auto MDIX(Medium-Dependent Interface crossover)
>
> 다이렉트 케이블과 크로스 케이블을 자동으로 판단하는 기능



## 리피터와 허브의 구조

### 리피터

전기 신호를 정형(일그러진 전기 신호를 복원)하고 증폭하는 기능을 가진 네트워크 중계 장비

### 허브(hub)

포트(실제로 통신하는 통로)를 여러 개 가지고 있고 리피터 역할도 수행해 리피터 허브라고도 불린다. 1대1 통신만 가능한 리피터에 비해 여러 대와 통신이 가능하다.

허브는 **어떤 특정 포트로부터 데이터를 받으면 해당 포트를 제외한 나머지 모든 포트로도 받은 데이터를 전송**한다. 스스로 판단하지 않고 모든 포트로 전기 신호를 보내 더미 허브(dummy hub)라고도 불린다.