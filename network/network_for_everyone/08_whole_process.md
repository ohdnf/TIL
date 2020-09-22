# 네트워크의 전체 흐름 살펴보기



## 랜 카드에서 데이터 전달과 처리

### OSI 모델 계층별 역할

| OSI 모델 계층                  | 역할                                                     |
| ------------------------------ | -------------------------------------------------------- |
| 응용 계층(세션/표현 계층 포함) | 애플리케이션 등에서 사용하는 데이터를 송수신하는 데 필요 |
| 전송 계층                      | 목적지에 데이터를 정확하게 전달하는 데 필요              |
| 네트워크 계층                  | 다른 네트워크에 있는 목적지에 데이터를 전달하는 데 필요  |
| 데이터 링크 계층               | 랜에서 데이터를 송수신하는 데 필요                       |
| 물리 계층                      | 데이터를 전기 신호로 변환하는 데 필요                    |

<img src="https://t1.daumcdn.net/cfile/tistory/2609913858E9CE0D1D" alt="OSI model network" />



### 컴퓨터의 데이터가 전기 신호로 변환되는 과정

> 웹 서버에 HTTP 요청을 보낸다고 가정

#### 1. 응용 계층

- 3-way handshake는 이미 완료되어 연결 확립
- 웹 브라우저에서 URL 입력하여 접속
- `GET /index.html HTTP/1.1 ~`과 같은 HTTP 메시지 데이터를 전송 계층에 전달

#### 2. 전송 계층

- **TCP 헤더 + 데이터 = TCP 세그먼트**
- TCP 헤더의 출발지 포트 번호는 1025번 이상인 포트 중에서 무작위로 선택
- 목적지 포트 번호는 HTTP이므로 80번 포트

#### 3. 네트워크 계층

- **IP 헤더 + 세그먼트 = IP 패킷**
- IP 헤더에는 출발지 IP 주소와 목적지 IP 주소가 들어있다.

#### 4. 데이터 링크 계층

- **이더넷 헤더 + 패킷 + 트레일러 = Ethernet 프레임**

#### 5. 물리 계층

- 프레임 데이터가 전기 신호로 변환되어 스위치 등으로 전송됨



## 스위치와 라우터에서 데이터 전달과 처리

### 스위치에서 데이터 전달과 처리

- 컴퓨터(랜 카드)에서 전기 신호로 변환된 데이터가 스위치로 전달
- 스위치의 데이터 링크 계층에서 데이터를 전기 신호로 변환하여 라우터로 전송



### 라우터에서 데이터 전달과 처리

#### 송신 측 라우터

- 스위치에서 전기 신호로 변환된 데이터가 라우터에 도착
- 데이터 링크 계층에서 이더넷 프레임의 목적지 MAC 주소와 라우터 자신의 MAC 주소를 비교
- 주소가 다르다면 무시, 같으면 이더넷 헤더와 트레일러를 분리(역캡슐화)하여 네트워크 계층에 전달
- 네트워크 계층에서 자신의 라우팅 테이블과 목적지 IP 주소를 비교
- 현재 출발지 IP 주소를 라우터 외부 IP 주소(실제로는 WAN)로 변경
  - 예. `192.168.1.10` => `172.16.0.1`
- 데이터 링크 계층으로 전달하여 목적지 라우터로 보내지도록 이더넷 헤더와 트레일러를 붙여(캡슐화) 프레임을 만든다.
- 물리 계층에서 데이터를 전기 신호로 변환하여 네트워크에 전달

#### 수신 측 라우터

- 이더넷 프레임의 목적지 MAC 주소와 자신의 MAC 주소 비교
- 같으면 역캡슐화 진행
- 네트워크 계층으로 전달되면 자신의 라우팅 테이블과 목적지 IP 주소 비교
- 현재 출발지 IP 주소를 라우터의 내부 IP 주소(실제로 LAN)로 변경
  - 예. `172.16.0.1` => `192.168.10.1`
- 데이터 링크 계층으로 전달하여 스위치에 전달되도록 캡슐화
- 물리 계층에서 데이터를 전기 신호로 변환하여 네트워크로 전달



## 웹 서버에서 데이터 전달과 처리

### 웹 서버에서 이루어지는 OSI 모델의 역캡슐화

#### 1. 물리 계층

- 스위치로부터 전기 신호로 변환된 데이터가 전달

#### 2. 데이터 링크 계층

- 이더넷 프레임의 목적지 MAC 주소와 자신의 MAC 주소를 비교
- 주소가 같으면 역캡슐화하여 네트워크 계층에 전달

#### 3. 네트워크 계층

- 목적지 IP 주소와 웹 서버의 IP 주소가 같은지 확인
- 같으면 역캡슐화하여 전송 계층에 전달

#### 4. 전송 계층

- 목적지 포트 번호를 확인해 어떤 애플리케이션으로 전달해야 하는지 판단
- TCP 헤더를 분리(역캡슐화)해 응용 계층에 전달

#### 5. 응용 계층

- HTTP 메시지 데이터 도착!



> ### 정적 라우팅과 동적 라우팅
>
> 라우팅은 패킷을 목적지 컴퓨터까지 보낼 때 최적의 경로를 선택하여 전송하는 것을 뜻합니다. 라우팅은 크게 정적 라우팅과 동적 라우팅으로 나뉩니다.
>
> #### 정적 라우팅
>
> - 관리자가 미리 라우팅 테이블에 경로를 수동으로 추가하는 방법
> - 경로를 고정하거나 경로가 하나인 경우 사용
> - 라우터가 네트워크에 존재하는 모든 목적지 네트워크의 정보를 알아야 함
> - 소규모 네트워크에서 사용
> - 장점
>   - 라우팅 정보가 교환되지 않아 대역폭이 작다
>   - 라우팅 정보가 전달되지 않아 보안 유지
> - 단점
>   - 장애 발생 시 경로 우회 불가능
>
> #### 동적 라우팅
>
> - 네트워크 변경을 자동으로 감지하여 라우팅 테이블을 업데이트
> - 네트워크 장애 발생 시 라우터끼리 정보 교환해 최적의 경로로 전환
> - 대규모 네트워크에서 사용