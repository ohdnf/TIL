# 고가용성(HA, High Availability)

서버와 네트워크, 프로그램 등의 정보 시스템이 오랜 기간동안 지속적으로 정상 운영이 가능한 성질

## 용어 정리

- Active
- Standby
- Master
  - Write
- Slave
  - 병렬 구조(Scale up)를 가지고, 트래픽이 많아질 때 처리를 담당
  - Read
- Backup(Standby와 동일한 개념)

## HA 방식

### 클러스터링(Clustering)

각기 다른 서버들을 하나로 묶어서 하나의 시스템과 같이 작동하도록 하는 것

### 로드밸런싱(Load Balancing)

_부하 분산을 위해서 Virtual IP를 통해 여러 서버에 접속하도록 분배하는 기능_

- 로드밸런싱(기능)을 하려면 클러스터링(구조)이 필요하다
- 하나의 서비스를 하나 이상의 노드가 처리하는 방식
- 서버의 로드를 **클러스터링**된 서버별로 균등하게 나눔
- 로드밸런싱을 해주는 소프트웨어 혹은 하드웨어를 **로드밸런서**

#### Layer 4 Load Balancing

- 로드밸런서를 클러스터링 구조로 가져갈 수도 있다(Single Point Failure 방지)
- Fail Over: 이중화 구조에서 Health Check를 통해 Standby를 Active하는 것
- [참고](https://www.digitalocean.com/community/tutorials/what-is-load-balancing#how-does-the-load-balancer-choose-the-backend-server)

### RAID(Redundant Array of Independent Disks)

_여러 개의 하드 디스크에 일부 중복된 데이터를 나눠서 저장하는 기술_

#### RAID 0

2개 이상의 disk에 '하나의 데이터'를 분산 저장

- High Risk - High Return

- 극단적인 성능 추구를 위한 Disk 구성

#### RAID 1

지속적인 가용성을 위하여 실시간으로 별도의 물리적인 디스크에 복제

- 속도(성능)보다는 안정성 확보
- 디스크 손상되더라도 데이터의 복구를 완벽히 수행
- Read/Write를 분리하여 어느 정도 성능 향상 기대 가능

> 질문: 왜 데이터를 분산 저장하는 것이 성능 향상인가?
>
> 하나의 디스크에서 처리하는 것보다 동시에 여러 디스크에서 처리할 수 있기 때문

#### RAID 0 + 1

Striping 방식과 Mirroring 방식을 혼합한 형태
