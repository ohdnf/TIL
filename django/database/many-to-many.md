# Many to Many Relationship

## 중개모델 활용

    ```py
    class Doctor(models.Model):
        name = models.CharField(max_length=10)

    class Patient(models.Model):
        name = models.CharField(max_length=10)
        # M:N 필드! Reservation 통해서, Doctor에 접근을 의미
        doctors = models.ManyToManyField(Doctor,
                                        through='Reservation',
                                        related_name='patients')    # 역참조 설정


    class Reservation(models.Model):
        doctor = models.ForeignKey(Doctor, on_delete=models.CASCADE)
        patient = models.ForeignKey(Patient, on_delete=models.CASCADE)

    ```

## 중개모델 없이 설정

    ```py
    class Doctor(models.Model):
        name = models.TextField()

    class Patient(models.Model):
        name = models.TextField()
        doctors = models.ManyToManyField(Doctor,
                            related_name='patients')
    ```

## 결론

- 중개모델이 필요없는 경우
  - 특정 Class에 `ManyToManyField` 선언(중개 테이블 자동선언)
- 중개모델이 필요한 경우(추가 정보가 필요한 경우)
  - 중개모델을 정의한 후
  - 특정 Class에 `ManyToMayField` + `through` 옵션을 통해 조작
- 그리고, ManyToMany에서 `related_name` 속성은 반드시 **모델의 복수형**으로 표현한다.(ex. `.like_users`, `like_articles`)
