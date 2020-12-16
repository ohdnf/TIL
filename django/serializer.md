# Serializer

Serializer는 queryset이나 모델 인스턴스와 같은 복잡한 데이터를 Python 자료형으로 바꾸어주고, `JSON`이나 `XML` 과 같은 콘텐츠 형식으로 쉽게 렌더링할 수 있게 도와줍니다. 또한 deserialization을 통해 데이터를 받아 다시 복잡한 형태로 변환해주는 기능을 제공합니다.

Django REST Framework의 serializer는 Django의 `Form`이나 `ModelForm`과 매우 유사합니다. `Serializer`, `ModelSerializer` 클래스를 통해 모델 인스턴스나 쿼리셋 등을 변환해 클라이언트 요청에 대한 응답으로 내보낼 결과들을 손쉽게 제어할 수 있습니다.



## Reference

https://www.django-rest-framework.org/api-guide/serializers/