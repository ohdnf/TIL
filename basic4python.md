# Basic for Python

SW Expert Academy 파이썬 프로그래밍 기초



## 파이썬의 기본 구조와 기초 문법

### 예외처리

* `if` 문

  ```python
  data = input()
  if not data.isdigit():
  	raise TypeError("Must be a number")
  elif data < 0:
      raise ValueError("Must be greater than 0")
  else:
      return 0
  ```

* `try ~ except` 문

  ```python
  try:
  	
  except Exception as ex:
  	# 예외가 발생했을 때 처리
      print("{0}: {1}".format(type(ex), ex))
  else:
      # 예외가 발생하지 않았을 때 실행
  finally:
  	# 예외 발생과 상관없이 실행
  ```

  