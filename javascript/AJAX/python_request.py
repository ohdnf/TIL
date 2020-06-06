import requests

res = requests.get('https://koreanjson.com/posts/1')
data = res.json()
post_content = data[0].get('content')
print(res)
print(data)