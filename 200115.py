# random 모듈

from random import random, uniform, randrange, choice, sample, shuffle

start, stop, step = 1, 45, 2
print("uniform({0}, {1}) => {2}".format(1.0, 10.0, uniform(1.0, 10.0)))

print("randrange({0}, {1}) => {2}".format(start, stop, randrange(start, stop)))
print("randrange({0}) => {1}".format(stop, randrange(stop)))
print("randrange({0}, {1}, {2}) => {3}".format(start, stop, step, randrange(start, stop, step)))

data = [1, 2, 3, 4, 5]
print("choice({0}) => {1}".format(data, choice(data)))
# print("choices({0}) => {1}".format(data, choices(data, k=2)))
print("sample({0}) => {1}".format(data, sample(data, k=2)))

shuffle(data)       # 반환값 없음
print("shuffled data => {0}".format(data))


# datetime 모듈
from datetime import datetime, timezone, timedelta

now = datetime.now()
print("{0}-{1:02}-{2:02} {3:02}:{4:02}:{5:02}".format(now.year, now.month, now.day, now.hour, now.minute, now.second))

fmt = "%Y{0} %m{1} %d{2} %H{3} %M{4} %S{5}"
print(now.strftime(fmt).format(*"년월일시분초"))

# from pytz import common_timezones, timezone, utc

# # 타임존 정보 출력
# for tz in list(common_timezones):
#     if tz.lower().find("paris") >= 0:
#         print(tz)

# tz_utc = timezone(utc.zone)
# tz_seoul = timezone("Asia/Seoul")
# tz_pacific = timezone("US/Pacific")
# tz_paris = timezone("Europe/Paris")

# fmt = "%Y-%m-%d %H:%M:%S %Z%z"

# # UTC 현재 시각
# now_utc = datetime.now(tz_utc)
# print(now_utc.strftime(fmt))

# # Asia/Seoul 타임존
# now_seoul = now_utc.astimezone(tz_seoul)
# print(now_seoul.strftime(fmt))

# # US/Pacific 타임존
# now_pacific = now_seoul.astimezone(tz_pacific)
# print(now_pacific.strftime(fmt))

# # Europe/Paris 타임존
# now_paris = now_pacific.astimezone(tz_paris)
# print(now_paris.strftime(fmt))



