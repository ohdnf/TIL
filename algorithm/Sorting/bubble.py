def bubble(arr):
    for i in range(len(arr)-1, 0, -1):
        for j in range(0, i):
            if arr[j] > arr[j+1]:
                arr[j], arr[j+1] = arr[j+1], arr[j]
            print(i, j, arr)
    return arr


def reverse_bubble(arr):
    for i in range(len(arr)):
        for j in range(i, len(arr)-1):
            if arr[j] > arr[j+1]:
                arr[j], arr[j+1] = arr[j+1], arr[j]
            print(i, j, arr)
    return arr


data = [55, 7, 78, 12, 42]

# res1 = bubble(data)
res2 = reverse_bubble(data)
# print('fin:', res1)
print('fin:', res2)
