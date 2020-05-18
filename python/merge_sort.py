def merge(left, right):
    result = list()
    if left and not right:
        return left
    if not left and right:
        return right
    while left or right:
        if left and right:
            if left[0] <= right[0]:
                result.append(left.pop(0))
            else:
                result.append(right.pop(0))
        elif left:
            result.append(left.pop(0))
        else:
            result.append(right.pop(0))
    return result

def merge_sort(data):
    if len(data) == 1:
        return data
    
    left = list()
    right = list()
    mid = len(data) // 2
    left = data[:mid]
    right = data[mid:]
    
    left = merge_sort(left)
    right = merge_sort(right)

    return merge(left, right)

if __name__ == "__main__":
    arr = [5, 1, 21, 7, 6, 13, 9, 8, 6, 4, 10]
    result = merge_sort(arr)
    print(result)