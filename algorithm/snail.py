def snail(row, col):
    width, height = col, row
    arr = [[0 for _ in range(width)] for _ in range(height)]

    num = 1
    y = 0
    x = -1
    position = 1

    # (0,1) -> (1,0) -> (0,-1) -> (-1,0)
    while num <= row*col:
        for _ in range(width):
            x += position
            arr[y][x] = num
            num += 1
        height -= 1
        for _ in range(height):
            y += position
            arr[y][x] = num
            num += 1
        width -= 1
        position *= -1
    
    print(*arr, sep='\n')
    return arr

l = snail(5, 4)