# flex

- `flex` 이전에는 배치를 위해서 `float`, `position` 지정을 해야 했다.

## flex 주요 개념

- `container`, `item`
    ```html
    <style>
        .container {
            display: flex;
        }
    </style>
    <div class="container">
      <div class="item"></div>
      <div class="item"></div>
    </div>
    ```

- `main axis`, `cross axis`
- `flex` 정의시
    - `main axis`를 기준으로 배치가 시작된다. (기본은 `row`)
    - 모든 `item`은 행으로 배치된다. (`flex-direction`: `row`값으로 기본 설정)
    - 모든 `item`은 `cross axis`를 모두 채운다. (`align-items`의 기본값: `stretch`)
    - 모든 `item`은 본인의 너비 혹은 컨텐츠 영역만큼 너비를 가지게 된다.
        - 경우에 따라서, 본인이 지정받은 너비보다 작을 수 있다.

## flex 속성

### 1. flex-grow
> `flex-grow`는 남은 너비를 비율로 나눠 가진다.
- 기본값: 0

### 2. flex-justify-content
> main 축을 기준으로 정렬한다.
- 기본값: `flex-start`
- `flex-start`
- `flex-end`
- `center`
- `space-around`
- `space-between`
- `space-evenly`

### 3. align-items