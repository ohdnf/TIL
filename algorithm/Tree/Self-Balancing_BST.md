# Self-Balancing BST

> 자가 균형 이진 탐색 트리, Self-Balancing Binary Search Tree

![assets/Untitled.png](assets/Untitled.png)

- 트리가 비효율적으로 구성된 경우 연결 리스트와 다르지 않다.
- 자가 균형 이진 탐색 트리는 **높이를 가능한 한 낮게 유지**해 탐색의 성능을 높인다.
- **모든 노드의 좌우 서브트리의 높이 차가 1 이하**다.
- 노드의 삽입, 삭제, 검색의 시간 복잡도가 `O(logN)`이다.
- 연관 배열(associative array) 등을 구현하는데 쓰이는 자료구조이다.
- 대표적으로 *AVL 트리*와 *레드-블랙 트리*가 있다.

# AVL 트리

![assets/avl_bf.png](assets/avl_bf.png)

## 특징

- 왼쪽 서브트리 < 부모 노드 < 오른쪽 서브트리의 크기 관계를 갖는다.
- 각 노드의 *왼쪽 서브트리 높이(hL)와 오른쪽 서브트리 높이(hR)의 차이(hL-hR)*를 노드의 **균형 인수(BF, Balance Factor)**라 한다.
- 각 노드의 균형 인수로 `{-1, 0, 1}` 값만 가지게 함으로써 균형을 유지한다.
- BF가 2 이상일 때, `+`이면 왼쪽 서브트리, `-`이면 오른쪽 서브트리에 문제가 있는 것이다.

균형 이진 탐색 트리에서는 노드의 삽입, 삭제 작업 후 균형 인수를 확인해 균형을 맞추는 재구성 작업이 필요하다. AVL 트리는 **회전(Rotation) 연산**을 통해 불균형을 해결한다. 다음은 어떤 노드를 삽입/삭제했을 때 생길 수 있는 유형들을 나타낸다.

## LL 유형

![assets/avl_ll.png](assets/avl_ll.png)

```
LL_rotate(L1)
	L2의 오른쪽 자식 노드를 L1의 왼쪽 자식 노드로 옮긴다.
	L1을 L2의 오른쪽 자식 노드로 옮긴다.
end LL_rotate()
```

## RR 유형

![assets/avl_rr.png](assets/avl_rr.png)

```
RR_rotate(L1)
	L2의 왼쪽 자식 노드를 L1의 오른쪽 자식 노드로 옮긴다.
	L1을 L2의 왼쪽 자식 노드로 옮긴다.
end RR_rotate()
```

## LR 유형

![assets/avl_lr.png](assets/avl_lr.png)

```
LR_rotate(L1)
	L2와 L3 구간에 대한 RR_rotate(L2)를 수행하고 반환된 노드 L3를 L1의 왼쪽 자식 노드로 만든다.
	L1에 대해서 LL_rotate(L1)를 수행한다.
end LR_rotate()
```

## RL 유형

![assets/avl_rl.png](assets/avl_rl.png)

```
RL_rotate(L1)
	L2와 L3 구간에 대한 LL_rotate(L2)를 수행하고 반환된 노드 L3를 L1의 오른쪽 자식 노드로 만든다.
	L1에 대해서 RR_rotate(L1)을 수행한다.
```

# 레드-블랙 트리

> 비교 가능한 자료를 정리하는 데 쓰이는 자료 구조
자식이 없는 노드를 리프 노드(leaf node)라고 부르고, 레드-블랙 트리에서 리프 노드는 비어 있다. 즉, 자료를 가지고 있지 않다(Nil로 표현).

복잡한 자료 구조이지만, 삽입, 삭제, 검색에서 **최악의 경우에도 일정한 실행 시간**($\log_2n$**)을 보장**하기 때문에 실사용에서 효율적이다.

![assets/Untitled%201.png](assets/Untitled%201.png)

## 특성(Properties)

레드-블랙 트리는 아래 다섯 가지 특성을 항상 만족해야 한다!

1. 모든 노드는 **레드** 혹은 **블랙** 중 하나다.
2. 루트 노드는 **블랙**이다.
3. 리프 노드(NIL)는 **블랙**이다.
    - 레드-블랙 트리의 리프(leaf) 노드는 일반적인 리프 노드와 의미가 다르다. 어떤 노드의 자식 노드가 없는 쪽에(즉, 자식 포인터 중 NIL이 있는 곳에), NIL 값을 갖는 노드를 만드는데 이것이 바로 레드-블랙 트리의 리프 노드이다.
    - 즉, 자료를 갖지 않으며, 트리의 끝을 나타내는 데만 쓰인다.
    - 다소 어색할 수 있지만 알고리즘에서 리프 노드가 개입될 때 특수 처리를 하지 않아도 되는 편리함을 갖는다.
    - 실제 구현 시엔 노드 하나를 리프로 정하고 모든 리프 노드에 대한 포인터를 이 노드를 가리키도록 한다. (그림 참고)

        ![assets/Untitled%202.png](assets/Untitled%202.png)

        (a)를 (b)와 같이 만들어 (c)처럼 사용한다.

4. **레드** 노드의 자식노드는 **블랙** 노드이다.
   
    - 따라서 **레드** 노드끼리는 부모-자식 관계가 될 수 없다.
5. 어떤 노드로부터 시작되어 그에 속한 하위 리프 노드에 도달하는 모든 경로에는 리프 노드를 제외하면 모두 같은 개수의 **블랙** 노드가 있다.

## 높이

**루트 노드로부터 가장 먼 경로까지의 거리가, 가장 가까운 경로까지의 거리의 두 배보다 항상 작다!**

- $h(x)$

    어떤 노드 `x`의 높이 `h(x)`는 자신으로부터 리프 노드까지 가장 긴 경로에 포함된 간선(edge)의 개수

- $bh(x)$

    어떤 노드 `x`의 높이 `bh(x)`는 `x`로부터 리프 노드까지의 경로 상에 존재하는 **블랙** 노드의 개수 (노드 `x` 자신 불포함)

- $bh \ge h/2$

    높이가 `h`인 노드의 **블랙** 높이 `bh`는 **특성 5**에 의해 레드 노드가 연속될 수 없으므로 `bh >= h/2`

    ![assets/red-black_bh.png](assets/red-black_bh.png)

- $2^{bh(x)}-1$

    노드 `x`를 루트로 하는 임의의 서브트리는 적어도 $2^{bh(x)}-1$개의 내부 노드를 포함한다.

- $2\log_2(n+1)$

    $n \ge 2^{bh(x)}-1 \ge 2^{h/2}-1$이므로, `n`개의 내부 노드를 가지는 레드-블랙 트리의 높이 `h`는 $2\log_2(n+1)$이다.

## 동작

- 탐색 과정은 이진 탐색 트리(BST)의 동작과 똑같다.
- 그러나 삽입과 삭제 후 레드-블랙 트리의 특성을 위반하는 경우가 발생한다!
- 삽입/삭제 후 레드-블랙 트리의 특성을 만족시키기 위해서는 `O(logn)` 또는 `amortized O(1)`번의 색 변환(실제로는 매우 빨리 이루어진다)과 최대 3회(삽입의 경우 2회)의 트리 회전이 필요하다.

### 회전

AVL 트리에서 봤던 바로 그 회전이다...! 레드-블랙 트리의 특성을 만족하지 못할 때 회전 연산을 수행한다.

![assets/red-black_rotate.png](assets/red-black_rotate.png)

$$⁍$$

![assets/Tree_rotation_animation_250x250.gif](assets/Tree_rotation_animation_250x250.gif)

- Left-rotation 예시

    ![assets/Untitled%203.png](assets/Untitled%203.png)

    5를 기준으로 왼쪽으로 돌리면

    ![assets/Untitled%204.png](assets/Untitled%204.png)

    10이 5의 부모 노드가 되고, 8은 5의 오른쪽 자식 노드가 된다.

    ![assets/Untitled%205.png](assets/Untitled%205.png)

    기존에 연결되어 있는 자식 노드들을 표시하면 아래와 같다.

    ![assets/Untitled%206.png](assets/Untitled%206.png)

    ```
    LEFT-ROTATE(T, x)
    	y = x.right
    	x.right = y.left
    	if y.left != T.nil
    		y.left.p = x
    	y.p = x.p
    	if x.p == T.nil
    		T.root = y
    	elseif x == x.p.left
    		x.p.left = y
    	else x.p.right = y
    	y.left = x
    	x.p = y
    ```

### 삽입(Insertion)

새로 추가되는 노드는 무조건 **레드** 노드다. 삽입 노드에서는 삼촌 노드(Uncle node) 개념이 등장한다. 삼촌 노드는 부모 노드의 형제 노드를 말한다.

- 삼촌 노드와 할아버지 노드

    삽입 후 **레드**-**블랙** 트리의 특성을 위반하는 경우를 설명하기 위해 필요하다. 앞으로

    - 삽입하는 원소는 **N**
    - **N**의 부모 노드는 **P**
    - **P**의 부모 노드는 **G**
    - **N**의 삼촌 노드는 **U**

    로 나타내기로 한다.

    ![assets/red-black_uncle.png](assets/red-black_uncle.png)

삽입 후 발생하는 경우의 수는 총 *다섯 가지(Five cases)*가 존재한다.

- Case 1

    새로운 노드 **N**이 트리의 루트 노드가 되는 경우다. **레드** 노드를 삽입하고 **블랙**으로 바꾼다.

- Case 2

    새로운 노드 **N**의 부모 노드 **P**가 **블랙** 노드일 경우, 모든 특성을 만족하기 때문에 아무런 조치를 취하지 않는다.

- Case 3

    **상황**

    부모 노드 **P**와 삼촌 노드 **U**가 모두 **레드** 노드일 경우

    ![assets/Untitled%207.png](assets/Untitled%207.png)

    **해결 방법**

    **P**와 **U**를 모두 **블랙**로 바꾸고 할아버지 노드 **G**를 **레드**로 바꾼다.

    이 경우 **G**가 두 가지 특성을 만족하지 않을 수 있다.

    1. **G**가 루트 노드일 경우 특성 2(루트 노드는 **블랙** 노드)를 만족하지 않는다.
    2. **G**의 부모 노드가 **레드**일 경우 특성 4(**레드** 노드의 자식 노드는 **블랙**)를 만족하지 않는다.

    이를 해결하기 위해 할아버지 노드 **G**에 대해 *Case 1*부터 *Case 3*를 재귀적으로 적용한다.

- Case 4-1

    **상황**

    - 부모 노드 **P**는 **레드** 노드, 삼촌 노드 **U**는 **블랙** 노드인 경우
    - 새로운 노드 **N**이 **P**의 **오른쪽 자식** 노드
    - **P**는 할아버지 노드 **G**의 **왼쪽 자식** 노드

    ![assets/Untitled%208.png](assets/Untitled%208.png)

    **해결 방법**

    **P**에 대해 **왼쪽 회전(left rotation)**을 한다. 이후 **P**와 **N**이 특성 4(**레드** 노드의 자식 노드는 **블랙**)를 만족하지 않기 때문에 *Case 4-2*에서 처리한다.

- Case 4-2

    **상황**

    - 부모 노드 **P**는 **레드** 노드, 삼촌 노드는 **U**는 **블랙** 노드일 경우
    - 새로운 노드 **N**이 부모 노드 **P**의 **왼쪽 자식** 노드
    - **P**가 할아버지 노드 **G**의 **왼쪽 자식** 노드

    ![assets/Untitled%209.png](assets/Untitled%209.png)

    **해결 방법**

    **G**에 대해 **오른쪽 회전(right rotation)**을 한다. **P**와 **G**의 색을 반대로 바꾼다.

### 삭제(Removal)

# 참고

책 'C로 배우는 쉬운 자료구조(개정3판)' 이지영 저, 한빛아카데미

책 '알고리즘: 문제해결 중심으로' 국형준 저, 교보문고

## AVL 트리

[https://ratsgo.github.io/data structure&algorithm/2017/10/27/avltree/](https://ratsgo.github.io/data%20structure&algorithm/2017/10/27/avltree/)

## 레드-블랙 트리

### 영상

[https://www.youtube.com/watch?v=A3JZinzkMpk&list=PL9xmBV_5YoZNqDI8qfOZgzbqahCUmUEin&index=4](https://www.youtube.com/watch?v=A3JZinzkMpk&list=PL9xmBV_5YoZNqDI8qfOZgzbqahCUmUEin&index=4)

### 문서

[https://ko.wikipedia.org/wiki/레드-블랙_트리](https://ko.wikipedia.org/wiki/%EB%A0%88%EB%93%9C-%EB%B8%94%EB%9E%99_%ED%8A%B8%EB%A6%AC)

[https://nesoy.github.io/articles/2018-08/Algorithm-RedblackTree](https://nesoy.github.io/articles/2018-08/Algorithm-RedblackTree)

[https://itstory.tk/entry/레드블랙-트리Red-black-tree](https://itstory.tk/entry/%EB%A0%88%EB%93%9C%EB%B8%94%EB%9E%99-%ED%8A%B8%EB%A6%ACRed-black-tree)

[https://assortrock.com/87](https://assortrock.com/87)

[http://www.euroinformatica.ro/documentation/programming/](http://www.euroinformatica.ro/documentation/programming/)!!!Algorithms_CORMEN!!!/DDU0076.html

[https://zeddios.tistory.com/237](https://zeddios.tistory.com/237)