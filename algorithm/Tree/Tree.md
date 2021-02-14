# Tree

> 루트(Root) 값과 부모-자식 관계의 서브트리(Sub-tree)로 구성되며, 서로 연결된 노드(Node)의 집합
>
> 재귀로 정의된 자기 참조 자료구조(부모가 트리면 자식도 트리고, 그 자식도 트리다)

```
면접에서 왜 물어보는가?
- 지원자의 재귀 호출과 실행 시간 분석에 관한 지식을 테스트
```

## 목차

- [vs. 그래프](#vs-그래프)
- [이진 트리](#이진-트리)
  - [종류](#종류)
  - [순차배열로 구현하기](#순차-자료구조배열를-이용한-구현)
  - [트리 순회](#트리-순회)
    - [전위 순회](#전위-순회)
    - [중위 순회](#중위-순회)
    - [후위 순회](#후위-순회)
- [BST](#bst)
  - [검색](#검색)
  - [삽입](#삽입)
  - [삭제](#삭제)
- [자가 균형 이진 탐색 트리](#자가-균형-이진-탐색-트리)
  - [AVL 트리](#AVL-트리)
  - [레드-블랙 트리](#레드-블랙-트리)



## vs. 그래프

> 우리 트리는요...

- **순환 구조(Cycle)가 없는 그래프**다.
- 부모 노드에서 자식 노드를 가리키는 **단방향 그래프**이다.
- 어떤 노드도 **단 하나의 부모 노드**를 갖는다.
- **단 하나의 루트 노드(최상위 노드)**를 가지며 루트 노드에서 시작한다.

<img src="https://adrianmejia.com/images/tree-parts.jpg" alt="img" style="zoom:80%;" />



## 이진 트리

> 모든 노드의 **차수(degree)가 2 이하**인 트리

### 종류

<img src="https://adrianmejia.com/images/full-complete-perfect-binary-tree.jpg" alt="img" style="zoom:80%;" />

- 정 이진 트리(Full binary tree)

  모든 노드가 0개 또는 2개의 자식 노드를 갖는다.

- 완전 이진 트리(Complete binary tree)

  마지막 레벨을 제외하고 모든 레벨이 완전히 채워져 있으며, 마지막 레벨의 모든 노드는 가장 왼쪽부터 채워져 있다.

- 포화 이진 트리(Perfect binary tree)

  모든 노드가 2개의 자식 노드를 갖고 있으며, 모든 리프 노드가 동일한 깊이 또는 레벨을 갖는다.

### 순차 자료구조(배열)를 이용한 구현

> 계산을 쉽게 하기 위해 인덱스 0번은 사용하지 않는다.

노드 개수가 `n`개인 이진 트리를 1차원 배열을 사용하여 표현 할 때

| 노드                        | 인덱스        | 성립 조건          |
| --------------------------- | ------------- | ------------------ |
| 노드 `i`의 부모 노드        | `i // 2`      | `i > 1`            |
| 노드 `i`의 왼쪽 자식 노드   | `2 * i`       | `(2 * i) <= n`     |
| 노드 `i`의 오른쪽 자식 노드 | `(2 * i) + 1` | `(2 * i) + 1 <= n` |
| 루트 노드                   | `1`           | `n > 0`            |

### 트리 순회

> 순회(Traversal)란 모든 노드를 빠트리거나 중복하지 않고 정확히 한 번씩 방문(처리)하는 연산

```
N: 현재 노드 자신
R: 현재 노드의 오른쪽 서브트리
L: 현재 노드의 왼쪽 서브트리
```

<img src="https://upload.wikimedia.org/wikipedia/commons/thumb/6/67/Sorted_binary_tree.svg/250px-Sorted_binary_tree.svg.png" alt="정렬된 이진 트리"/>

#### 전위 순회

> Pre-order, NLR
>
> 현재 노드를 먼저 순회한 다음 왼쪽과 오른쪽 서브트리 순회

```
preorder(node)
	if (node == null)
		return
	visit(node)
	preorder(node.left)
	preorder(node.right)
```

- 방문 순서: F B A D C E G I H

#### 중위 순회

> In-order, LNR
>
> 왼쪽 서브트리를 순회한 다음, 현재 노드, 그 다음 오른쪽 서브트리 순회

```
inorder(node)
	if (node == null)
		return
	inorder(node.left)
	visit(node)
	inorder(node.right)
```

- 방문 순서: A B C D E F G H I
- 이진 탐색 트리의 모든 노드 값들을 순서대로 나열하는 방법

#### 후위 순회

> Post-order, LRN
>
> 왼쪽과 오른쪽 서브트리를 모두 순회한 다음 현재 노드 순회

```
postorder(node)
	if (node == null)
		return
	postorder(node.left)
	postorder(node.right)
	visit(node)
```

- 방문 순서: A C E D B H I G F



## BST

> Binary Search Tree(이진 탐색 트리)

- 각 노드에 저장된 값은 유일(unique)하다.
- 값들은 *전순서*가 있다. 즉, 임의의 두 노드는 비교가 가능하다.
- 부모 노드의 **왼쪽 서브트리**에는 **부모 노드의 값보다 작은 값들을 지닌 노드들**로 이루어져 있다.
- 부모 노드의 **오른쪽 서브트리**에는 **부모 노드의 값보다 큰 값들을 지닌 노드들**로 이루어져 있다.
- 좌우 서브트리도 이진 탐색 트리다.
- 균형 BST에서 검색(look-up), 삭제, 삽입의 시간 복잡도는 `O(log(n))` 이다.
  - `O(log(n))`은 `O(h)`로도 쓸 수 있는데, `h`는 트리의 높이다.
- 최악의 경우(skewed tree) 연결 리스트와 동일한 `O(n)` 연산을 보인다.

### 검색

```
search(node, x)
	while node:
		if node.value == x:
			break
		elif node.value < x:
			node = node.right
		elif node.value > x:
			node = node.left
	return node
```

- 값 `x`를 가진 노드를 검색하고자 할 때, 트리에 해당 노드가 존재하면 해당 노드를 반환, 존재하지 않으면 `null`을 반환한다.
- 검색하고자 하는 값을 루트 노드와 먼저 비교하고, 일치할 경우 루트 노드를 반환한다.
  - 불일치하고 루트 노드의 값이 `x`보다 클 경우 왼쪽 서브트리에서 재귀적으로 검색한다.
  - 불일치하고 루트 노드의 값이 `x`보다 작을 경우 오른쪽 서브트리에서 재귀적으로 검색한다.

### 삽입

```python
"""
class Node:
	def __init__(self, value, left=None, right=None):
		self.value = value
		self.left = left
		self.right = right
"""
def insert(root, x):
	curr = root		# 삽입할 자리
	parent = None	# 삽입할 노드의 부모 노드

	# 삽입할 노드 탐색
	while curr:
		if curr.value == x:		# 같은 값이 있다면 삽입 종료
			return
		parent = curr
		if curr.value < x:
			curr = curr.right
		elif curr.value > x:
			curr = curr.left

	# 삽입할 노드 생성
	new = Node(x)

	# 삽입 노드 연결
	if root is None:
		root = new
	elif parent.value > x:
		parent.left = new
	elif parent.value < x:
		parent.right = new

	return
```

- 삽입을 하기 전, 검색을 수행한다.
- 트리를 검색한 후 삽입하려는 값과 일치하는 노드가 없으면 마지막 노드에서 키와 노드의 크기를 비교해 왼쪽이나 오른쪽에 새로운 노드를 삽입한다.

### 삭제

```python
def delete(root, x):
    curr = root		# 삭제할 노드
    parent = None	# 삭제할 노드의 부모 노드
    
    # 삭제할 노드 탐색
	while curr:
		if curr.value == x:		# 삭제할 노드를 찾으면 탐색 종료
			break
		parent = curr
		if curr.value < x:
			curr = curr.right
		elif curr.value > x:
			curr = curr.left
    
    # 삭제할 노드가 없는 경우
    if curr is None:
        return
    
    # 삭제할 노드의 차수가 0인 경우
    if curr.left is None and curr.right is None:
        if parent.left.value = x:
            parent.left = None
        elif parent.right.value = x:
            parent.right = None
    # 삭제할 노드의 차수가 1인 경우
    elif curr.left is None or curr.right is None:
        if curr.left:	# 삭제할 노드의 왼쪽 자식노드 처리
            if parent.left == curr:
                parent.left = curr.left
            else:
                parent.right = curr.left
        else:	# 삭제할 노드의 오른쪽 자식노드 처리
            if parent.left == curr:
                parent.left = curr.right
            else:
                parent.right = curr.right
    # 삭제할 노드의 차수가 2인 경우
    elif curr.left and curr.right:
        successor = max_node(curr.left)		# 왼쪽 서브트리에서 후계자 결정
        # successor = min_node(curr.right)	# 오른쪽 서브트리에서 후계자 결정
        curr.value = successor.value	# 삭제할 노드 자리에 후계자 노드 값 반영
        delete(curr.left, curr.value)	# 후계자 노드 자리를 재귀적으로 삭제
        # delete(curr.right, curr.value)
    return
```



삭제하려는 노드의 자식 수에 따라

- 자식 노드가 없는 노드(리프 노드) 삭제: 해당 노드를 단순히 삭제한다.

- 자식 노드가 1개인 노드 삭제: 해당 노드를 삭제하고 그 위치에 해당 노드의 자식 노드를 대입한다.

- 자식 노드가 2개인 노드 삭제: 삭제하고자 하는 노드의 값을 해당 노드의 왼쪽 서브트리에서 가장 큰 값으로 변경하거나, 오른쪽 서브트리에서 가장 작은 값으로 변경한 뒤, 해당 노드(왼쪽 서브트리에서 가장 큰 값을 가지는 노드 또는 오른쪽 서브트리에서 가장 작은 값을 가지는 노드)를 삭제한다.

  <img src="assets/tree_delete_2.png" alt="delete node with two subnodes in bst" style="zoom: 50%;"/>


