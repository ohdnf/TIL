<template>
  <div class="todo-list">
    <h1>TodoList</h1>
    <ul>
      <li v-for="todo in todos" :key="todo.id" :class="{ done: todo.isCompleted }">
        <input type="checkbox" :checked="todo.isCompleted" @change="$emit('checked', todo)">
        {{ todo.goal }}
      </li>
    </ul>
    <p>{{ newTodo }}</p>
  </div>
</template>

<script>
export default {
  name: 'TodoList',
  props: {
    todos: {
      type: Array,
      required: true
    }
  }
}
</script>

<style>
  ul {
    list-style-type: none;
  }
  
  .done {
    text-decoration: line-through;
  }

</style>