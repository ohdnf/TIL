<template>
  <div class="todolist">
    <h1>TodoList</h1>
    <ul>
      <li v-for="todo in todos" :key="todo.createdAt">
        <input type="checkbox" name="isCompleted" id="isCompleted" @change="check($event)">
        <span :class="{done: isCompleted}">{{todo.title}}</span>
      </li>
    </ul>
    <p>{{ newTodo }}</p>
  </div>
</template>

<script>
export default {
  name: 'TodoList',
  props: {
    newTodo: String,
  },
  data() {
    return {
      todos: [],
    }
  },
  methods: {
    check() {
      
    }
  },
  computed: {
    createdTodo() {
      let goal = {
        title: this.newTodo,
        isCompleted: false,
        createdAt: Date.now()
      }
      this.todos.push(goal)
    }
  }
}
</script>

<style>
  .done {
    text-decoration: line-through;
  }
</style>