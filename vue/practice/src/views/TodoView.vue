<template>
  <div class="todo-view">
    <TodoInput @create="createTodo"/>
    <TodoList :todos="todos" @checked="updateTodo"/>
  </div>
</template>

<script>
import TodoInput from '@/components/TodoInput.vue'
import TodoList from '@/components/TodoList.vue'

export default {                                                        
  name: 'TodoView',
  components: {
    TodoInput,
    TodoList
  },
  data() {
    return {
      todos: [
        // Dummies
        {id: 1, goal: 'Django 복습', isCompleted: true},
        {id: 2, goal: 'JavaScript 복습', isCompleted: false},
        {id: 3, goal: 'Vue 복습', isCompleted: false},
      ]
    }
  },
  methods: {
    createTodo(todo) {
      this.todos.push(todo)
    },
    updateTodo(todo) {
      todo.isCompleted = !todo.isCompleted
    }
  }
}
</script>

<style>

</style>