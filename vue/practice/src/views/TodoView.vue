<template>
  <div class="todoview">
    <TodoInput @submit-todo="createTodo"/>
    <TodoList :newTodo="whatTodo"/>
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
      whatTodo: ''
    }
  },
  methods: {
    createTodo(data) {
      this.whatTodo = data
    }
  }
}
</script>

<style>

</style>