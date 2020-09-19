const string = 'abc'
const number = 1
const boolean = true
const obj = {
  outside: {
    inside: {
      key: 'value'
    }
  }
}

console.time('total time')
console.log('this is a normal log...')
console.log(string, number, boolean)
console.error('Put error logs in console.error(HERE YOU PUT THE ERRORS)')

console.table([{name: 'Zero', birth: 1994}, {name: 'hero', birth: 1988}])

console.dir(obj, {colors: false, depth: 2})
console.dir(obj, {colors: true, depth: 1})

console.time('time count')
for (let i = 0; i < 100000; i++) {}
console.timeEnd('time count')

function b() {
  console.trace('track error location')
}

function a() {
  b()
}

a()

console.timeEnd('total time')