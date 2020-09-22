const EventEmitter = require('events');

const myEvent = new EventEmitter();
myEvent.addListener('event1', () => {
  console.log('Event 1');
});
myEvent.on('event2', () => {
  console.log('Event 2');
});
myEvent.on('event2', () => {
  console.log('Event 2 second');
});
myEvent.once('event3', () => {
  console.log('Event 3');
}); // 한 번만 실행됨

myEvent.emit('event1');
myEvent.emit('event2');

myEvent.emit('event3');
myEvent.emit('event3');

myEvent.on('event4', () => {
  console.log('Event 4');
});
myEvent.removeAllListeners('event4');
myEvent.emit('event4');

const listener = () => {
  console.log('Event 5');
};
myEvent.on('event5', listener);
// myEvent.removeListener('event5', listener);
myEvent.off('event5', listener);  // removeListener와 같은 기능
myEvent.emit('event5');

console.log(myEvent.listenerCount('event2'));