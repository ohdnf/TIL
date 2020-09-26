const mongoose = require('mongoose');

const { Schema } = mongoose;
const userSchema = new Schema({
  // _id를 기본키로 자동 생성함
  name: {
    type: String,
    required: true,
    unique: true,
  },
  age: {
    type: Number,
    required: true,
  },
  married: {
    type: Boolean,
    required: true,
  },
  comment: String,
  createdAt: {
    type: Date,
    default: Date.now,
  }
})

module.exports = mongoose.model('User', userSchema);