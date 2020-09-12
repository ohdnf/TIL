const express = require('express')
const app = express()
const router = express.Router()
const path = require('path')

const LocalStrategy = require('passport-local').Strategy

// DB Setting
const mysql = require("mysql")
const passport = require('passport')

const connection = mysql.createConnection({
  host: 'localhost',
  port: 3306,
  user: 'root',
  password: 'q1w2e3r4',
  database: 'jsman'
})

connection.connect()

router.get('/', function(req, res) {
  console.log('GET /join URL')
  const errMsg = req.flash('error')
  if (errMsg) {
    const msg = errMsg
  }
  // res.sendFile(path.join(__dirname, '../public/join.html'))
  res.render('join.ejs', {'message': msg})
})


// session 관리
passport.serializeUser((user, done) => {
  console.log('passport session save: ', user.id)
  done(null, user.id)
})
passport.deserializeUser((id, done) => {
  console.log('passport session get id: ', id)
  done(null, id)
})

// passport strategy 설정
passport.use('local-login', new LocalStrategy({
  usernameField: 'email',
  passwordField: 'password',
  passReqToCallback: true
}, function(req, email, password, done) {   // 콜백함수
  const query = connection.query('select * from user where email=?', [email], function(err, rows) {
    if (err) return done(err)
    if (rows.length) {
      return done(null, {email: email, id: rows[0].UID})
    } else {
      return done(null, false, {message: 'Login failed'})
    }
  })
}))

// form을 ajax로 받았으니 custom callback으로 json 응답 보내기
router.post('/', (req, res, nxt) => {
  passport.authenticate('local-login', (err, user, info) => {
    if (err) res.status(500).json(err)
    if (!user) return res.status(401).json(info.message)
    
    req.logIn(user, err => {
      if (err) return nxt(err)
      return res.json(user)
    })
  })(req, res, nxt)
})

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리