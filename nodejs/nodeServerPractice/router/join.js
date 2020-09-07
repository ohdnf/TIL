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

passport.serializeUser((user, done) => {
  console.log('passport session save: ', user.id)
  done(null, user.id)
})

passport.deserializeUser((id, done) => {
  console.log('passport session get id: ', id)

  done(null, id)
})

// passport strategy 설정
passport.use('local-join', new LocalStrategy({
  usernameField: 'email',
  passwordField: 'password',
  passReqToCallback: true
}, function(req, email, password, done) {   // 콜백함수
  const query = connection.query('select * from user where email=?', [email], function(err, rows) {
    if (err) return done(err)
    if (rows.length) {
      console.log('existed user')
      return done(null, false, {message: 'Your email is already used'})
    } else {
      const sql = {email: email, password: password}
      const query = connection.query('insert into user set ?', sql, function(err, rows) {
        if (err) throw err
        return done(null, {email: email, id: rows.insertId})
      })
    }
  })
}))

router.post('/', passport.authenticate('local-join', {
    successRedirect: '/main',   // 회원가입 성공
    failureRedirect: '/join',   // 중복회원 체크 등 가입 실패
    failureFlash: true 
  })
)

// router.post('/', (req, res) => {
//   const body = req.body
//   const name = body.nickname
//   const email = body.email
//   const password = body.password

//   const sql = {email: email, name: name, password: password}
//   const query = connection.query('insert into user set ?', sql, (err, rows) => {
//     if (err) { throw err }
//     else res.render('main.ejs', {id: rows.insertId, name: name})
//   })
// })

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리