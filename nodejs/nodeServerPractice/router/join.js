const express = require('express')
const app = express()
const router = express.Router()
const path = require('path')
const passport = require('passport')
const LocalStrategy = require('passport-local').Strategy
const session = require('express-session')
const flash = require('connect-flash')

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
  res.sendFile(path.join(__dirname, '../public/join.html'))
})

passport.use('local-join', new localStrategy({
  usernameField: 'email',
  passwordField: 'password',
  passReqToCallback: true
}, function(req, email, password, done) {
  const query = connection.query('select * from user where email=?', )
}))

router.post('/', passport.authenticate('local-join', {
    successRedirect: '/main',
    failureRedirect: '/join',
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