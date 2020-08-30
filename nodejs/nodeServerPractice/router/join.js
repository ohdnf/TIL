const express = require('express')
const app = express()
const router = express.Router()
const path = require('path')

// DB Setting
const mysql = require("mysql")

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

router.post('/', (req, res) => {
  const body = req.body
  const name = body.nickname
  const email = body.email
  const password = body.password

  const sql = {email: email, name: name, password: password}
  const query = connection.query('insert into user set ?', sql, (err, rows) => {
    if (err) { throw err }
    // console.log("New user joined!", rows)
    // rows == OkPacket {
    //   fieldCount: 0,
    //   affectedRows: 1,
    //   insertId: 5,
    //   serverStatus: 2,
    //   warningCount: 0,
    //   message: '',
    //   protocol41: true,
    //   changedRows: 0
    // }
    else res.render('main.ejs', {id: rows.insertId, name: name})
  })
})

module.exports = router;  // 다른 파일에서 main.js를 사용할 수 있게 exports 처리