const header = {
  "typ": "JWT",
  "alg": "HS256"
};

// encode to base64
const encodedHeader = new Buffer(JSON.stringify(header))
                            .toString('base64')
                            .replace('=', '');
console.log('header: ', encodedHeader);

/* Result:
header: eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9
*/