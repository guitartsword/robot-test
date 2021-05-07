from flask import Flask
from web_server.prueba_joints import connect, move
from flask_cors import CORS

app = Flask(__name__)
CORS(app)


client_id = connect(19999)
print(client_id)

@app.route('/potenciomentro1/<int:degree>')
def potenciomentro1(degree):
    move(client_id, 0, degree*0.0175)
    return f'{degree} grados'

@app.route('/potenciomentro2/<int(signed=True):degree>')
def potenciomentro2(degree):
    print(degree)
    if degree < 0:
        print(degree)
        degree += 360
    print(degree)
    move(client_id, 1, degree*0.0175)
    return f'{degree} grados'
