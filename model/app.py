from flask import Flask, jsonify

app = Flask(__name__)

@app.route('/example', methods=['GET'])
def example():
    data = {
        "Item": "Screwdriver",
        "location": "A1",
        "onHandCount": "4"
    }
    return jsonify(data)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)

