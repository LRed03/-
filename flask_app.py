from flask import Flask, request, jsonify

app = Flask(__name__)

# 假设我们有一个方法来处理设置参数
# 这里我们用一个简单的字典来模拟存储参数
parameters = {
    "new_param1": 0.0,
    "new_param2": 0.0
}

@app.route('/set_parameters', methods=['POST'])
def set_parameters():
    global parameters
    data = request.json
    for key in data:
        if key in parameters:
            parameters[key] = data[key]
    return jsonify(parameters)

@app.route('/get_parameters', methods=['GET'])
def get_parameters():
    return jsonify(parameters)

if __name__ == '__main__':
    app.run(port=5000)
