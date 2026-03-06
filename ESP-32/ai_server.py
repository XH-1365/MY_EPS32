from flask import Flask, request, jsonify
import requests

app = Flask(__name__)

API_KEY = "你的火山API_KEY"

URL = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"

MODEL = "deepseek-v3"

@app.route("/ai", methods=["POST"])
def ai():

    data = request.json
    text = data.get("text","")

    headers = {
        "Authorization": f"Bearer {API_KEY}",
        "Content-Type": "application/json"
    }

    payload = {
        "model": MODEL,
        "messages":[
            {"role":"user","content":text}
        ]
    }

    r = requests.post(URL, headers=headers, json=payload)

    result = r.json()

    reply = result["choices"][0]["message"]["content"]

    return jsonify({
        "reply": reply
    })


app.run(host="0.0.0.0", port=5000)