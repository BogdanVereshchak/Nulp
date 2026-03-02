from flask import Flask

app = Flask(__name__)
counter = 0

@app.route("/")
def hello():
    global counter
    counter += 1
    return f"Вітаю! Цю сторінку відвідали {counter} разів!"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
