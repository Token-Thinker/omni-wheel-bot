from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def index():
    print("Serving index.html")
    return render_template('index.html')

if __name__ == '__main__':
    print("Starting server...")
    app.run(host='0.0.0.0', port=5001)
    print("Server stopped.")
