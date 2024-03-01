import json
from flask import Flask, render_template
# from flask import Flask, request, jsonify, render_template
# from flask_cors import CORS, cross_origin

app = Flask(__name__)
# CORS(app)


@app.route("/")
def home():
	return render_template("WebUI.html")

# app.run(host='192.168.137.230',debug=False)
app.run(host='10.9.72.7',debug=False)