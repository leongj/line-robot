from flask import Flask, request, jsonify
from datetime import datetime

app = Flask(__name__)

# In-memory storage for requests
request_history = []

@app.route('/readings', methods=['GET'])
def receive_message():
    # Get the request parameters
    ir1 = request.args.get('ir1')
    ir2 = request.args.get('ir2')
    ir3 = request.args.get('ir3')
    ir4 = request.args.get('ir4')
    
    # Add timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    stored_data = {
        "timestamp": timestamp,
        "ir1": ir1,
        "ir2": ir2,
        "ir3": ir3,
        "ir4": ir4
    }
    
    # Store the request in memory
    request_history.append(stored_data)
    
    # Print the request to console
    print(f"[{timestamp}] Received IR values: ir1={ir1}, ir2={ir2}, ir3={ir3}, ir4={ir4}")
    
    # Return a response
    return jsonify({"status": "success", "message": "Request received"})

@app.route('/history', methods=['GET'])
def get_history():
    # Return all stored requests
    return jsonify(request_history)

if __name__ == '__main__':
    print("API server starting. Listening for requests...")
    app.run(debug=True, host='0.0.0.0', port=5000)