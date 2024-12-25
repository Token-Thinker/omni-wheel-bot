from websocket_server import WebsocketServer

# Callback when a client connects
def on_client_connect(client, server):
    print(f"Client connected: {client['id']}")

# Callback when a message is received
def on_message(client, server, message):
    print(f"Received message: {message}")
    server.send_message(client, f"Echo: {message}")

# Start the server
server = WebsocketServer(host="0.0.0.0", port=9001)
server.set_fn_new_client(on_client_connect)
server.set_fn_message_received(on_message)
print("WebSocket Echo Server running on ws://localhost:9001")
server.run_forever()