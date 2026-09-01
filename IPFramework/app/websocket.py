import json

from fastapi import APIRouter, WebSocket, WebSocketDisconnect

from .state import state

ws_router = APIRouter()


@ws_router.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    state.ws_clients.append(ws)
    print(f"[WS] Cliente conectado. Total: {len(state.ws_clients)}")
    try:
        while True:
            data = await ws.receive_text()
            try:
                cmd = json.loads(data)
                action = cmd.get("action")

                if action == "set_controller":
                    name = cmd.get("controller", "LQR")
                    state.current_controller_type = name
                    state.controller.set_controller_type(name)
                    gains = cmd.get("gains")
                    if gains and len(gains) == 4:
                        state.controller.set_gains(gains)

                elif action == "set_gains":
                    gains = cmd.get("gains", [])
                    if len(gains) == 4:
                        state.controller.set_gains(gains)

            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        pass
    finally:
        if ws in state.ws_clients:
            state.ws_clients.remove(ws)
        print(f"[WS] Cliente desconectado. Total: {len(state.ws_clients)}")
