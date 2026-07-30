import numpy as np
import socket
import json
import os

class UserController:
    def __init__(self, precomputed_file_path=None):
        self.simulationtime = 0.0
        self.PORT = 5005
        #os.environ['WINGLOOP_N'] = "510" 
        
        print("\n" + "="*50)
        print("[TCP Controller] Modulo Server TCP (Bridge) avviato!")
        
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(('0.0.0.0', self.PORT))
        self.srv.listen(1)
        
        print(f"[TCP Controller] In ascolto sulla porta {self.PORT}...")
        print("[TCP Controller] ---> PREMI 'RUN' SU SIMULINK ORA! <---")
        
        self.conn, self.addr = self.srv.accept()
        self.conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def step(self, instantaneous_state, Dt):
        self.simulationtime += Dt
        
        # 1. ESTRAZIONE E RADAR
        try:
            arr = np.array(instantaneous_state).flatten()
            y_phys = arr.tolist()
            max_val = np.max(np.abs(arr)) if len(arr) > 0 else 0.0
        except Exception as e:
            y_phys = [0.0] * 1888
            max_val = 0.0
            
        z_mod = [0.0] * 91

        if int(self.simulationtime/Dt) % 50 == 0:
            print(f"[Python Radar] t={self.simulationtime:.2f}s | Valore Massimo Y={max_val:.6f}")

        # 2. INVIAMO A SIMULINK
        resp = json.dumps({'z': z_mod, 'y': y_phys}).encode()
        try:
            self.conn.sendall(len(resp).to_bytes(4, 'big') + resp)
            
            # 3. RICEVIAMO I COMANDI DA SIMULINK
            hdr = b''
            while len(hdr) < 4: 
                chunk = self.conn.recv(4 - len(hdr))
                if not chunk: raise ConnectionError("Simulink ha chiuso la connessione.")
                hdr += chunk
            mlen = int.from_bytes(hdr, 'big')
            
            raw_data = b''
            while len(raw_data) < mlen: 
                chunk = self.conn.recv(mlen - len(raw_data))
                if not chunk: raise ConnectionError("Simulink ha chiuso la connessione.")
                raw_data += chunk
            data = json.loads(raw_data.decode())
            
            u = data.get('u', [0.0, -6.638, 0.0, 0.0, 23.647, 23.647])
        except Exception as e:
            # Se Simulink si scollega, diciamo a WingLoop di mantenere l'ultimo comando
            u = [0.0, -6.638, 0.0, 0.0, 23.647, 23.647]

        return {"F1":u[0],"F2":u[1],"F3":u[2],"F4":u[3],"E1":u[4],"E2":u[5]}

    def __del__(self):
        try:
            self.conn.close()
            self.srv.close()
        except:
            pass