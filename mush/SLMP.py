import socket
from sys import byteorder
import numpy as np

# HOST = '192.168.0.100'
# PORT = 8800
# DATA_CODE = 'Binary'

# Binary
COM_HEADER = [0x50, 0x00]
RES_HEADER = [0xD0, 0x00]
NET_NO = [0x00]
EX_NO = [0xFF]
IO_NO = [0xFF, 0x03]
MULTI_EX_NO = [0x00]
RESERVE = [0x00, 0x00]
NORM_COM_CODE = [0x00, 0x00]
COMMAND = {'read': [0x01, 0x04], 'write': [0x01, 0x14]}
SUB_COMMAND = {'bit': [0x01, 0x00], 'word': [0x00, 0x00]}
DEV_CODE = {'X': [0x9C], 'Y': [0x9D], 'M': [0x90], 'D': [0xA8]}

class SLMP:
    def __init__(self, host, port):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((host, port))
        print('connected')

    def read(self, bw, start_p, device, p_num):
        target = bytes(COM_HEADER + NET_NO + EX_NO + IO_NO + MULTI_EX_NO)
        command = bytes(COMMAND['read'])
        sub_command = bytes(SUB_COMMAND[bw])
        start_point = start_p.to_bytes(3, byteorder='little')
        dev_code = bytes(DEV_CODE[device])
        points_num = p_num.to_bytes(2, byteorder='little')
        reserve = bytes(RESERVE)
        data_len = len(reserve + command + sub_command + start_point + dev_code + points_num).to_bytes(2, byteorder='little')

        req = target + data_len + reserve + command + sub_command + start_point + dev_code + points_num
        print('send data: ', req)
        self.client.send(req)
        res = self.client.recv(1024)
        print('received data: ', res)
        come_code, dev_data = transfer.interpret(self, res)
        if come_code == bytes(NORM_COM_CODE):
            print('Successfully Read')
            # dev_data를 (bw, start_point, dev_code, dev_data)를 기반으로 해석해야 함
        else:
            print('Error')
        
    def write(self, bw, start_p, device, p_num, w_data):
        target = bytes(COM_HEADER + NET_NO + EX_NO + IO_NO + MULTI_EX_NO)
        command = bytes(COMMAND['write'])
        sub_command = bytes(SUB_COMMAND[bw])
        start_point = start_p.to_bytes(3, byteorder='little')
        dev_code = bytes(DEV_CODE[device])
        points_num = p_num.to_bytes(2, byteorder='little')
        reserve = bytes(RESERVE)
        data_len = len(reserve + command + sub_command + start_point + dev_code + points_num + w_data).to_bytes(2, byteorder='little')
        
        req = target + data_len + reserve + command + sub_command + start_point + dev_code + points_num + w_data
        print('send data: ', req)
        self.client.send(req)
        res = self.client.recv(1024)
        print('received data: ', res)
        come_code, dev_data = transfer.interpret(self, res)
        if come_code == bytes(NORM_COM_CODE):
            print('Successfully Written')
        else:
            print('Error')
        
class transfer:
    def bool2bit(self, wData):
        bw = 'bit'
        data = []
        if len(wData) % 2 != 0:
            wData = wData + list(np.zeros((1,), dtype=np.int8))

        for i in range(int(len(wData)/2)):
            data.append(wData[2*i] * 16 + wData[2*i+1])
            
        return bw, len(wData), bytes(data)
        
    def bool2word(self, wData):
        bw = 'word'
        data = b''
        num_rem = len(wData) % 16
        if num_rem != 0:
            wData = wData + list(np.zeros((16-num_rem,), dtype=np.int8))

        for i in range(int(len(wData)/16)):
            num = []
            for j in range(4):
                num.append(wData[8*i+4*j]*(2**0) + wData[8*i+4*j+1]*(2**1) + wData[8*i+4*j+2]*(2**2) + wData[8*i+4*j+3]*(2**3))
            data += int(num[1]*16+num[0]).to_bytes(2, byteorder='little')
            
        return bw, int(len(wData)/16), bytes(data)
        
    def int2word(self, wData):
        bw = 'word'
        data = b''
        for i in range(len(wData)):
            data += wData[i].to_bytes(2, byteorder='little')
            
        return bw, len(wData), data
        
    def interpret(self, rData):
        len_data = int.from_bytes(rData[7:9], byteorder='little')
        com_code = rData[9:11]
        devData = rData[11:11+len_data-2]

        return com_code, devData

# # sample code
# if __name__ == "__main__":
#     HOST = '192.168.0.103' # PLC IP
#     PORT = 9988 # PLC port
#     PLC1 = SLMP(HOST, PORT) # PLC 객체 생성 및 연결
#     trans = transfer() # 데이터 형식 변환

#     # 그리퍼 오픈
#     bw, points, w_data = trans.bool2bit([0, 0])
#     PLC1.write(bw, 204, 'M', points, w_data)
    
#     # # 그리퍼 클로즈
#     # bw, points, w_data = trans.bool2bit([1, 0])
#     # PLC1.write(bw, 204, 'M', points, w_data)

#     # # bool to word
#     # bw, points, w_data = trans.bool2word([1, 0])
#     # PLC1.write(bw, 204, 'M', points, w_data)

#     # # int to word
#     # bw, points, w_data = trans.int2word([100, 200])
#     # PLC1.write(bw, 204, 'D', points, w_data)

#     PLC1.read('bit', 204, 'M', 2) # 그리퍼 상태 확인
    
#     PLC1.client.close() # 접속 종료