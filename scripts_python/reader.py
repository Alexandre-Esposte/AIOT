import serial
from datetime import datetime
import struct
import pandas as pd

nome = input('Entre com o nome: ')+'.csv'

dados_dict={"time":[],'ax':[],'ay':[],'az':[],'gx':[],'gy':[],'gz':[]}


while(True):
    try:
        ser = serial.Serial("/dev/ttyUSB0",115200)
        var = ser.read(24)
        dados = struct.unpack('ffffff', var)
        ax=dados[0]
        ay=dados[1]
        az=dados[2]
        gx=dados[3]
        gy=dados[4]
        gz=dados[5]
        time= datetime.now()

        dados_dict['time'].append(time)
        dados_dict['ax'].append(ax)
        dados_dict['ay'].append(ay)
        dados_dict['az'].append(az)
        dados_dict['gx'].append(gx)
        dados_dict['gy'].append(gy)
        dados_dict['gz'].append(gz)
        
    except:
        print('Salvando os dados')
        pd.DataFrame(dados_dict,index=None).to_csv(nome)
        break



      
print('Finalizando programa')
