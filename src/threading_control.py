from ctrl_widow_x import WidowX
import serial
import time
import threading

wx = WidowX()  # Cria uma instância da classe WidowX

def send_flag(ser):
    try:
        ser.write(b'A')  # Envia o caractere 'A' para ativar a flag no Arduino
        print("Sent 'A'")
        time.sleep(1)
    except Exception as e:
        print(f"Error sending 'A': {e}")

def read_from_serial(ser, flag):
    try:
        while not flag['stop']:
            if ser.in_waiting > 0:
                try:
                    text = ser.readline().decode('utf-8').strip()
                    print(f"Received: {text}")
                    time.sleep(0.05)
                    if text == "65":  # Ignora o valor '65' que é o 'A' enviado
                        continue

                    values = text.split(',')
                    if len(values) == 4:
                        pressure_read = process_values(values)
                        if all(pr >= 90 for pr in pressure_read):
                            print("All sensors activated")
                            flag['stop'] = True

                except Exception as e:
                    print(f"Error reading: {e}")
            time.sleep(0.05)  # Pequeno atraso para não sobrecarregar a CPU
    except Exception as e:
        print(f"Unexpected error during read: {e}")
    finally:
        ser.close()

def proportion(x):
    # Função para converter um valor de 0 a 1023 para um valor de 0 a 100
    y = (x - 1024) * 100 / (-1024)  # Se x=1024, y=0; se x=0, y=100
    return y

def process_values(values):
    try:
        a0, a1, a2, a3 = map(int, values)
        threshold = 90  # Limiar de pressão para ativação do gripper
        levels = [proportion(a0), proportion(a1), proportion(a2), proportion(a3)]  # Converte os valores para a escala de 0 a 100
        print(f"Levels: {levels}")
        return levels
    except ValueError as e:
        print(f"Error processing values: {e}")
        return [0, 0, 0, 0]

def control_gripper(flag):
    gripper_value = 400  # Valor inicial para o gripper
    try:
        while not flag['stop']:
            wx.sendValue(gripper=gripper_value)
            gripper_value -= 20
            time.sleep(0.5)
            if gripper_value <= 20:
                print("Gripper value reached minimum")
                flag['stop'] = True
                wx.goHome()
                break
    except Exception as e:
        print(f"Error controlling gripper: {e}")

def main():
    wx.connect()  # Conecta ao robô
    port = '/dev/ttyUSB1'
    time.sleep(7)

    if wx.isConnected:
        try:
            ser = serial.Serial(port, 115200, timeout=2)
            print(ser)
            time.sleep(2)
            print(f"Connected to {port}")
            
            send_flag(ser)
            
            flag = {'stop': False}
            
            # Cria threads para leitura serial e controle do gripper
            read_thread = threading.Thread(target=read_from_serial, args=(ser, flag))
            control_thread = threading.Thread(target=control_gripper, args=(flag,))
            
            # Inicia as threads
            read_thread.start()
            control_thread.start()
            
            # Espera as threads terminarem
            read_thread.join()
            control_thread.join()
        
        except serial.SerialException as e:
            print(f"Serial exception: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
