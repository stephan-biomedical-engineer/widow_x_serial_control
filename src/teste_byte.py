from src.ctrl_widow_x import WidowX
import serial
import time

wx = WidowX()  # Cria uma instância da classe WidowX

def send_flag(ser):
    try:
        ser.write(b'A')  # Envia o caractere 'A' para ativar a flag no Arduino
        print("Sent 'A'")
        time.sleep(1)

    except Exception as e:
        print(f"Error sending 'A': {e}")


def read_from_serial(ser):
    gripper_value = 400  # Valor inicial para o gripper

    try:
        while True:
            if ser.in_waiting > 0:
                try:
                    text = ser.readline().decode('utf-8').strip()
                    print(f"Received: {text}")
                    time.sleep(0.05)
                    if text == "65":  # Ignora o valor '65' que é o 'A' enviado
                        continue
                    
                    try:
                        wx.sendValue(gripper=gripper_value)
                        gripper_value -= 20
                        time.sleep(0.5)
                    except:
                        print("Error sending gripper value")

                    values = text.split(',')
                    if len(values) == 4:
                        pressure_read = process_values(values)

                        if all(pr >= 90 for pr in pressure_read):
                            print("All sensors activated")
                            break

                    if gripper_value == 20:
                        print("Gripper value reached 20")
                        break

                    else:
                        print(f"Unexpected data format: {text}")

                except Exception as e:
                    print(f"Error reading: {e}")
                    break
                
            time.sleep(0.05)  # Pequeno atraso para não sobrecarregar a CPU

        # response = input("Pressione Enter")
        # wx.goHome()
        # ser.close()
        # wx.disconnect()

    except Exception as e:
        print(f"Unexpected error during read: {e}")
    finally:
        ser.close()


def proportion(x):
    # Função para converter um valor de 0 a 1023 para um valor de 0 a 100
    y = (x - 1024) * 100 / (-1024) #Se x=1024, y=0; se x=0, y=100
    return y


def process_values(values):
    try:
        a0, a1, a2, a3 = map(int, values)
        threshold = 90  # Limiar de pressão para ativação do gripper
        levels = [proportion(a0), proportion(a1), proportion(a2), proportion(a3)] # Converte os valores para a escala de 0 a 100
        print(f"Levels: {levels}")
        return levels

    except ValueError as e:
        print(f"Error processing values: {e}")
        return [0, 0, 0, 0]



def main():
    wx.connect()  # Conecta ao robô
    port = '/dev/ttyUSB1'

    if wx.isConnected:
        try:
            ser = serial.Serial(port, 115200, timeout=2)
            time.sleep(2)
            print(f"Connected to {port}")
            
            send_flag(ser)
            
            read_from_serial(ser)
        
        except serial.SerialException as e:
            print(f"Serial exception: {e}")
        except Exception as e:
            print(f"Unexpected error: {e}")

if __name__ == "__main__":
    main()
