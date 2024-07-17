from ctrl_widow_x import WidowX
import serial
import time

wx = WidowX()  # Cria uma instância da classe WidowX
GO_SLEEP_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x60, 0x9f]

def send_flag(ser):
    try:
        ser.write(b'A')  # Envia o caractere 'A' para ativar a flag no Arduino
        print("Sent 'A'")
        time.sleep(1)

    except Exception as e:
        print(f"Error sending 'A': {e}")


def read_from_serial(ser):
    gripper_value = 400  # Valor inicial para o gripper
    holding = False

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
                        if holding == False:
                            wx.sendValue(gripper=gripper_value)
                            gripper_value -= 40
                            time.sleep(0.5)
                    except:
                        print("Error sending gripper value")

                    values = text.split(',')
                    if len(values) == 4:
                        pressure_read = process_values(values)

                        if pressure_read > 10:
                            print("Already holding")
                            holding = True
                            
                        if pressure_read < 10 and holding == True:
                            print("Releasing")
                            holding = False
                            gripper_value = 400

                    if gripper_value == 0:
                        print("Gripper value reached 0")
                        holding = True

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
    except KeyboardInterrupt:
                    wx.comunicacaoSerial.write(GO_SLEEP_CMD)  # Envia o comando de inicialização
                    print("Programa encerrado pelo usuário.")
                    
    except Exception as e:
        print(f"Unexpected error during read: {e}")
    finally:
        ser.close()


def proportion(x):
    # Função para converter um valor de 0 a 1023 para um valor de 0% a 100%
    y = (x - 1023) * 100 / (-1023) #Se x=1023, y=0%; se x=0, y=100%
    return y


def process_values(values):
    try:
        a0, a1, a2, a3 = map(int, values)
          # Limiar de pressão para ativação do gripper
        levels = [proportion(a0), proportion(a1), proportion(a2), proportion(a3)] # Converte os valores para a escala de 0 a 100
        mean = sum(levels) / len(levels)  # Calcula a média dos valores
        print(f"Levels: {levels}, Mean: {mean}")
        return mean

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
