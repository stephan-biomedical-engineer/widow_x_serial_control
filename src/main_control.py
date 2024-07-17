#Posição "zero" = [255, 0, 0, 0, 200, 0, 200, 0, 0, 2, 0, 1, 0, 128, 0, 112, 124]
#Modo cilíndrico = [255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 207]
#Posição inicial = [255, 5, 10, 0, 127, 0, 202, 0, 90, 2, 0, 1, 0, 125, 0, 0, 205]

from ctrl_widow_x import WidowX as widow_x
import serial as ser

GO_SLEEP_CMD = [0xff, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x60, 0x9f]

def main():
    wx = widow_x()  # Cria uma instância da classe WidowX
    wx.connect()  # Conecta ao robô
    
    if wx.isConnected:
        i = 0  # Inicializa a variável de controle do loop
        try:
            while i < 1:
                gripper_position = gripper_value()  # Obtém o valor para o gripper
                wx.sendValue(gripper=gripper_position)  # Envia o comando para o robô
                if gripper_position == 0:
                    i = 1  # Encerra o loop se o usuário não desejar continuar
                    wx.comunicacaoSerial.write(GO_SLEEP_CMD)  # Envia o comando de inicialização
                else:
                    # Opcionalmente, você pode adicionar um delay aqui para evitar envio contínuo de comandos
                    pass
        except KeyboardInterrupt:
            wx.comunicacaoSerial.write(GO_SLEEP_CMD)  # Envia o comando de inicialização
            print("Programa encerrado pelo usuário.")
        finally:
            print("Fim do programa.")

def gripper_value():
    # Função para obter o valor do gripper do usuário
    try:
        value = int(input("Digite um valor para abrir ou fechar o gripper (posição padrão = 256), ou digite 0 para encerrar o programa: "))
        return value
    except ValueError:
        print("Valor inválido. Por favor, insira um número inteiro.")
        return gripper_value()  # Solicita novamente o valor

def position_xyz():
    # Função para obter as posições x, y e z do usuário
    try:
        x = int(input("Digite o valor de x: "))
        y = int(input("Digite o valor de y: "))
        z = int(input("Digite o valor de z: "))
        position = {
            'x': x,
            'y': y,
            'z': z,
            'gripper': gripper_value()  # Inclui o valor do gripper
        }
        return position
    except ValueError:
        print("Valor inválido. Por favor, insira números inteiros.")
        return position_xyz()  # Solicita novamente os valores

if __name__ == "__main__":
    main()
