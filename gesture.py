import cv2 as opencv
import numpy as np
import math
import serial
#arduino_serial_port = serial.Serial('/dev/ttyACM0', 9600)
capture_camera = opencv.VideoCapture(0)
count = 0
flag = [0,0,0,0,0]
while(capture_camera.isOpened()):
    #Inicia Captura de Camera
    ret, image = capture_camera.read()
    #Determina o Retangulo de Acao
    opencv.rectangle(image,(350,350),(100,100),(0,255,0),0)
    #Recorta o retangulo de Acao
    cropped_image = image[100:350, 100:350]
    #Tons de cinza
    grey = opencv.cvtColor(cropped_image, opencv.COLOR_BGR2GRAY)
    value = (35, 35)
    blurred = opencv.GaussianBlur(grey, value, 0)
    #ThreshHold (Gera imagem Binaria do retangulo de acao)
    _, thresh1 = opencv.threshold(blurred, 127, 255,
                               opencv.THRESH_BINARY_INV+opencv.THRESH_OTSU)
    #Calculo dos  contornos
    contours, hierarchy = opencv.findContours(thresh1.copy(),opencv.RETR_TREE, \
        opencv.CHAIN_APPROX_NONE)

    cnt = max(contours, key = lambda x: opencv.contourArea(x))
    
    x,y,w,h = opencv.boundingRect(cnt)
    #Aqui os pontos convexos da imagem sao capturados
    hull = opencv.convexHull(cnt)
    drawing = np.zeros(cropped_image.shape,np.uint8)
    #Setando returnPoints = False para achar os defects
    hull = opencv.convexHull(cnt,returnPoints = False)
    #Calculando os Defects a partir da imagem de Hull gerada
    defects = opencv.convexityDefects(cnt,hull)
    #Setando o contador de Defects como 0 
    count_defects = 0
    #Iniciando o calculo de Defects
    for i in range(defects.shape[0]):
        s,e,f,d = defects[i,0]
        start = tuple(cnt[s][0])
        end = tuple(cnt[e][0])
        far = tuple(cnt[f][0])
        #Inicio para calculo dos angulos de abertura dos dedos
        a = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        b = math.sqrt((far[0] - start[0])**2 + (far[1] - start[1])**2)
        c = math.sqrt((end[0] - far[0])**2 + (end[1] - far[1])**2)
        angle = math.acos((b**2 + c**2 - a**2)/(2*b*c)) * 57
        #Se o angulo gerado entre os dedos for menor que o estabelecido, consideremos um defect valido
        #Evita erros!!!
        if angle <= 70:
            count_defects += 1
            #Coloca um ponto no defect
            opencv.circle(cropped_image,far,1,[0,0,255],2)
        #Desenha a regiao de acao da mao
        opencv.line(cropped_image,start,end,[0,0,0],3) 
    #Inicio dos calculos de acao do arduino
    #De acordo com cada flag, uma acao sera tomada e enviaremos um codigo ao arduino
    #Este codigo e prefinido na programacao do Arduino.
    #Flags sao usadas para o sistema entrar no if apenas 1 vez e nao causar Buffer na porta serial do Arduino
    if count_defects == 1 and flag[0]==0:
        flag = [1,0,0,0,0]
        #Envia 70 graus para o pitch
        #arduino_serial_port.write('70p')
    elif count_defects == 2 and flag[1]==0:
        flag = [0,1,0,0,0]
        #Envia 120 para o pitch
        #arduino_serial_port.write('120p')
    elif count_defects == 3 and flag[2]==0:
        flag = [0,0,1,0,0]
        #Envia 120 graus para o roll
        #arduino_serial_port.write('120r')
    elif count_defects == 4 and flag[3]==0:
        flag = [0,0,0,1,0]
        #Envia 70 graus para o roll
        #arduino_serial_port.write('70r')
    elif count_defects == 0 and flag[4]==0:
        flag = [0,0,0,0,1]
        #Eniva 90 graus para o pitch e para o roll
        #arduino_serial_port.write('90p90r')
    #Impressao simples na tela da acao tomada
    if flag[0] == 1:
        opencv.putText(image, "CAMERA UP", (5,50), opencv.FONT_HERSHEY_SIMPLEX, 1, 2)
    elif flag[1] == 1:
        opencv.putText(image, "CAMERA DOWN", (5,50), opencv.FONT_HERSHEY_SIMPLEX, 1, 2)
    elif flag[2] == 1:
        opencv.putText(image,"CAMERA LEFT", (50,50), opencv.FONT_HERSHEY_SIMPLEX, 2, 2)
    elif flag[3] == 1:
        opencv.putText(image,"CAMERA RIGHT", (50,50), opencv.FONT_HERSHEY_SIMPLEX, 2, 2)
    elif flag[4] == 1:
        opencv.putText(image,"CAMERA CENTER", (50,50),opencv.FONT_HERSHEY_SIMPLEX, 2, 2)
    #Exibicao das imagens
    opencv.imshow('Gesture', image)
    #Exibindo o cropped
    opencv.imshow("Cropped", cropped_image)
    #Exibindo o ThreshHold
    opencv.imshow('Thresholded', thresh1)
    k = opencv.waitKey(10)
    if k == 27:
        break
