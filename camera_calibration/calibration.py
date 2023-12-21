import numpy as np
import cv2 as cv
import glob
import pickle

# Configurações do tabuleiro de xadrez e do frame da câmera
chessboardSize = (8, 5)
frameSize = (640, 480)

# Critérios de término para otimização de algoritmos
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Preparando os pontos do objeto 3D com base no tamanho do tabuleiro de xadrez
objp = np.zeros((chessboardSize[0] * chessboardSize[1], 3), np.float32)
objp[:,:2] = np.mgrid[0:chessboardSize[0], 0:chessboardSize[1]].T.reshape(-1, 2)

# Definindo o tamanho real dos quadrados no tabuleiro (em milímetros)
size_of_chessboard_squares_mm = 30
objp = objp * size_of_chessboard_squares_mm

# Arrays para armazenar pontos do objeto e pontos da imagem de todas as imagens
objpoints = [] # Pontos 3d no espaço do mundo real
imgpoints = [] # Pontos 2d no plano da imagem

# Carregando todas as imagens com padrão de tabuleiro de xadrez
images = glob.glob('./images/*.png')

for image in images:
    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Encontrando os cantos do tabuleiro de xadrez
    ret, corners = cv.findChessboardCorners(gray, chessboardSize, None)

    # Se encontrado, adicionar pontos do objeto, pontos da imagem (depois de refinar)
    if ret == True:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners)

        # Desenhando e exibindo os cantos
        cv.drawChessboardCorners(img, chessboardSize, corners2, ret)
        cv.putText(img, image, (30, 60), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv.LINE_AA, False)
        cv.imshow('img', img)
        cv.waitKey(1000)

cv.destroyAllWindows()

# Calibração da câmera
ret, cameraMatrix, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, frameSize, None, None)

# Salvando o resultado da calibração para uso posterior
pickle.dump((cameraMatrix, dist), open("calibration.pkl", "wb"))
