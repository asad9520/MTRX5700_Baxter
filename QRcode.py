import os
import matplotlib.pyplot as plt
import qrcode
from PIL import Image
from pyzbar import pyzbar
import cv2
import numpy as np
import imutils
from imutils.perspective import four_point_transform

def make_qr_code_easy(content, save_path=None):

    img = qrcode.make(data=content)
    if save_path:
        img.save(save_path)
    else:
        img.show()


def make_qr_code(content, save_path=None):

    qr_code_maker = qrcode.QRCode(version=2,
                                  error_correction=qrcode.constants.ERROR_CORRECT_M,
                                  box_size=8,
                                  border=4,
                                  )
    qr_code_maker.add_data(data=content)
    qr_code_maker.make(fit=True)
    img = qr_code_maker.make_image(fill_color="black", back_color="#23dda0")



    if save_path:
        img.save(save_path)
    else:
        img.show()


def make_qr_code_with_icon(content, icon_path, save_path=None):

    if not os.path.exists(icon_path):
        raise FileExistsError(icon_path)

    # First, generate an usual QR Code image
    qr_code_maker = qrcode.QRCode(version=4,
                                  error_correction=qrcode.constants.ERROR_CORRECT_H,
                                  box_size=8,
                                  border=1,
                                  )
    qr_code_maker.add_data(data=content)
    qr_code_maker.make(fit=True)
    qr_code_img = qr_code_maker.make_image(fill_color="black", back_color="white").convert('RGBA')


    # Second, load icon image and resize it
    icon_img = Image.open(icon_path)
    code_width, code_height = qr_code_img.size
    icon_img = icon_img.resize((code_width // 4, code_height // 4), Image.ANTIALIAS)

    # Last, add the icon to original QR Code
    qr_code_img.paste(icon_img, (code_width * 3 // 8, code_width * 3 // 8))

    if save_path:
        qr_code_img.save(save_path)
    else:
        qr_code_img.show()


def decode_qr_code(code_img_path):

    if not os.path.exists(code_img_path):
        raise FileExistsError(code_img_path)

    # Here, set only recognize QR Code and ignore other type of code
    return pyzbar.decode(Image.open(code_img_path), symbols=[pyzbar.ZBarSymbol.QRCODE])


if __name__ == "__main__":

    new = make_qr_code("Ayoan 23 Good", 'test.PNG')

    image = cv2.imread("674a59df92c935bc2812d15a9b4d5b7.jpg")

    scale_percent = 60  # percent of original size
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)

    dim = (width, height)

    image = cv2.resize(image, dim, interpolation = cv2.INTER_AREA)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (3, 3), 5)

    dilate = cv2.dilate(blurred, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
    edged = cv2.Canny(dilate, 30, 120, 3)


    cnts = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]  # 轮廓检测
    # cnts = cnts[0] if imutils.is_cv2() else cnts[1]  # 判断是opencv2还是opencv3

    docCnt = None

    if len(cnts) > 0:
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)  # 根据轮廓面积从大到小排序
        for c in cnts:
            peri = cv2.arcLength(c, True)  # 计算轮廓周长
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)  # 轮廓多边形拟合
            # 轮廓为4个点表示找到纸张
            if len(approx) == 4:
                docCnt = approx
                break

    for peak in docCnt:
        peak = peak[0]
        cv2.circle(image, tuple(peak), 10, (255, 0, 0))


    result_img = four_point_transform(image, docCnt.reshape(4, 2))

    results = pyzbar.decode(result_img, symbols=[pyzbar.ZBarSymbol.QRCODE])
    cv2.imshow('edge', dilate)


    cv2.imshow('image', image)

    cv2.imshow('result', result_img)
    if len(results):
        QRcode_information = results[0].data.decode("utf-8").split()
        # Name = QRcode_information[0]
        # ID = QRcode_information[1]
        # Health_con = QRcode_information[2]
        print(QRcode_information)
        print(results[0].data.decode("utf-8"))





    else:
        print("Can not recognize.")

    cv2.waitKey(0)



