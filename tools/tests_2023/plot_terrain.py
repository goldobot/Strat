from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QFrame, QPushButton, QSpinBox, QSizePolicy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QTabWidget, QGridLayout, QLabel
from PyQt5.QtGui import QImage, QImageReader, QPixmap, qRgb
import sys
import math

IM_W = 300
IM_H = 200
PIX_SZ = 0.01

def pix_to_real_m(px, py):
    rx = py*PIX_SZ
    ry = (px - IM_W/2)*PIX_SZ
    return (rx, ry)

def real_m_to_pix(rx, ry):
    px = ry/PIX_SZ + IM_W/2
    py = rx/PIX_SZ
    return (px, py)

class GoldoPos:
    def __init__(self,vi=None):
        if vi==None:
            self.x = 0
            self.y = 0
        else:
            self.x = vi[0]
            self.y = vi[1]
    def from_pix(self,px, py):
        self.x, self.y = pix_to_real_m(px, py)

def real_dist(p1, p2):
    dx = p1.x - p2.x
    dy = p1.y - p2.y
    return math.sqrt(dx*dx + dy*dy)

app = QApplication(sys.argv)

test_img = QImage(IM_W, IM_H, QImage.Format_RGB32)

Pcurr = GoldoPos()
P0 = GoldoPos(( 0.0, 0.0))
P1 = GoldoPos(( 2.0,-1.5))
P2 = GoldoPos(( 2.0, 1.5))
for x in range(0,IM_W):
    for y in range(0,IM_H):
        v = qRgb(255, 255, 255)

        Pcurr.from_pix(x,y)

        D0 = real_dist(Pcurr, P0)
        D1 = real_dist(Pcurr, P1)
        D2 = real_dist(Pcurr, P2)

        if (D0<D1) and (D0<D2):
            v = qRgb(250, 0, 0)
        elif (D1<D0) and (D1<D2):
            v = qRgb(0, 250, 0)
        elif (D2<D0) and (D2<D1):
            v = qRgb(0, 0, 250)

        #if (D0<0.3):
        #    v = qRgb(250,     0,   0)
        #elif (D1<0.3):
        #    v = qRgb(  0,   250,   0)
        #elif (D2<0.3):
        #    v = qRgb(  0,     0, 250)

        test_img.setPixel(x,y,v)

test_img_pixmap2 = QPixmap.fromImage(test_img)

test_img_pixmap2.save("out.ppm")

test_img_label = QLabel()
test_img_label.setPixmap(test_img_pixmap2)
layout = QtWidgets.QGridLayout()
layout.addWidget(test_img_label)
_main_widget = QWidget()
_main_widget.setLayout(layout)
main_window = QMainWindow()
main_window.setCentralWidget(_main_widget)
main_window.show()

sys.exit(app.exec_())
