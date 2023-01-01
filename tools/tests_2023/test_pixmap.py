from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QLabel, QLineEdit, QGridLayout, QFrame, QPushButton, QSpinBox, QSizePolicy
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QTabWidget, QGridLayout, QLabel
from PyQt5.QtGui import QImage, QImageReader, QPixmap, qRgb
import sys

app = QApplication(sys.argv)

fi=open("test_bitmap.ppm","r+b")
#fo=open("out.ppm","w+b")

my_buff=fi.read()
test_img_pixmap = QPixmap()
test_img_pixmap.loadFromData(my_buff)

test_img = test_img_pixmap.toImage()

for x in range(20,80):
    for y in range(55,77):
        v = qRgb(0, 0, 250)
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
