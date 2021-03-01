# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file '3d_reconstructrionui_smaller.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(936, 630)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.tlLayout = QtWidgets.QVBoxLayout()
        self.tlLayout.setContentsMargins(0, -1, 1, -1)
        self.tlLayout.setObjectName("tlLayout")
        self.horizontalLayout_2.addLayout(self.tlLayout)
        self.mapLayout = QtWidgets.QVBoxLayout()
        self.mapLayout.setContentsMargins(1, -1, 0, -1)
        self.mapLayout.setObjectName("mapLayout")
        self.horizontalLayout_2.addLayout(self.mapLayout)
        self.verticalLayout_2.addLayout(self.horizontalLayout_2)
        self.playButton = QtWidgets.QPushButton(self.centralwidget)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playButton.setIcon(icon)
        self.playButton.setIconSize(QtCore.QSize(165,35))
        self.playButton.setObjectName("pushButton")
        # self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        # font = QtGui.QFont()
        # font.setPointSize(15)
        # font.setBold(True)
        # font.setWeight(75)
        # self.stopButton.setFont(font)
        # icon = QtGui.QIcon()
        # icon.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        # self.stopButton.setIcon(icon)
        # self.stopButton.setObjectName("stopButton")
        self.verticalLayout_2.addWidget(self.playButton)
        self.horizontalLayout_3.addLayout(self.verticalLayout_2)
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.verticalLayout.setObjectName("verticalLayout")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.imageLeft = QtWidgets.QLabel(self.centralwidget)
        self.imageLeft.setObjectName("imageLeft")
        self.gridLayout.addWidget(self.imageLeft, 2, 0, 1, 1)
        self.imageLeftFiltered = QtWidgets.QLabel(self.centralwidget)
        self.imageLeftFiltered.setText("")
        self.imageLeftFiltered.setObjectName("imageLeftFiltered")
        self.gridLayout.addWidget(self.imageLeftFiltered, 4, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.centralwidget)
        self.label.setObjectName("label")
        self.gridLayout.addWidget(self.label, 1, 0, 1, 1)
        self.imageRightFiltered = QtWidgets.QLabel(self.centralwidget)
        self.imageRightFiltered.setText("")
        self.imageRightFiltered.setObjectName("imageRightFiltered")
        self.gridLayout.addWidget(self.imageRightFiltered, 4, 1, 1, 1)
        self.imageRight = QtWidgets.QLabel(self.centralwidget)
        self.imageRight.setObjectName("imageRight")
        self.gridLayout.addWidget(self.imageRight, 2, 1, 1, 1)
        self.label_2 = QtWidgets.QLabel(self.centralwidget)
        self.label_2.setObjectName("label_2")
        self.gridLayout.addWidget(self.label_2, 3, 0, 1, 1)
        self.verticalLayout.addLayout(self.gridLayout)
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)
        self.horizontalLayout.setSpacing(2)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.runLayout = QtWidgets.QVBoxLayout()
        self.runLayout.setObjectName("runLayout")
        self.horizontalLayout.addLayout(self.runLayout)
        self.verticalLayout.addLayout(self.horizontalLayout)
        self.horizontalLayout_3.addLayout(self.verticalLayout)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 936, 19))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Obstacle Avoidance"))
        #self.stopButton.setText(_translate("MainWindow", "Stop Code"))
        self.imageLeft.setText(_translate("MainWindow", "TextLabel"))
        self.label.setText(_translate("MainWindow", "Input"))
        self.imageRight.setText(_translate("MainWindow", "TextLabel"))
        self.label_2.setText(_translate("MainWindow", "Processed images"))
        #self.playButton.setText(_translate("MainWindow", "Play Code"))

import resources_rc
