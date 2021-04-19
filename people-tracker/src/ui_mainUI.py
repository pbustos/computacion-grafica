# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 5.15.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *


class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(1402, 633)
        self.verticalLayout = QVBoxLayout(guiDlg)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.horizontalLayout = QHBoxLayout()
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.label_img_left = QLabel(guiDlg)
        self.label_img_left.setObjectName(u"label_img_left")
        self.label_img_left.setMinimumSize(QSize(640, 480))

        self.horizontalLayout.addWidget(self.label_img_left)

        self.label_img_right = QLabel(guiDlg)
        self.label_img_right.setObjectName(u"label_img_right")
        self.label_img_right.setMinimumSize(QSize(640, 480))

        self.horizontalLayout.addWidget(self.label_img_right)


        self.verticalLayout.addLayout(self.horizontalLayout)


        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"ComponentSM", None))
        self.label_img_left.setText(QCoreApplication.translate("guiDlg", u"TextLabel", None))
        self.label_img_right.setText(QCoreApplication.translate("guiDlg", u"TextLabel", None))
    # retranslateUi

