from PyQt5.QtWidgets import QMainWindow, QApplication
from base_interface.mdrs_base_interface import BaseInterface
import sys
import qdarktheme


if __name__ == '__main__':
    qdarktheme.enable_hi_dpi()
    app = QApplication(sys.argv)
    qdarktheme.setup_theme()
    MainWindow = BaseInterface('./settings.yaml')
    MainWindow.position.x = 0
    MainWindow.position.y = -18
    MainWindow.show()
    app.exec_()