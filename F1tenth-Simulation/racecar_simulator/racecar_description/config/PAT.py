#!/usr/bin/python


# GUI to adjust YAML config file that includes parameters of the robot sensors to be simulated using Gazebo
# to get default parameters, restart PAT and update YAML
import sys
from PyQt4.QtCore import *
from PyQt4.QtGui import *
import yaml
 

IMU = {"alwaysOn" : "true", "gaussian_noise": 0.0, "update_rate" : 20.0}

Lidar = {"visualize": "True", "update_rate" : 40.0 , "samples" : 1080, "resolution" : 1, 
         "min_angle" : -2.3561944902, "max_angle" : 2.3561944902, "range_min" : 0.06,
         "range_max" : 10.0 , "range_resolution" : 0.01 , "noise_mean" : 0.0 , "noise_stddev" : 0.03}
         
Zed = {"update_rate" : 30.0, "image_w" : 640, "image_h" : 480, "clip_near" : 0.02,
       "clip_far" : 300 , "noise_mean" : 0.0 , "noise_stddev" : 0.007, "cam_controller_update_rate" : 30.0}


class Window(QMainWindow):

    def __init__(self):
        super(Window, self).__init__()
        self.setGeometry(50, 50, 1000, 1000)
        self.setWindowTitle("Parameter Adjustment Tool")
        self.setWindowIcon(QIcon('logo.png'))

        self.home()

    def home(self):
        
        '''titles'''
        titleLidar = QLabel(self)       
        titleLidar.move(50, 5)
        titleLidar.setText("Lidar")
        
        titleIMU = QLabel(self)       
        titleIMU.move(400, 5)
        titleIMU.setText("IMU")
        
        titleZed = QLabel(self)       
        titleZed.move(700, 5)
        titleZed.setText("Zed")
        
        '''Update yaml button'''
        btn = QPushButton("Update YAML", self)
        btn.clicked.connect(self.update_yaml)
        btn.resize(btn.minimumSizeHint())
        btn.move(0,550)
      
        
        '''IMU params'''
        
        #### always on #######################################
        title00 = QLabel(self)       
        title00.move(300, 25)
        title00.setText("always on : ")
        
        checkBox01 = QCheckBox(self)
        checkBox01.move(400, 25)
        checkBox01.stateChanged.connect(self.imu_always_on)
        checkBox01.toggle() # default is selected

        #### gaussian_noise #######################################

        
        title02 = QLabel(self)       
        title02.move(300, 50)
        title02.setText("gauss noise : ")
                
        self.e01 = QLineEdit(self)        
        self.e01.setValidator(QDoubleValidator())
        self.e01.move(420, 50)
        self.e01.editingFinished.connect(self.imu_gaussian_noise_update)
        
        #### update rate #######################################
        title1 = QLabel(self)       
        title1.move(300, 100)
        title1.setText("update rate : ")
        
        self.e02 = QLineEdit(self)        
        self.e02.setValidator(QDoubleValidator())
        self.e02.move(420, 100)
        self.e02.editingFinished.connect(self.imu_update_rate)       
        

        '''Lidar params'''
  
        #### visualize #######################################
        title0 = QLabel(self)       
        title0.move(5, 25)
        title0.setText("Visualize : ")
        
        checkBox1 = QCheckBox(self)
        checkBox1.move(120, 25)
        checkBox1.stateChanged.connect(self.lidar_visualize)
        checkBox1.toggle() # default is selected
        
        #### update rate #######################################
        title1 = QLabel(self)       
        title1.move(5, 50)
        title1.setText("update rate : ")
       
        
        sld1 = QSlider(Qt.Horizontal, self)
        sld1.setFocusPolicy(Qt.NoFocus)
        sld1.setMinimum(0)
        sld1.setMaximum(50)
        sld1.setValue(25)
        sld1.move(120, 50)
        sld1.valueChanged[int].connect(self.lidar_update_rate)
        
                
        self.label1 = QLabel(self)
        self.label1.move(220, 50)
        self.label1.setText(str(sld1.value()))
        ###### min_angle ########################################################
        
        title2 = QLabel(self)       
        title2.move(5, 100)
        title2.setText("min_angle : ")
                
        self.e1 = QLineEdit(self)        
        self.e1.setValidator(QDoubleValidator())
        self.e1.move(120, 100)
        self.e1.editingFinished.connect(self.min_angle_update)
        
        ###### max_angle ########################################################
        
        title3 = QLabel(self)       
        title3.move(5, 150)
        title3.setText("max_angle : ")
                
        self.e2 = QLineEdit(self)        
        self.e2.setValidator(QDoubleValidator())
        self.e2.move(120, 150)
        self.e2.editingFinished.connect(self.max_angle_update)
        
        ###### samples ########################################################
        
        title4 = QLabel(self)       
        title4.move(5, 200)
        title4.setText("samples : ")
                
        self.e3 = QLineEdit(self)        
        self.e3.setValidator(QDoubleValidator())
        self.e3.move(120, 200)
        self.e3.editingFinished.connect(self.samples_update)

        ###### resolution ########################################################
        
        title5 = QLabel(self)       
        title5.move(5, 250)
        title5.setText("resolution : ")
                
        self.e4 = QLineEdit(self)        
        self.e4.setValidator(QIntValidator())
        self.e4.move(120, 250)
        self.e4.editingFinished.connect(self.resolution_update)
        
        ###### range_min ########################################################
        
        title6 = QLabel(self)       
        title6.move(5, 300)
        title6.setText("range_min : ")
                
        self.e5 = QLineEdit(self)        
        self.e5.setValidator(QDoubleValidator())
        self.e5.move(120, 300)
        self.e5.editingFinished.connect(self.range_min_update)
        
        ###### range_max ########################################################
        
        title7 = QLabel(self)       
        title7.move(5, 350)
        title7.setText("range_max : ")
                
        self.e6 = QLineEdit(self)        
        self.e6.setValidator(QDoubleValidator())
        self.e6.move(120, 350)
        self.e6.editingFinished.connect(self.range_max_update)
        
        ###### range_resolution ########################################################
        
        title8 = QLabel(self)       
        title8.move(5, 400)
        title8.setText("range_res : ")
                
        self.e7 = QLineEdit(self)        
        self.e7.setValidator(QDoubleValidator())
        self.e7.move(120, 400)
        self.e7.editingFinished.connect(self.range_resolution_update)
    
        ###### noise_mean ########################################################
        
        title9 = QLabel(self)       
        title9.move(5, 450)
        title9.setText("noise_mean : ")
                
        self.e8 = QLineEdit(self)        
        self.e8.setValidator(QDoubleValidator())
        self.e8.move(120, 450)
        self.e8.editingFinished.connect(self.noise_mean_update)
        
        ###### noise_stddev ########################################################
        
             
        title10 = QLabel(self)       
        title10.move(5, 500)
        title10.setText("noise_stddev : ")
                
        self.e9 = QLineEdit(self)        
        self.e9.setValidator(QDoubleValidator())
        self.e9.move(120, 500)
        self.e9.editingFinished.connect(self.noise_stddev_update)
        
        '''Zed params'''
        #### update_rate #######################################
        title11 = QLabel(self)       
        title11.move(700, 50)
        title11.setText("update_rate : ")
        
        self.e10 = QLineEdit(self)        
        self.e10.setValidator(QDoubleValidator())
        self.e10.move(800, 50)
        self.e10.editingFinished.connect(self.zed_update_rate)
        
        #### image_w #######################################
        title12 = QLabel(self)       
        title12.move(700, 100)
        title12.setText("img width : ")
        
        self.e11 = QLineEdit(self)        
        self.e11.setValidator(QIntValidator())
        self.e11.move(800, 100)
        self.e11.editingFinished.connect(self.zed_image_w)        
        
        #### image_h #######################################
        title13 = QLabel(self)       
        title13.move(700, 150)
        title13.setText("img height : ")
        
        self.e12 = QLineEdit(self)        
        self.e12.setValidator(QIntValidator())
        self.e12.move(800, 150)
        self.e12.editingFinished.connect(self.zed_image_h)         
        
        #### clip_near #######################################
        title14 = QLabel(self)       
        title14.move(700, 200)
        title14.setText("clip near : ")
        
        self.e13 = QLineEdit(self)        
        self.e13.setValidator(QDoubleValidator())
        self.e13.move(800, 200)
        self.e13.editingFinished.connect(self.zed_clip_near)       
        
        #### clip_far #######################################
        title15 = QLabel(self)       
        title15.move(700, 250)
        title15.setText("clip far : ")
        
        self.e14 = QLineEdit(self)        
        self.e14.setValidator(QDoubleValidator())
        self.e14.move(800, 250)
        self.e14.editingFinished.connect(self.zed_clip_far)          
        
        #### noise mean #######################################
        title16 = QLabel(self)       
        title16.move(700, 300)
        title16.setText("noise mean : ")
        
        self.e15 = QLineEdit(self)        
        self.e15.setValidator(QDoubleValidator())
        self.e15.move(800, 300)
        self.e15.editingFinished.connect(self.zed_noise_mean)           
        
        #### noise stddev #######################################
        title17 = QLabel(self)       
        title17.move(700, 350)
        title17.setText("noise stddev : ")
        
        self.e16 = QLineEdit(self)        
        self.e16.setValidator(QDoubleValidator())
        self.e16.move(800, 350)
        self.e16.editingFinished.connect(self.zed_noise_stddev)           
        
        #### cam_controller_update_rate #######################################
        title18 = QLabel(self)       
        title18.move(700, 400)
        title18.setText("cntlr upd rate : ")
        
        self.e17 = QLineEdit(self)        
        self.e17.setValidator(QDoubleValidator())
        self.e17.move(800, 400)
        self.e17.editingFinished.connect(self.zed_controller_update_rate)         
        
       
        
        ####################
        self.show()
        
        
    ##################### IMU functions
    def imu_always_on(self, state):
        if state == Qt.Checked:
            IMU["alwaysOn"] = "False"
        else:
            IMU["alwaysOn"] = "True"
    
    def imu_gaussian_noise_update(self):
        IMU["gaussian_noise"] = float(self.e01.text())
        
    def imu_update_rate(self):
        IMU["update_rate"] = float(self.e02.text())


    ##################### Lidar functions
    def lidar_visualize(self, state):
        if state == Qt.Checked:
            Lidar["visualize"] = "False"
        else:
            Lidar["visualize"] = "True"
            
    def lidar_update_rate(self, value):
        
        self.label1.setText(str(value))
        self.label1.setAlignment(Qt.AlignCenter)
        Lidar["update_rate"] = value
        
    def min_angle_update(self):
        Lidar["min_angle"] = float(self.e1.text())
            
    def max_angle_update(self):
        Lidar["max_angle"] = float(self.e2.text())
        
    def samples_update(self):
        Lidar["samples"] = float(self.e3.text())
        
    def resolution_update(self):
        Lidar["resolution"] = int(self.e4.text())

    def range_min_update(self):
        Lidar["range_min"] = float(self.e5.text())

    def range_max_update(self):
        Lidar["range_max"] = float(self.e6.text())
        
    def range_resolution_update(self):
        Lidar["range_resolution"] = float(self.e7.text())
        
    def noise_mean_update(self):
        Lidar["noise_mean"] = float(self.e8.text())
        
    def noise_stddev_update(self):
        Lidar["noise_stddev"] = float(self.e9.text())
        
        
    ##################### Zed functions
        
    def zed_update_rate(self):
        Zed["update_rate"] = float(self.e10.text())
        
    def zed_image_w(self):
        Zed["image_w"] = int(self.e11.text())    
        
    def zed_image_h(self):
        Zed["image_h"] = int(self.e12.text())           
        
    def zed_clip_near(self):
        Zed["clip_near"] = float(self.e13.text())     

    def zed_clip_far(self):
        Zed["clip_far"] = float(self.e14.text())  

    def zed_noise_mean(self):
        Zed["noise_mean"] = float(self.e15.text())  

    def zed_noise_stddev(self):
        Zed["noise_stddev"] = float(self.e16.text())          
        
    def zed_controller_update_rate(self):
        Zed["cam_controller_update_rate"] = float(self.e17.text())          
        
    ############################update YAML file

    def update_yaml(self):
        Parameters = {"IMU": IMU, "Lidar": Lidar, "Zed": Zed }

        #update yaml file
        filepath = "parameters.yaml"
        with open(filepath, "w") as file_descriptor:
            yaml.dump(Parameters, file_descriptor)
        
        #print yaml file contents
        with open(filepath, "r") as file_descriptor:
            data = yaml.load(file_descriptor)
        print data['Lidar']['visualize']
    
def run():
    app = QApplication(sys.argv)
    GUI = Window()
    sys.exit(app.exec_())



run()