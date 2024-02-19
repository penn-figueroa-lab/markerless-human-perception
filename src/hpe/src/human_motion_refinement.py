import rospy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

import numpy as np
import os
from numpy.linalg import inv
import json
import sys
sys.path.append('/usr/local/python')
import time


# -----------------------------------------------------------------------
# Class Completor
# -----------------------------------------------------------------------

import numpy as np
import tensorflow as tf
from tensorflow import keras
from Skeleton import Skeleton,ConstrainedSkeleton

# limit the GPU usage 
gpus = tf.config.experimental.list_physical_devices('GPU')
for gpu in gpus:
  tf.config.experimental.set_memory_growth(gpu, True)

class VAE(tf.keras.Model):
    def __init__(self,dim):
        super(VAE, self).__init__()
        dropout_rate = 0.9
        latent_dim = 20
        self.input_layer = tf.keras.layers.InputLayer(input_shape=12 * dim)

        self.encoder1 = tf.keras.layers.Dense(64, activation='relu')

        self.encoder2 = tf.keras.layers.Dense(128, activation='relu')

        # Latent space parameters
        self.z_mean = tf.keras.layers.Dense(latent_dim)
        self.z_log_var = tf.keras.layers.Dense(latent_dim)


        self.decoder1 = tf.keras.layers.Dense(128, activation='relu')
   
        self.decoder2 = tf.keras.layers.Dense(64, activation='relu')
        self.decoder3 = tf.keras.layers.Dense(12 * dim, activation='linear')

    def build(self, input_shape):
        # Definisci i pesi dei layer in base alle dimensioni dell'input
        super(VAE, self).build(input_shape)

    def reparameterize(self, z_mean, z_log_var):
        epsilon = tf.random.normal(shape=tf.shape(z_mean))
        return z_mean + tf.exp(0.5 * z_log_var) * epsilon
    
    def call(self, inputs):
        inputs = tf.where(tf.math.is_nan(inputs), -1 * tf.ones_like(inputs), inputs)
        x = self.input_layer(inputs)
        x = self.encoder1(x)
        #x = self.dropout1(x)
        x = self.encoder2(x)

        z_mean = self.z_mean(x)
        z_log_var = self.z_log_var(x)
        z = self.reparameterize(z_mean, z_log_var)

        #x = self.dropout2(x)
        z = self.decoder1(z)
        z = self.decoder2(z)
        z = self.decoder3(z)
    
        return z


def vae_loss(y_true, y_pred):
    return None

def custom_mse_loss():
    return None


class Completor():
    
    def __init__(self, path: str,dim):
        
        try:
            self.model = keras.models.load_model(path, compile=False)
        except:
            print("maybe VAE?")
            self.model = VAE(dim)
            self.model.build(input_shape=(1, 12 * dim))
            opt = keras.optimizers.Adam(learning_rate=1e-3)
            self.model.compile(optimizer=opt, loss=vae_loss, metrics=["mae"])
            self.model.load_weights(path)

    def minmax_norm(self,skeleton):
        self.in_max = np.nanmax(skeleton)
        self.in_min = np.nanmin(skeleton)
        skeleton_norm = (skeleton.copy()-self.in_min)/(self.in_max-self.in_min)
        skeleton_norm = skeleton_norm.reshape((1, -1)) # maybe 12*dim

    def minmax_denorm(self,filtered_skeleton):
        return filtered_skeleton * (self.in_max-self.in_min) + self.in_min

    def bio_norm(self,skeleton):
        s12 = Skeleton('/home/rmhri/markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY12.xml')
        s15 = ConstrainedSkeleton('/home/rmhri/markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY15_constrained_3D.xml')
        labels = ['LShoulder','RShoulder','LElbow','RElbow','LWrist','RWrist','LHip','RHip','LKnee','RKnee','LAnkle','RAnkle']
        x = skeleton.reshape(-1,3)
        s12.load_from_numpy(x,labels)
        s15.load_from_BODY12(s12)
        self.h = s15.estimate_height()
        self.root = s15.to_numpy(["Root"],dim=3)
        if np.any(np.isnan(self.root)):
            self.root=np.nanmean(x,0)
        x = (x-self.root)/self.h
        return x.reshape((1, -1))

    def bio_denorm(self,filtered_skeleton):
        filtered_skeleton = filtered_skeleton.reshape((-1,3)) * self.h + self.root
        return filtered_skeleton.reshape((1, -1))

    def bio_norm_onehot(self,skeleton):
        s12 = Skeleton('/home/rmhri/markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY12.xml')
        s15 = ConstrainedSkeleton('/home/rmhri/markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY15_constrained_3D.xml')
        labels = ['LShoulder','RShoulder','LElbow','RElbow','LWrist','RWrist','LHip','RHip','LKnee','RKnee','LAnkle','RAnkle']
        x = skeleton.reshape(-1,3)
        s12.load_from_numpy(x,labels)
        s15.load_from_BODY12(s12)
        self.h = s15.estimate_height()
        self.root = s15.to_numpy(["Root"],dim=3)
        if np.any(np.isnan(self.root)):
            self.root=np.nanmean(x,0)
        x = (x-self.root)/self.h
        x = np.insert(x, 3, values=np.where(np.isnan(x[:,0]), 1, 0), axis=1)
        return x.reshape((1, -1))

    def __call__(self,skeleton):
        filtered_skeleton = self.model(self.bio_norm_onehot(skeleton)).numpy()
        # filtered_skeleton = self.model(self.bio_norm(skeleton)).numpy()
        filtered_skeleton = self.bio_denorm(filtered_skeleton)
        filled = np.where(np.isnan(skeleton),filtered_skeleton[0,:],skeleton)    
        return filled
    
# -----------------------------------------------------------------------







parts_15 = ["Root","MidShoulder","LShoulder","LElbow","LWrist","RShoulder",\
            "RElbow","RWrist","MidHip","LHip","LKnee","LAnkle","RHip","RKnee","RAnkle"]

parts_25 = ["Nose","Neck", "RShoulder", "RElbow", "RWrist", "LShoulder", "LElbow","LWrist", "MidHip",\
            "RHip", "RKnee", "RAnkle", "LHip", "LKnee", "LAnkle", "REye", "LEye", "REar", "LEar", "LBigToe",\
            "LSmallToe", "LHeel", "RBigToe", "RSmallToe", "RHeel", "Background"]

labels =  ['LShoulder','RShoulder','LElbow','RElbow','LWrist','RWrist','LHip','RHip','LKnee','RKnee','LAnkle','RAnkle']
def callback(pose):
    pose = json.loads(pose.data)
    refined_poses = {}
    refined_poses["timestamp"] = pose["timestamp"]
    refined_poses["value"] = []
    
    # For each person
    for p in range(len(pose["value"])):
        x = []
        for kp in labels:
            if pose["value"][p][kp]["acc"] > 0.2:
                x.append(pose["value"][p][kp]["x"])
                x.append(pose["value"][p][kp]["y"])
                x.append(pose["value"][p][kp]["z"])
            else:
                x.append(np.nan)
                x.append(np.nan)
                x.append(np.nan)
        x = np.array(x)
        missings = (len(x)-np.count_nonzero(np.isnan(x))) /len(x)
        
        # You have to see at least 50% of the keypoints to complete the others!
        if completor is not None and missings < 0.5:
            y = completor(x)
            # If there are still gaps skip the person
            if np.any(np.isnan(y)):
                continue
            s12 = Skeleton('/home/rmhri/markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY12.xml')
            s15 = ConstrainedSkeleton('/home/rmhri/markerless-human-perception/src/hpe/src/Biomechanical-Model/BODY15_constrained_3D.xml')
            y = y.reshape(-1,3)
            s12.load_from_numpy(y,labels)
            s15.load_from_BODY12(s12)
            s = s15.to_numpy(parts_15)
            s_dict = {}
            for i,label in enumerate(parts_15):
                s_dict[label] = {'x':s[i,0],'y':s[i,1],'z':s[i,2]}
            refined_poses["value"].append(s_dict)
    
    if refined_poses["value"]:
        out_msg = json.dumps(refined_poses)
        pub.publish(out_msg)
        
        
completor = None 
def main():
    global pub, completor
    rospy.init_node("human_pose_refinement")  
    # Initiate listener
    rospy.Subscriber("hpe/poses", String, callback)
    
    # Setup publisher
    pub = rospy.Publisher('hpe/refined_poses', String, queue_size=10)
    completor = Completor("/home/rmhri/markerless-human-perception/src/hpe/src/MLP_bio_h36m_cameraview_3D_absolute_onehot",3)
    
    # Synchronize
    rospy.spin()

if __name__ == main():
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.logwarn("Node Not Executed")
