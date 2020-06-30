import numpy as np
import sys
import os
sys.argv = ['pdm']
os.environ['TF_CPP_MIN_LOG_LEVEL']='2'
import tensorflow as tf
import rospkg


def evaluate_singleString(pic):  
	rospack = rospkg.RosPack() 		   
	with tf.gfile.FastGFile(rospack.get_path('manipulation_vision')+"/src/tf_files/retrained_graph.pb", 'rb') as f:
	    graph_def = tf.GraphDef()	## The graph-graph_def is a saved copy of a TensorFlow graph; objektinitialisierung
	    graph_def.ParseFromString(f.read())	#Parse serialized protocol buffer data into variable
	    _ = tf.import_graph_def(graph_def, name='')	# import a serialized TensorFlow GraphDef protocol buffer, extract objects in the GraphDef as tf.Tensor
	image_path=pic
	image_data = tf.gfile.FastGFile(image_path, 'rb').read()

	with tf.Session() as sess:

	    softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')

	    predictions = sess.run(softmax_tensor, \
		     {'DecodeJpeg/contents:0': image_data})
	    # gibt prediction values in array zuerueck:

	    top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
	return str(top_k[0])


def evaluate_multipleString(pic):   
	rospack = rospkg.RosPack() 
	picPaths = pic.split(';')
	res = ''
	tf.reset_default_graph()
        with tf.gfile.FastGFile(rospack.get_path('manipulation_vision')+"/src/tf_files/retrained_graph.pb", 'rb') as f:
	    graph_def = tf.GraphDef()	## The graph-graph_def is a saved copy of a TensorFlow graph; objektinitialisierung
	    graph_def.ParseFromString(f.read())	#Parse serialized protocol buffer data into variable
	    _ = tf.import_graph_def(graph_def, name='')	# import a serialized TensorFlow GraphDef protocol buffer, extract objects in the GraphDef as tf.Tensor
	for picPath in picPaths[:-1]:	
		print picPath				   
		image_path=picPath
		image_data = tf.gfile.FastGFile(image_path, 'rb').read()
		with tf.Session() as sess:
		    softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
		    predictions = sess.run(softmax_tensor, \
			     {'DecodeJpeg/contents:0': image_data})
		    # gibt prediction values in array zuerueck:
		    top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
		    # res=res+str(top_k[0])
		    res=res+str(predictions[0][0])+';'
		#print str(predictions[0][0])
	return res

def evaluate_pose(pic):    
	rospack = rospkg.RosPack() 
	picPaths = pic.split(';')
	res = ''
	tf.reset_default_graph()
	with tf.gfile.FastGFile(rospack.get_path('manipulation_vision')+"/src/tf_files/retrained_graph_pose.pb", 'rb') as f:
	    graph_def = tf.GraphDef()	
	    graph_def.ParseFromString(f.read())	
	    _ = tf.import_graph_def(graph_def, name='')	
	for picPath in picPaths[:-1]:	
		print picPath				   
		image_path=picPath
		image_data = tf.gfile.FastGFile(image_path, 'rb').read()
		with tf.Session() as sess:
		    softmax_tensor = sess.graph.get_tensor_by_name('final_result:0')
		    predictions = sess.run(softmax_tensor, \
			     {'DecodeJpeg/contents:0': image_data})
		    # gibt prediction values in array zuerueck:
		    #print predictions
		    top_k = predictions[0].argsort()[-len(predictions[0]):][::-1]
		    # res=res+str(top_k[0])
		    res=res+str(predictions[0][2])+';'
		    res=res+str(predictions[0][1])+';'
		    res=res+str(predictions[0][0])+';'
		#print str(predictions[0])
	return res



