import tensorflow as tf
from tensorflow.keras import models
from tensorflow.keras.utils import get_custom_objects

# Convert the model
from tensorflow.python.framework.convert_to_constants import convert_variables_to_constants_v2

converter = tf.lite.TFLiteConverter.from_saved_model("mymodel")
# converter.optimizations = [tf.lite.Optimize.DEFAULT]
tflite_model = converter.convert()

# Save the model.
with open('exported_model/model.tflite', 'wb') as f:
  f.write(tflite_model)

# export frozen model according to
# https://medium.com/@sebastingarcaacosta/how-to-export-a-tensorflow-2-x-keras-model-to-a-frozen-and-optimized-graph-39740846d9eb

def my_loss_function(y_true, y_pred):
  squared_difference = tf.square(y_true[:, 0:4] - y_pred[:, 0:4])
  # print("squared_difference: ", squared_difference)
  position_loss = tf.reduce_mean(squared_difference, axis=-1) * y_true[:, 4]
  #print("position_loss: ", position_loss)
  loss = position_loss + tf.square((y_true[:, 4]*2-1) - y_pred[:, 4]) * 0.25
  #print("loss: ", loss)
  return loss

get_custom_objects().update({"my_loss_function": my_loss_function})

#path of the directory where you want to save your model
frozen_out_path = 'exported_model'
# name of the .pb file
frozen_graph_filename = 'model'

model_path = "mymodel"
model = models.load_model(model_path)

full_model = tf.function(lambda x: model(x))
full_model = full_model.get_concrete_function(
  tf.TensorSpec(model.inputs[0].shape, model.inputs[0].dtype))

# Get frozen graph def
frozen_func = convert_variables_to_constants_v2(full_model)
frozen_func.graph.as_graph_def()

layers = [op.name for op in frozen_func.graph.get_operations()]
print("-" * 60)
print("Frozen model layers: ")
for layer in layers:
  print(layer)
print("-" * 60)
print("Frozen model inputs: ")
print(frozen_func.inputs)
print("Frozen model outputs: ")
print(frozen_func.outputs)

tf.io.write_graph(graph_or_graph_def=frozen_func.graph,
                  logdir=frozen_out_path,
                  name=f"{frozen_graph_filename}.pb",
                  as_text=False)

tf.io.write_graph(graph_or_graph_def=frozen_func.graph,
                  logdir=frozen_out_path,
                  name=f"{frozen_graph_filename}.pbtxt",
                  as_text=True)

