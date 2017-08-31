import os.path
import tensorflow as tf
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests


# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), 'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pretrained VGG Model into TensorFlow.

    conv1_1 (224x224x64), conv1_2,  pool1 (112x112x64)
    conv2_1 (112x221x128), conv2_2, pool2 (56x56x128)
    conv3_1 (56x56x256), conv3_2, conv3_3, pool3 (28x28x256)    --> layer3_out
    conv4_1 (28x28x512), conv4_2, conv4_3, pool4 (14x14x512)    --> layer4_out
    conv5_1 (14x14x512), conv5_2, conv5_3
    fc6 (7x7x512),
    fc7, dropout_1 (1x1x4096)                                   --> layer7_out

    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """

    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)

    image_input = sess.graph.get_tensor_by_name(vgg_input_tensor_name)
    keep_prob = sess.graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    layer3_out = sess.graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    layer4_out = sess.graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    layer7_out = sess.graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return image_input, keep_prob, layer3_out, layer4_out, layer7_out
tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """

    layer7_score = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, 1, 'same',
        kernel_initializer=tf.truncated_normal_initializer(stddev=1e-3), name='layer7_score')
    upscore2 = tf.layers.conv2d_transpose(layer7_score, num_classes, 4, 2, 'same',
        kernel_initializer=tf.truncated_normal_initializer(stddev=1e-3), name='upscore2')


    layer4_score = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, (1, 1), 'same',
        kernel_initializer=tf.truncated_normal_initializer(stddev=1e-3), name='layer4_score')
    fuse_pool4 = tf.add(upscore2, layer4_score, name='fuse_pool4')
    upscore_pool4 = tf.layers.conv2d_transpose(fuse_pool4, num_classes, 4, 2, 'same',
        kernel_initializer=tf.truncated_normal_initializer(stddev=1e-3), name='upscore_pool4')

    layer3_score = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, (1, 1), 'same',
        kernel_initializer=tf.truncated_normal_initializer(stddev=1e-3), name='layer3_score')
    fuse_pool3 = tf.add(upscore_pool4, layer3_score, name='fuse_pool3')
    upscore_pool8 = tf.layers.conv2d_transpose(fuse_pool3, num_classes, 16, 8, 'same',
        kernel_initializer=tf.truncated_normal_initializer(stddev=1e-3), name='upscore_pool8')

    return upscore_pool8
tests.test_layers(layers)


def optimize(nn_last_layer, label_image, learning_rate, num_classes):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param label_image: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :return: Tuple of (logits, optimizer, cross_entropy_loss)
    """
    logits = tf.reshape(nn_last_layer, (-1, num_classes), name='logits')
    labels = tf.reshape(label_image, (-1, num_classes), name='labels')

    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(
        logits=logits, labels=labels))
    optimizer = tf.train.AdamOptimizer(learning_rate).minimize(cross_entropy_loss)

    return logits, optimizer, cross_entropy_loss
tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_batches_fn, optimizer, cross_entropy_loss, input_image,
             label_image, keep_prob, learning_rate):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_batches_fn: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param optimizer: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param label_image: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    """
    idx = 0
    print("Training...")
    for i in range(epochs):
        count = 0
        total_loss = 0
        # print("Epoch {}/{}".format(i+1, epochs), end='', flush=True)
        for images, labels in get_batches_fn(batch_size):
            idx += 1
            count += 1
            summary, loss = sess.run([optimizer, cross_entropy_loss],
                feed_dict= {input_image: images, label_image: labels,
                            keep_prob: 0.8, learning_rate: 1e-4})
            total_loss += loss
            # print(" {:.4f}".format(loss), end='', flush=True)
            print(" . ", end='', flush=True)
        print("Epoch {}, Avg loss: {:.4f}".format(i+1, total_loss/count))
tests.test_train_nn(train_nn)

def run():
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    tests.test_for_kitti_dataset(data_dir)

    epochs = 50
    batch_size = 10
    label_image = tf.placeholder(tf.float32, (None, None, None, num_classes))
    learning_rate = tf.placeholder(tf.float32)

    saver = tf.train.Saver()

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')

        # Create function to get batches
        get_batches_fn = helper.gen_batch_function(os.path.join(data_dir, 'data_road/training'), image_shape)

        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # Build NN using load_vgg, layers, and optimize function
        vgg_input, vgg_keep_prob, vgg_layer3_out, vgg_layer4_out, vgg_layer7_out = load_vgg(sess, vgg_path)
        print(vgg_layer3_out.shape)
        print(vgg_layer4_out.shape)
        print(vgg_layer7_out.shape)
        fcn8s = layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes)
        logits, optimizer, cross_entropy_loss = optimize(fcn8s, label_image, learning_rate, num_classes)

        # Train NN using the train_nn function
        sess.run(tf.global_variables_initializer())
        train_nn(sess, epochs, batch_size, get_batches_fn, optimizer, cross_entropy_loss,
                 vgg_input, label_image, vgg_keep_prob, learning_rate)

        # Save the variables to disk.
        save_path = saver.save(sess, "/fcn8s.ckpt")
        print("Model saved in file: %s" % save_path)

        # Save inference data using helper.save_inference_samples
        helper.save_inference_samples(runs_dir, data_dir, sess, image_shape, logits, vgg_keep_prob, vgg_input)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    run()
