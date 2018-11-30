import numpy as np
import tensorflow as tf

from utils.bounding_box import BoundingBox


class TFDetector:
    def __init__(self):
        MODEL = 'ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb'
        LABELS = 'mscoco_label_map.pbtxt'
        REPORT_TIMING = False

        # read the model graph and labels from disk:
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.labels = TFDetector.read_labels(LABELS)

    @staticmethod
    def read_labels(label_file):
        with open(label_file, 'r') as fid:
            lines = fid.readlines()
        ids = lines[2::5]
        displaynames = lines[3::5]
        ids = [int(ix.strip("id: ").rstrip()) for ix in ids]
        displaynames = [d.lstrip("display_name: ")[1:].rstrip("\"\n") for d in displaynames]
        labelmap = {}
        for ix, dn in zip(ids, displaynames):
            labelmap[ix] = dn
        return labelmap

    # Given an image, returns the bounding box of the person in the image.
    # TODO: generalize to return a list of bounding boxes
    def detect_bounding_box(self, image):
        # Create a tf session.
        self.detection_graph.as_default()
        with tf.Session(graph=self.detection_graph) as sess:
            # Get handles to input and output tensors
            ops = self.detection_graph.get_operations()
            all_tensor_names = {output.name for op in ops for output in op.outputs}
            tensor_dict = {}
            for key in ['num_detections', 'detection_boxes', 'detection_scores', 'detection_classes']:
                tensor_name = key + ':0'
                if tensor_name in all_tensor_names:
                    tensor_dict[key] = self.detection_graph.get_tensor_by_name(
                        tensor_name)
            image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

            output_dict = sess.run(tensor_dict, feed_dict={image_tensor: np.expand_dims(image, 0)})

            bb = TFDetector.create_bb_from_tf_result(image, output_dict, self.labels)
            return bb

    @staticmethod
    def create_bb_from_tf_result(image, output_dict, labels, detect_thresh=0.5):
        imheight, imwidth, _ = image.shape
        for ix, score in enumerate(output_dict['detection_scores']):
            if score > detect_thresh:
                [ymin, xmin, ymax, xmax] = output_dict['detection_boxes'][ix]
                classid = output_dict['detection_classes'][ix]
                classname = labels[classid]

                if classid == 1:  # refers to person

                    dimensions = ((xmax - xmin) * imwidth, (ymax - ymin) * imheight)
                    centroid = (np.mean([xmax, xmin]), np.mean([ymax, ymin]))
                    bb = BoundingBox(dimensions, centroid)
                    return bb
