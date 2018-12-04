import numpy as np
import tensorflow as tf
import os
import sys
import cv2

sys.path.append("..")

from utils.bounding_box import BoundingBox


class TFDetector:
    def __init__(self, model_filepath='ssdlite_mobilenet_v2_coco_2018_05_09/frozen_inference_graph.pb',
                 label_filepath='mscoco_label_map.pbtxt'):
        print("Creating TFDetector")
        MODEL = model_filepath
        LABELS = label_filepath
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
    def detect_bounding_box(self, image,iternum, visualize=False, firstrun=False):

        if firstrun: 
            # Create a tf session.
            self.detection_graph.as_default()
            print(firstrun, "firstrun")

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
            TFDetector.update_output_dict(output_dict)

            bb = TFDetector.create_bb_from_tf_result(image, output_dict, self.labels)

            if visualize:
                # Display the image and the overlayed bounding box
                if bb is None:
                    print("No bounding box to visualize.")
                    # cv2.imshow("image", image)
                    cv2.imwrite("nobbimage" + str(iternum) + ".png", image)
                    # cv2.waitKey(0)
                    return None

                center_x, center_y = bb.centroid
                width, height = bb.dimensions

                min_x = int(center_x - width / 2)
                min_y = int(center_y - height / 2)
                print("center_x", center_x)
                max_x = int(center_x + width / 2)
                max_y = int(center_y + height / 2)
                cv2.rectangle(image, (min_x, min_y), (max_x, max_y), (0,255,0), 2)
                
                # class label viz
                label_background_color = (0, 255, 0)
                
                label_text = "this guy"
                label_text_color = (0,0,0)

                label_size = cv2.getTextSize(label_text, cv2.FONT_HERSHEY_SIMPLEX, 1, 1)[0]
                label_left = min_x
                label_top = min_y - label_size[1]
                if (label_top < 1):
                    label_top = 1
                label_right = label_left + label_size[0]
                label_bottom = label_top + label_size[1]
                cv2.rectangle(image, (label_left - 1, label_top - 1), (label_right + 1, label_bottom + 1),
                              label_background_color, -1)

                # label text above the box
                cv2.putText(image, label_text, (label_left, label_bottom), cv2.FONT_HERSHEY_SIMPLEX, 1, label_text_color, 2,cv2.LINE_AA)

                cv2.imwrite("bbimage" + str(iternum) + ".png", image)
                # cv2.imshow("image", image)
                # cv2.waitKey(0) 
            return [bb]  # FIXME: support lists, but right now it's only a list of length 1



    @staticmethod
    def update_output_dict(original_dict):
        original_dict['num_detections'] = int(original_dict['num_detections'][0])
        original_dict['detection_classes'] = original_dict['detection_classes'][0].astype(np.uint8)
        original_dict['detection_boxes'] = original_dict['detection_boxes'][0]
        original_dict['detection_scores'] = original_dict['detection_scores'][0]

    @staticmethod
    def create_bb_from_tf_result(image, output_dict, labels, detect_thresh=0.5):
        imheight, imwidth, _ = image.shape
        for ix, score in enumerate(output_dict['detection_scores']):
            #max_score = max(score)  # FIXME: this is bad
            if score > detect_thresh:
                [ymin, xmin, ymax, xmax] = output_dict['detection_boxes'][ix]
                classid = output_dict['detection_classes'][ix]
                classname = labels[classid]

                if classid == 1:  # refers to person

                    dimensions = ((xmax - xmin) * imwidth, (ymax - ymin) * imheight)
                    centroid = (np.mean([xmax * imwidth, xmin * imwidth]), np.mean([ymax * imheight, ymin * imheight]))
                    bb = BoundingBox(dimensions, centroid)
                    return bb

if __name__=='__main__':
    pass
    # print("hello world2")
    # my_detector = TFDetector()
    # loaded_image = cv2.imread('../../data/thi.png')
    # detect_bb = my_detector.detect_bounding_box(loaded_image, visualize=True)
    # print(detect_bb)

    
