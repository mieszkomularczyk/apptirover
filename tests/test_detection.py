import unittest

import numpy as np

from apptirover.vision.detection import decode_predictions, letterbox


class DetectionGeometryTests(unittest.TestCase):
    def test_letterbox_uses_neutral_padding_and_preserves_aspect(self):
        rgb = np.zeros((480, 640, 3), dtype=np.uint8)
        rgb[:] = (255, 0, 0)
        padded, geometry = letterbox(rgb, 320)
        self.assertEqual(geometry, (320, 240, 0, 40))
        np.testing.assert_array_equal(padded[0, 0], [114, 114, 114])
        np.testing.assert_array_equal(padded[40, 0], [255, 0, 0])

    def test_nms_is_class_aware_and_boxes_map_back_to_video(self):
        # xywh in model pixels: full 640x480 source occupies y=40..280.
        predictions = np.array([[160, 160, 320, 240, .9, .1],
                                [160, 160, 320, 240, .8, .1],
                                [160, 160, 320, 240, .1, .85]], dtype=np.float32).T
        rows = decode_predictions(predictions, (320, 240, 0, 40), 2)
        self.assertEqual(len(rows), 2)
        self.assertEqual({int(row[5]) for row in rows}, {0, 1})
        np.testing.assert_allclose(np.array(rows)[:, :4], [[0, 0, 1, 1], [0, 0, 1, 1]])

    def test_end_to_end_head_clips_padding_and_rejects_invalid_rows(self):
        rows = np.array([[0, 0, 320, 320, .9, 0], [1, 1, 2, 2, .1, 1],
                         [1, 1, 2, 2, np.nan, 0], [1, 1, 2, 2, .9, 500]])
        decoded = decode_predictions(rows, (320, 240, 0, 40), 80)
        self.assertEqual(len(decoded), 1)
        np.testing.assert_allclose(decoded[0][:4], [0, 0, 1, 1])

    def test_no_detections(self):
        self.assertEqual(decode_predictions(np.zeros((84, 2100)), (320, 240, 0, 40), 80), [])

    def test_person_filter_removes_other_classes_from_both_heads(self):
        raw = np.zeros((84, 3), dtype=np.float32)
        raw[:4, :] = np.array([[80, 160, 80, 120], [240, 160, 80, 120], [80, 160, 80, 120]]).T
        raw[4, 0], raw[9, 1], raw[9, 2] = 0.8, 0.95, 0.99  # Person, bus, overlapping bus.
        rows = decode_predictions(raw, (320, 240, 0, 40), 80, class_ids=(0,))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0][5], 0)
        end_to_end = np.array([[40, 100, 120, 220, .8, 0], [200, 100, 280, 220, .95, 5]])
        rows = decode_predictions(end_to_end, (320, 240, 0, 40), 80, class_ids=(0,))
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0][5], 0)


if __name__ == "__main__":
    unittest.main()
