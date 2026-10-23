import itertools
import math
import unittest

from apptirover.tracking import PersonTracker, assignment


def detection(center=.5, width=.15, score=.9):
    return [center - width / 2, .15, center + width / 2, .85, score, 0]


class TrackingTests(unittest.TestCase):
    def test_global_assignment_does_not_steal_the_only_match(self):
        costs = [[.1, .2, .85, .85], [.11, 1000, .85, .85]]
        self.assertEqual(set(assignment(costs)), {(0, 1), (1, 0)})
        for rows in range(1, 4):
            matrix = [[((row + 3) * (column + 2) % 7) / 10 for column in range(4)] for row in range(rows)]
            actual = sum(matrix[row][column] for row, column in assignment(matrix))
            expected = min(sum(matrix[row][column] for row, column in enumerate(columns))
                           for columns in itertools.permutations(range(4), rows))
            self.assertAlmostEqual(actual, expected)

    def test_ids_survive_motion_and_detection_order_changes(self):
        tracker = PersonTracker()
        first = tracker.update([detection(.25), detection(.75, score=.85)], 10)
        ids = [track['id'] for track in first]
        for index in range(1, 11):
            tracks = tracker.update([detection(.75 - index * .01, score=.85), detection(.25 + index * .01)], 10 + index * .1)
            self.assertEqual([track['id'] for track in tracks], ids)
            self.assertTrue(all(track['confirmed'] for track in tracks))
        self.assertGreater(tracks[0]['velocity'][0], 0)
        self.assertLess(tracks[1]['velocity'][0], 0)

    def test_low_scores_continue_a_track_but_cannot_create_one(self):
        tracker = PersonTracker()
        self.assertEqual(tracker.update([detection(score=.2)], 10), [])
        original = tracker.update([detection()], 10.1)[0]['id']
        continued = tracker.update([detection(.51, score=.2)], 10.2)
        self.assertEqual(continued[0]['id'], original)
        self.assertTrue(continued[0]['measured'])
        self.assertAlmostEqual(continued[0]['score'], .2)

    def test_empty_frames_predict_but_do_not_refresh_measurement_age(self):
        tracker = PersonTracker()
        tracker.update([detection(.4)], 10)
        tracker.update([detection(.42)], 10.1)
        predicted = tracker.update([], 10.2)[0]
        self.assertFalse(predicted['measured'])
        self.assertEqual(predicted['seen_at'], 10.1)
        self.assertGreater(predicted['box'][0], .42 - .075)
        self.assertEqual(tracker.update([], 10.9), [])
        self.assertGreater(tracker.update([detection()], 11)[0]['id'], 1)

    def test_reset_and_duplicate_observations_cannot_recycle_ids(self):
        tracker = PersonTracker()
        tracker.update([detection()], 10)
        self.assertEqual(tracker.update([detection()], 10), [])
        self.assertEqual(tracker.update([detection()], 9), [])
        tracker.reset()
        self.assertEqual(tracker.update([detection()], 11)[0]['id'], 2)

    def test_crossing_or_competing_people_are_marked_ambiguous(self):
        tracker = PersonTracker()
        tracker.update([detection(.4, .25), detection(.6, .25)], 10)
        for index in range(1, 5):
            tracks = tracker.update([detection(.4 + index * .015, .25), detection(.6 - index * .015, .25)], 10 + index * .1)
        self.assertTrue(all(track['ambiguous'] for track in tracks if track['measured']))

    def test_invalid_boxes_and_work_bounds(self):
        tracker = PersonTracker()
        bad = [detection(), detection(), detection(), detection()]
        bad[0][0] = math.nan
        bad[1][2] = -1
        bad[2][4] = math.inf
        bad[3][1] = 1.1
        self.assertEqual(tracker.update(bad, 10), [])
        rows = [detection(.02 + i * .014, .02) for i in range(64)]
        self.assertEqual(len(tracker.update(rows, 10.1)), 32)
        self.assertLessEqual(len(tracker.update(rows, 10.2)), 32)


if __name__ == '__main__':
    unittest.main()
