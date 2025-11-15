import os
import tempfile
import unittest
import numpy as np

from self_driving_robot import Grid


class TestGridSaveLoad(unittest.TestCase):
    def test_save_load_roundtrip(self):
        g = Grid()
        # create a fake occupied cell
        cx, cy = g.size//2 + 5, g.size//2
        g.logodds[cx, cy] = g.occ_threshold + 1.0
        g.grid[g.logodds > g.occ_threshold] = 1

        fd, path = tempfile.mkstemp(suffix='.npz')
        os.close(fd)
        try:
            g.save(path)
            g2 = Grid.load(path)
            self.assertEqual(g2.grid.shape, g.grid.shape)
            # the occupied cell should still be occupied
            self.assertEqual(g2.grid[cx, cy], 1)
        finally:
            try:
                os.remove(path)
            except Exception:
                pass


class TestGridInflation(unittest.TestCase):
    def test_inflate_expands_obstacle(self):
        """Test that inflation expands a single occupied cell to a circular region."""
        g = Grid()
        cx, cy = g.size//2, g.size//2
        g.grid[cx, cy] = 1

        # Inflate by 1 cell
        g.inflate(g.res * 1.5)

        # Check that nearby cells are now occupied
        # At least the cardinal neighbors should be occupied
        self.assertEqual(g.grid[cx+1, cy], 1, "Right neighbor should be occupied")
        self.assertEqual(g.grid[cx-1, cy], 1, "Left neighbor should be occupied")
        self.assertEqual(g.grid[cx, cy+1], 1, "Top neighbor should be occupied")
        self.assertEqual(g.grid[cx, cy-1], 1, "Bottom neighbor should be occupied")

    def test_inflate_zero_radius(self):
        """Test that zero radius inflation does nothing."""
        g = Grid()
        cx, cy = g.size//2, g.size//2
        g.grid[cx, cy] = 1
        original = np.array(g.grid, copy=True)

        g.inflate(0.0)

        np.testing.assert_array_equal(g.grid, original)


class TestGridRosExport(unittest.TestCase):
    def test_export_ros_map_creates_files(self):
        """Test that ROS map export creates both PGM and YAML files."""
        g = Grid()
        # Add a simple obstacle
        cx, cy = g.size//2 + 5, g.size//2
        g.logodds[cx, cy] = g.occ_threshold + 1.0
        g.grid[g.logodds > g.occ_threshold] = 1

        fd_pgm, pgm_path = tempfile.mkstemp(suffix='.pgm')
        fd_yaml, yaml_path = tempfile.mkstemp(suffix='.yaml')
        os.close(fd_pgm)
        os.close(fd_yaml)
        try:
            g.export_ros_map(pgm_path, yaml_path)

            # Check that both files were created
            self.assertTrue(os.path.exists(pgm_path), "PGM file should exist")
            self.assertTrue(os.path.exists(yaml_path), "YAML file should exist")

            # Check YAML has expected content
            with open(yaml_path, 'r') as f:
                yaml_content = f.read()
            self.assertIn("image:", yaml_content)
            self.assertIn("resolution:", yaml_content)
            self.assertIn("origin:", yaml_content)
        finally:
            try:
                os.remove(pgm_path)
                os.remove(yaml_path)
            except Exception:
                pass


if __name__ == '__main__':
    unittest.main()
