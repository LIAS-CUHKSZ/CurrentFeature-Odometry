# Open Source Checklist

Before publishing this folder publicly:

1. Choose and add a license.
2. Confirm redistribution rights for the bundled KITTI seq04 demo files and the externally hosted full data package.
3. Confirm that the full OV2SLAM-exported data package download link remains public and stable.
4. Run at least one sequence from a clean MATLAB session:

   ```matlab
   init_currentfeature_paths;
   run_kitti_currentfeature(4, true);
   ```

5. Clean or translate mojibake comments in copied MATLAB source files.
6. Consider renaming versioned research files if they are promoted out of `legacy/`.
7. Add final paper citation metadata after DOI/publication details are available.
