### Mine_PCR: Code for registration of LiDAR point cloud map in Jinchuan mine

- [x] Release the the point cloud registration code.
- [x] Release the the downsample code.
- [x] Release the 2D boundary map extraction code.
- [x] Release the dataset.
- [ ] Release the results.
- [ ] Hugging Face live demo.

### NEWS
- We have implemented the initial version of Mine-PCR on handheld radar measurements.
- More detailed code and a more unified version are about to be released.

### Getting Started
Mine-PCR runs on Python versions 3.10 and 3.6.
You may need to modify the path of the point cloud in `boundary.by`, `downsample-py`, and `fpfh_icp.py`.
```sh
conda create -n mine_pcr310 python=3.10
conda activate mine_pcr310
pip install -r pcd310_requirements.txt

#You may need to modify the path of the point cloud in `boundary.by`, `downsample-py`, and `fpfh_icp.py`.
python fpfh_icp.py
python downsample.py

conda create -n mine_pcr36 python=3.6
conda activate mine_pcr36
pip install -r pcd36_requirements.txt

python boundary.py

conda activate mine_pcr310
python boundary_xyz.py
python slope.py
python slope_fine.py
```
Then you can see the  running results are saved in the `output` folder.


